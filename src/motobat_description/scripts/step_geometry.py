#!/usr/bin/python3
"""
Exact geometry out of a STEP assembly, with nothing but numpy. (task 2.4)

Task 2.4 needs the inspector flange's sensor frames as numbers derived from
the CAD, not read off a screen (project_plan.md §4A.4: a 2 deg error corrupts
every registration downstream). STEP is plain text, and the analytic surfaces
in it (planes, cylinders, cones) carry their exact placement. This module reads
them per part and places them in the assembly frame, so features such as bolt
holes, the dowel hole and sensor faces can be located to the precision of the CAD.

    step_geometry.py cad/inspector/inspector_flange.step            # summary per part

Library use (step_frames.py builds on it):
    asm = StepAssembly(path)
    asm.parts                          -> {name: Part}
    part.cylinders, part.planes        -> features in ASSEMBLY coordinates, metres
"""
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

REF = re.compile(r'#(\d+)')


def _split_args(s):
    """Split a STEP argument list at top-level commas."""
    out, depth, cur, q = [], 0, [], False
    for ch in s:
        if ch == "'" :
            q = not q
        if not q:
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            elif ch == ',' and depth == 0:
                out.append(''.join(cur).strip())
                cur = []
                continue
        cur.append(ch)
    if cur:
        out.append(''.join(cur).strip())
    return out


def _nums(s):
    return [float(v) for v in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', s)]


@dataclass
class Cylinder:
    origin: np.ndarray      # a point on the axis, m (assembly frame)
    axis: np.ndarray        # unit
    radius: float           # m
    face: int               # ADVANCED_FACE id


@dataclass
class Plane:
    origin: np.ndarray
    normal: np.ndarray      # surface normal of the PLANE entity (not the face orientation)
    face: int


@dataclass
class Part:
    name: str
    to_assembly: np.ndarray                      # 4x4, part frame -> assembly frame
    cylinders: list = field(default_factory=list)
    planes: list = field(default_factory=list)
    points: np.ndarray = None                    # every topological vertex, assembly frame


class StepAssembly:
    def __init__(self, path):
        text = open(path, encoding='latin-1').read()
        data = text[text.index('DATA;') + 5:text.index('ENDSEC;', text.index('DATA;'))]
        self.ent = {}
        for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)', data, re.S):
            self.ent[int(m.group(1))] = ' '.join(m.group(2).split())
        self.scale = self._length_scale()
        self._by_type = defaultdict(list)
        for k, v in self.ent.items():
            t = v.split('(', 1)[0] if not v.startswith('(') else 'COMPLEX'
            self._by_type[t].append(k)
        self.parts = self._parts()

    # ---- primitives --------------------------------------------------------------
    def args(self, i):
        v = self.ent[i]
        return _split_args(v[v.index('(') + 1:v.rindex(')')])

    def kind(self, i):
        v = self.ent[i]
        return 'COMPLEX' if v.startswith('(') else v.split('(', 1)[0]

    def point(self, i):
        return np.array(_nums(self.args(i)[1])) * self.scale

    def direction(self, i):
        d = np.array(_nums(self.args(i)[1]))
        return d / np.linalg.norm(d)

    def placement(self, i):
        """AXIS2_PLACEMENT_3D -> 4x4 (columns: x, y, z axes, origin)."""
        a = self.args(i)
        o = self.point(int(a[1][1:]))
        z = self.direction(int(a[2][1:])) if a[2] != '$' else np.array([0, 0, 1.0])
        x = self.direction(int(a[3][1:])) if len(a) > 3 and a[3] != '$' else np.array([1.0, 0, 0])
        x = x - z * (x @ z)
        x /= np.linalg.norm(x)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, o
        return T

    def _length_scale(self):
        """Metres per file length unit, from the SI_UNIT / CONVERSION_BASED_UNIT."""
        for v in self.ent.values():
            if 'LENGTH_UNIT' in v and 'SI_UNIT' in v:
                m = re.search(r'SI_UNIT\((\.\w+\.|\$),\.METRE\.\)', v)
                if m:
                    return {'.MILLI.': 1e-3, '.CENTI.': 1e-2, '$': 1.0}[m.group(1)]
        raise ValueError('no SI length unit found (conversion-based units not handled)')

    def reachable(self, root):
        seen, stack = set(), [root]
        while stack:
            i = stack.pop()
            if i in seen or i not in self.ent:
                continue
            seen.add(i)
            stack.extend(int(r) for r in REF.findall(self.ent[i]))
        return seen

    # ---- assembly structure ------------------------------------------------------
    def _parts(self):
        # PRODUCT -> name; PRODUCT_DEFINITION_FORMATION -> product; PRODUCT_DEFINITION -> formation
        prod_name = {i: _split_args(self.ent[i][8:-1])[1].strip("'") for i in self._by_type['PRODUCT']}
        formation = {}
        for t in ('PRODUCT_DEFINITION_FORMATION', 'PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE'):
            for i in self._by_type[t]:
                formation[i] = int(self.args(i)[2][1:])
        pdef = {i: formation[int(self.args(i)[2][1:])] for i in self._by_type['PRODUCT_DEFINITION']}
        # PRODUCT_DEFINITION_SHAPE -> its definition (a PRODUCT_DEFINITION or a NAUO)
        pds = {i: int(self.args(i)[2][1:]) for i in self._by_type['PRODUCT_DEFINITION_SHAPE']}
        # SHAPE_DEFINITION_REPRESENTATION(pds, shape_rep): the part's top representation
        rep_of_pdef = {}
        for i in self._by_type['SHAPE_DEFINITION_REPRESENTATION']:
            a = self.args(i)
            d = pds[int(a[0][1:])]
            if d in pdef:
                rep_of_pdef[d] = int(a[1][1:])
        # geometry reps hang off the top rep through SHAPE_REPRESENTATION_RELATIONSHIP
        geo_of_rep = defaultdict(list)
        for i in self._by_type['SHAPE_REPRESENTATION_RELATIONSHIP']:
            a = self.args(i)
            r1, r2 = int(a[2][1:]), int(a[3][1:])
            geo_of_rep[r1].append(r2)
            geo_of_rep[r2].append(r1)
        # component placements: CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(rel, pds_of_nauo)
        nauo_child = {i: int(self.args(i)[4][1:]) for i in self._by_type['NEXT_ASSEMBLY_USAGE_OCCURRENCE']}
        placed = {}
        for i in self._by_type['CONTEXT_DEPENDENT_SHAPE_REPRESENTATION']:
            rel_id, pds_id = (int(x[1:]) for x in self.args(i)[:2])
            rel = self.ent[rel_id]
            idt = int(re.search(r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\(#(\d+)\)', rel).group(1))
            a1, a2 = (int(x[1:]) for x in self.args(idt)[2:4])
            # ITEM_DEFINED_TRANSFORMATION maps placement a1 (child) onto a2 (parent)
            placed[nauo_child[pds[pds_id]]] = self.placement(a2) @ np.linalg.inv(self.placement(a1))
        parts = {}
        for d, rep in rep_of_pdef.items():
            if d not in placed:          # the top-level assembly itself
                continue
            name = prod_name[pdef[d]]
            part = Part(name, placed[d])
            roots = [g for g in geo_of_rep[rep]] + [rep]
            ids = set()
            for r in roots:
                ids |= self.reachable(r)
            self._collect(part, ids)
            parts[name] = part
        self._adopt_orphan_shells(parts)
        return parts

    def _adopt_orphan_shells(self, parts):
        """Fusion sometimes writes a body as a CLOSED_SHELL that no solid or
        representation references (inspector_flange.step, 1 Nov 2026: 'Mount v3',
        twice). A strict reader drops it. If exactly one part came out empty and
        the orphans are copies of one shell, give that part one copy, and say so."""
        referenced = set()
        for v in self.ent.values():
            if 'CLOSED_SHELL' not in v:
                referenced.update(int(r) for r in REF.findall(v))
        orphans = [s for s in self._by_type['CLOSED_SHELL'] if s not in referenced]
        empty = [p for p in parts.values() if len(p.points) == 0]
        self.orphan_note = None
        if not orphans or not empty:
            return
        verts = []
        for s in orphans:
            ids = self.reachable(s)
            verts.append(np.unique(np.round([self.point(int(self.args(i)[1][1:])) for i in ids
                                             if self.kind(i) == 'VERTEX_POINT'], 9), axis=0))
        copies = all(v.shape == verts[0].shape and np.allclose(v, verts[0]) for v in verts)
        if len(empty) != 1 or not copies:
            raise ValueError(f'{len(orphans)} unreferenced shells and {len(empty)} empty parts: '
                             'cannot tell which geometry belongs to which part')
        self._collect(empty[0], self.reachable(orphans[0]))
        self.orphan_note = (f"'{empty[0].name}' has no linked geometry; adopted unreferenced "
                            f'CLOSED_SHELL #{orphans[0]} ({len(orphans)} identical copies in the file). '
                            'The model is fine in Fusion; this is a quirk of the export, and only strict STEP readers drop the body.')

    def _collect(self, part, ids):
        T = part.to_assembly

        def P(p):
            return T[:3, :3] @ p + T[:3, 3]

        def D(d):
            return T[:3, :3] @ d

        for i in ids:
            k = self.kind(i)
            if k != 'ADVANCED_FACE':
                continue
            surf = int(self.args(i)[2][1:])
            sk = self.kind(surf)
            a = self.args(surf)
            if sk == 'CYLINDRICAL_SURFACE':
                pl = self.placement(int(a[1][1:]))
                part.cylinders.append(Cylinder(P(pl[:3, 3]), D(pl[:3, 2]), float(a[2]) * self.scale, i))
            elif sk == 'PLANE':
                pl = self.placement(int(a[1][1:]))
                part.planes.append(Plane(P(pl[:3, 3]), D(pl[:3, 2]), i))
        # Topological vertices only. Other CARTESIAN_POINTs are construction data
        # (a plane's or line's anchor point, a circle's centre) and can lie far
        # outside the part, which would wreck any bounding box.
        pts = [self.point(int(self.args(i)[1][1:])) for i in ids if self.kind(i) == 'VERTEX_POINT']
        part.points = (np.array(pts) @ T[:3, :3].T + T[:3, 3]) if pts else np.zeros((0, 3))


def main():
    asm = StepAssembly(sys.argv[1])
    np.set_printoptions(precision=4, suppress=True)
    print(f'length unit: {asm.scale} m')
    if asm.orphan_note:
        print('NOTE:', asm.orphan_note)
    for name, p in asm.parts.items():
        lo, hi = p.points.min(0), p.points.max(0)
        print(f'\n== {name}')
        print(f'  placement in assembly: origin {p.to_assembly[:3, 3] * 1e3} mm')
        print(f'  rotation:\n{p.to_assembly[:3, :3]}')
        print(f'  bbox (mm): {lo * 1e3} .. {hi * 1e3}')
        radii = defaultdict(int)
        for c in p.cylinders:
            radii[round(c.radius * 2e3, 3)] += 1
        print(f'  {len(p.planes)} planar faces; cylindrical faces by diameter (mm): '
              f'{dict(sorted(radii.items()))}')


if __name__ == '__main__':
    main()

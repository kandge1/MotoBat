"""Diagnostics for oblique (arm) views: why a clean module raises findings.

    python -m defectgen.probe_arm --stage pre_weld [--fixture]

Runs detection on a clean module through the arm's 1.5 view plan and prints:
per capture, the registration error against the known fixture placement and
the misregistration the residual pass assumed; the housing strips each capture
saw (`detect.datum_strips`: whole or cut by the view, and what each says about
X, Y and yaw); how completely each busbar and terminal outline was in view
(golden outline extent against the face's design extent); then every finding
with its capture.
A development tool for the arm-hardening step (project_plan.md §4B.6), kept in
the package so the evidence behind that step can be produced again.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from . import detect as DT
from . import regress as R
from .module import busbar_length


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m defectgen.probe_arm")
    ap.add_argument("--stage", default="pre_weld")
    ap.add_argument("--fixture", action="store_true")
    ap.add_argument("--setup", default="arm")
    args = ap.parse_args()
    cfg = R.CFG
    rows = []
    strips = {}
    orig_release = DT.Measurer.release
    orig_fit = DT.housing_fit

    def housing_fit(cap, gold, ref, p):
        t, info = orig_fit(cap, gold, ref, p)
        strips[cap.name] = list(info.get("strips", []))
        return t, info

    def release(self):
        for kind, d in (("busbar", self.busbars), ("terminal", self.terminals)):
            for key, pair in d.items():
                g = pair.gold.rect
                if g is None:
                    continue
                if kind == "busbar":
                    b = next(b for b in self.ref.labels.busbars if b.index == key)
                    nom = (busbar_length(cfg, b.kind), cfg.busbar_w)
                else:
                    nom = (cfg.terminal_d, cfg.terminal_w)
                ext = sorted((g.ex, g.ey), reverse=True)
                rows.append({"cap": self.cap.name, "kind": kind, "key": str(key), "support": pair.support,
                             "gold_ext": [round(x, 2) for x in ext], "nominal": sorted(nom, reverse=True),
                             "short_by": [round(n - e, 2) for n, e in zip(sorted(nom, reverse=True), ext)],
                             "spacing": round(self.gold.spacing, 3),
                             "scan_vs_gold": None if pair.scan is None or pair.scan.rect is None else
                             [round(pair.scan.rect.cx - g.cx, 3), round(pair.scan.rect.cy - g.cy, 3),
                              round(pair.scan.rect.yaw_deg - g.yaw_deg, 3)]})
        orig_release(self)

    DT.Measurer.release = release
    DT.housing_fit = housing_fit
    rep, ev, td = R.run(args.stage, [], [], fixture=R.FIXTURE if args.fixture else None, setup=args.setup)
    print(f"{args.stage} clean ({'fixture' if args.fixture else 'placed'}) via {args.setup}: "
          f"{len(rep.findings)} findings, {td:.0f} s")
    T = np.eye(4)
    if args.fixture:
        T[:3, :3] = R._rot(R.FIXTURE[3], R.FIXTURE[4])
        T[:3, 3] = R.FIXTURE[:3]
    corners = np.array([[x, y, 100.0, 1.0] for x in (-172.0, 172.0) for y in (-75.0, 75.0)])
    print("\n# registration error: |t_final @ fixture - I| at the module's top corners, mm (max over corners)")
    for c in rep.captures:
        icp = c.get("icp", {})
        t = np.array(icp.get("transform", np.eye(4)))
        e = (corners @ (t @ T).T - corners)[:, :3]
        print(f"  {c['name']:28s} err xyz max {np.round(np.abs(e).max(axis=0), 3)}  axes {icp.get('registered_axes')}")
    print("\n# housing strips (offset = golden - scan across the strip, mm; dyaw deg; edge = long edge in view, mm)")
    for name, obs in strips.items():
        for o in obs:
            print(f"  {name:28s} {o['strip']:7s} {'whole' if o['whole'] else 'cut  '} offset {o['offset']:+.4f} "
                  f"dyaw {o['dyaw']:+.4f} edge {o['edge']:6.1f} spacing {o['spacing']:.3f}")
        if not obs:
            print(f"  {name:28s} none measured (housing unseen, or sampling too coarse for outlines)")
    print("\n# captures")
    for c in rep.captures:
        icp = c.get("icp", {})
        res = c.get("residual", {})
        print(f"  {c['name']:28s} pts {c['points']:9d} kept_fixture {icp.get('kept_fixture_pose')} "
              f"datum {str(icp.get('datum'))[:40]:40s} t {np.round(icp.get('translation_mm', []), 3)} "
              f"misreg {res.get('misregistration_mm')} noise {res.get('noise_mm', 0):.4f} "
              f"regions {res.get('regions')}")
    print("\n# outline completeness (short_by = design extent - golden outline extent, mm)")
    for r in rows:
        flag = "CUT" if max(r["short_by"]) > 3 * r["spacing"] + 0.3 else ""
        print(f"  {r['cap']:28s} {r['kind']:8s} {r['key']:12s} sup {r['support']:7d} spacing {r['spacing']:.3f} "
              f"short_by {r['short_by']} scan-gold {r['scan_vs_gold']} {flag}")
    print("\n# findings")
    for f in rep.findings:
        m = f.measured
        print(f"  {f.cls:17s} {f.magnitude:8.3f} [{f.capture}] {f.feature[:45]:45s} "
              f"{tuple(round(x, 1) for x in f.location)} "
              + (f"area {m.get('area_mm2', 0):.2f} peak {m.get('peak_mm', 0):.3f}" if "area_mm2" in m else
                 json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()
                             if k in ("dx", "dy", "dyaw_deg", "dz", "offset", "drift", "sagitta", "width_change",
                                      "crown_change", "short")})))


if __name__ == "__main__":
    main()

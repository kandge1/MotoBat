#!/usr/bin/env python3
"""
A defectgen module as a Gazebo model on a pallet that rides the line (task 2.6). Runs in the
project venv (trimesh, Open3D), not ROS.

    mb-venv && python src/motobat_bringup/scripts/gazebo_module.py \\
        data/arm_scans/weld_walls/module --stage post_weld --out data/gazebo/models

reads <module>/<stage>/<group>.stl (a defectgen --out folder, module frame = pallet datum, mm)
and writes <out>/motobat_pallet/{model.config, model.sdf, meshes/<group>.stl}:

  * one mesh per body group, in metres (no <scale> needed), coloured by material, minus the
    groups the stage's inspection leaves out (post_weld: CMC and cover, scan.without_groups);
    for collision, one box around the module (mesh-on-mesh contacts stall the physics);
  * groups over --max-faces triangles decimated (Open3D quadric): a defectgen module is ~2.5 M
    triangles, mostly weld beads and dented walls. Gazebo is for motion and data flow; the
    sub-mm detail stays in the numpy sensor models (plan §3.7.8). The kept fraction is printed;
  * the pallet (A: 25 mm margin, 20 mm thick, view_reach.py PALLET_*), with the module datum at
    its top face, on two 50 mm skids along its long edges: they ride the belt 50 mm below the
    pallet, the clearance 2.5's conveyor placeholder was planned with (conveyor_lift 0.05);
  * a prismatic joint `pallet_x` along world +X carrying it, driven by Gazebo's
    JointPositionController on /pallet/x (gz.msgs.Double, metres from the model's origin) and
    reported by JointStatePublisher on /world/<world>/model/motobat_pallet/joint_state.

The model's origin is where the pallet datum sits with pallet_x = 0: the world file places it.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh

PALLET_MARGIN, PALLET_T = 0.025, 0.020       # m (A), as view_reach.py
SKID_H, SKID_W = 0.050, 0.030                # m (A): the pallet rides the belt 50 mm below its top,
                                             # as view_reach.py's conveyor_lift 0.05 (base A)
COLOUR = {   # rough appearance only (RGB 0-1)
    'cells': (0.20, 0.35, 0.65), 'terminals_pos': (0.75, 0.75, 0.78), 'terminals_neg': (0.80, 0.50, 0.30),
    'busbars': (0.85, 0.50, 0.25), 'welds': (0.55, 0.35, 0.20), 'carrier': (0.12, 0.12, 0.12),
    'pads': (0.90, 0.80, 0.30), 'end_plates': (0.70, 0.72, 0.75), 'side_plates': (0.70, 0.72, 0.75),
    'cooling_plate': (0.60, 0.62, 0.65), 'cmc': (0.10, 0.45, 0.20), 'cover': (0.65, 0.67, 0.70),
    'fod': (0.90, 0.10, 0.10)}
LEFT_OUT = {'post_weld': ('cmc', 'cover')}   # defectgen scan.without_groups


def decimate(mesh: trimesh.Trimesh, target: int) -> trimesh.Trimesh:
    import open3d as o3d
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(mesh.vertices),
                                  o3d.utility.Vector3iVector(mesh.faces))
    m = m.simplify_quadric_decimation(target_number_of_triangles=target)
    return trimesh.Trimesh(np.asarray(m.vertices), np.asarray(m.triangles), process=True)


def material(rgb):
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular></material>')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('module', type=Path, help='a defectgen --out folder')
    ap.add_argument('--stage', default='post_weld')
    ap.add_argument('--out', type=Path, default=Path('data/gazebo/models'))
    ap.add_argument('--max-faces', type=int, default=60_000, help='decimate any group above this')
    ap.add_argument('--travel', type=float, nargs=2, default=(-2.0, 2.0), metavar=('LO', 'HI'),
                    help='pallet_x limits, m')
    a = ap.parse_args()

    model = a.out / 'motobat_pallet'
    (model / 'meshes').mkdir(parents=True, exist_ok=True)
    for old in (model / 'meshes').glob('*.stl'):
        old.unlink()
    visuals, lo, hi = [], np.full(3, np.inf), np.full(3, -np.inf)
    for f in sorted((a.module / a.stage).glob('*.stl')):
        g = f.stem
        if g in LEFT_OUT.get(a.stage, ()):
            continue
        m = trimesh.load(f, force='mesh')
        n0 = len(m.faces)
        if n0 > a.max_faces:
            m = decimate(m, a.max_faces)
        m.apply_scale(0.001)
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
        m.export(model / 'meshes' / f'{g}.stl')
        print(f'  {g:16s} {n0:9d} -> {len(m.faces):7d} triangles')
        uri = f'model://motobat_pallet/meshes/{g}.stl'
        visuals.append(f"""
      <visual name="{g}"><geometry><mesh><uri>{uri}</uri></mesh></geometry>{material(COLOUR.get(g, (0.6, 0.6, 0.6)))}</visual>""")
    pl = hi[:2] - lo[:2] + 2 * PALLET_MARGIN
    skids = ''
    for side in (-1, 1):
        y = side * (pl[1] / 2 - SKID_W / 2)
        box = (f'<pose>0 {y:.4f} {-PALLET_T - SKID_H / 2} 0 0 0</pose>'
               f'<geometry><box><size>{pl[0]:.4f} {SKID_W} {SKID_H}</size></box></geometry>')
        skids += (f'\n      <visual name="skid{side:+d}">{box}{material((0.3, 0.3, 0.32))}</visual>'
                  f'\n      <collision name="skid{side:+d}">{box}</collision>')
    sdf = f"""<?xml version="1.0"?>
<!-- GENERATED by motobat_bringup/scripts/gazebo_module.py from {a.module}/{a.stage}. Do not hand-edit. -->
<sdf version="1.9">
  <model name="motobat_pallet">
    <link name="base"/>
    <joint name="fixed_to_world" type="fixed"><parent>world</parent><child>base</child></joint>
    <link name="carriage">
      <inertial><mass>8.0</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="pallet">
        <pose>0 0 {-PALLET_T / 2} 0 0 0</pose>
        <geometry><box><size>{pl[0]:.4f} {pl[1]:.4f} {PALLET_T}</size></box></geometry>
        {material((0.35, 0.45, 0.6))}
      </visual>
      <collision name="pallet">
        <pose>0 0 {-PALLET_T / 2} 0 0 0</pose>
        <geometry><box><size>{pl[0]:.4f} {pl[1]:.4f} {PALLET_T}</size></box></geometry>
      </collision>
      <!-- One box for the whole module: mesh-on-mesh contacts with 60 k-triangle groups slowed
           Gazebo to a real-time factor of 0.07 (25 Nov). The arm's clearance is MoveIt's job. -->
      <collision name="module_envelope">
        <pose>{(lo[0] + hi[0]) / 2:.4f} {(lo[1] + hi[1]) / 2:.4f} {(lo[2] + hi[2]) / 2:.4f} 0 0 0</pose>
        <geometry><box><size>{hi[0] - lo[0]:.4f} {hi[1] - lo[1]:.4f} {hi[2] - lo[2]:.4f}</size></box></geometry>
      </collision>{skids}{''.join(visuals)}
    </link>
    <joint name="pallet_x" type="prismatic">
      <parent>base</parent><child>carriage</child>
      <axis><xyz>1 0 0</xyz><limit><lower>{a.travel[0]}</lower><upper>{a.travel[1]}</upper>
        <effort>1000</effort><velocity>1.0</velocity></limit></axis>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>pallet_x</joint_name>
      <topic>/pallet/x</topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>20</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
      <cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>pallet_x</joint_name>
    </plugin>
  </model>
</sdf>
"""
    (model / 'model.sdf').write_text(sdf)
    (model / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model><name>motobat_pallet</name><version>1.0</version>'
        '<sdf version="1.9">model.sdf</sdf><description>defectgen module on a line pallet '
        '(task 2.6, generated)</description></model>\n')
    print(f'wrote {model}  (module {np.round((hi - lo) * 1000, 1).tolist()} mm)')


if __name__ == '__main__':
    main()

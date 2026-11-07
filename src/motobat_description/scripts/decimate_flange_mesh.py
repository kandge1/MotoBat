"""
A light visual mesh for the inspector flange. (task 2.4)

inspector_flange.stl (Fusion export) is ~2.1 M triangles / 99 MB: too heavy for
RViz, Gazebo and MoveIt. This writes a quadric-decimated copy and measures how
far it strays from the original, so the simplification is a number, not a hope.
Collision does not use this mesh: it uses simple boxes (step_frames.py).

Needs Open3D, so it runs in the project venv:
    mb-venv
    python -I src/motobat_description/scripts/decimate_flange_mesh.py [target_triangles]
"""
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

HERE = Path(__file__).resolve().parent.parent
SRC = HERE / 'meshes' / 'inspector' / 'inspector_flange.stl'
DST = HERE / 'meshes' / 'inspector' / 'inspector_flange_visual.stl'


def read_binary_stl(path):
    n = int(np.fromfile(path, dtype='<u4', count=1, offset=80)[0])
    rec = np.fromfile(path, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                      count=n, offset=84)
    return rec['v'].reshape(-1, 3).astype(np.float64)


def main():
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 60000
    v = read_binary_stl(SRC)
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),
                                     o3d.utility.Vector3iVector(np.arange(len(v)).reshape(-1, 3)))
    mesh = mesh.merge_close_vertices(1e-6).remove_duplicated_triangles().remove_degenerate_triangles()
    n0 = len(mesh.triangles)
    small = mesh.simplify_quadric_decimation(target).remove_degenerate_triangles()
    small.compute_triangle_normals()

    # Deviation, both ways, in mm (the STL is in mm): decimated -> original, original -> decimated
    def dist(a, b, n=200000):
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(b))
        pts = np.asarray(a.sample_points_uniformly(n).points, dtype=np.float32)
        return scene.compute_distance(o3d.core.Tensor(pts)).numpy()

    d1, d2 = dist(small, mesh), dist(mesh, small)
    o3d.io.write_triangle_mesh(str(DST), small, write_ascii=False)
    print(f'{n0} -> {len(small.triangles)} triangles, {DST.stat().st_size / 1e6:.1f} MB: {DST.name}')
    for name, d in (('decimated -> original', d1), ('original -> decimated', d2)):
        print(f'  {name}: median {np.median(d):.3f} mm, p99 {np.percentile(d, 99):.3f} mm, max {d.max():.3f} mm')


if __name__ == '__main__':
    main()

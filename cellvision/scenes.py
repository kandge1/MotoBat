"""The two S1 scenes as boxes, in world millimetres with S1's pallet stop at the origin. (task 3.2)

Geometry from the CAD as exported (module_spec.md §7; part STL bounds, 26 Nov 2026):
    cell can 26.5 (x) x 148 (y) x 91, standing on the 4 mm cooling plate: top 95 mm above the
    pallet datum; pads 1.5 mm between cells; end plates 10.25 thick, 101 tall; cooling plate
    355 x 151 x 4; cell centres at x = (k - 5.5) * 28.0 mm.
    + terminal 18.5 x 22 x 6 at y = +57.5, - terminal 18.5 x 38.5 x 6 at y = -49.2 (cell frame;
    3P4S flips every group of three, so the sign of y alternates with the group).
Line (line_layout.xacro): pallet top 900 mm above the floor, S1 stop at x = 0, line along +x;
the infeed pick point at (-450, -450) mm with the cell's top at the same 995 mm (A).
Reflectances under the ring light (A, mono, 0..1): can lid 0.70, terminals 0.50, end plate 0.45,
pads 0.12, cooling plate 0.30, infeed pad / pallet 0.25 (the floor), vertical sides 0.08.
"""
from __future__ import annotations

import numpy as np

from .camera import Box, Camera, look_down

PALLET_TOP = 900.0
CELL = (26.5, 148.0, 91.0)
CELL_TOP = PALLET_TOP + 95.0
PITCH = 28.0
INFEED = (-450.0, -450.0)
CAMERA_Z = 1600.0                     # both cameras, above the GP7's swept envelope (1.37 m)


def infeed_camera(f=16.0):
    t, R = look_down((INFEED[0], INFEED[1], CAMERA_Z), yaw=np.pi / 2)   # image x along the cell's length
    return Camera(f=f, t=t, R=R)


def stack_camera(f=12.0):
    t, R = look_down((0.0, 0.0, CAMERA_Z), yaw=0.0)                     # image x along the module
    return Camera(f=f, t=t, R=R)


def cell(x, y, yaw, flip=False, top=CELL_TOP, size=CELL):
    """A cell can and its two terminals."""
    sx, sy, h = size
    out = [Box(x, y, yaw, sx, sy, top - h, top, 0.70)]
    sgn = -1.0 if flip else 1.0
    c, s = np.cos(yaw), np.sin(yaw)
    for ly, ty, reflect in ((sgn * 57.5, 22.0, 0.5), (-sgn * 49.2, 38.5, 0.5)):
        out.append(Box(x - s * ly, y + c * ly, yaw, 18.5, ty, top, top + 6.0, reflect))
    return out


def infeed_scene(x, y, yaw, dz=0.0, size=CELL):
    """One cell standing on the infeed pad (the floor of the scene)."""
    return cell(x, y, yaw, top=CELL_TOP + dz, size=size)


def stack_scene(poses, end_plate_a=True):
    """Cells 1..len(poses) placed: poses = [(x, y, yaw, dz)] (world mm, rad), pads between them.
    Pads sit midway between neighbours at their nominal pitch positions, 0.5 mm below the cell
    tops (A: compressible foam), so a cell's own top always wins where they overlap."""
    boxes = [Box(0.0, 0.0, 0.0, 355.0, 151.0, PALLET_TOP, PALLET_TOP + 4.0, 0.30),
             Box(0.0, 0.0, 0.0, 405.0, 201.0, PALLET_TOP - 20.0, PALLET_TOP, 0.25)]
    if end_plate_a:
        boxes.append(Box(-172.375, 0.0, 0.0, 10.25, 148.0, PALLET_TOP + 4.0, PALLET_TOP + 105.0, 0.45))
    for k, (x, y, yaw, dz) in enumerate(poses):
        boxes += cell(x, y, yaw, flip=(k // 3) % 2 == 1, top=CELL_TOP + dz)
        if k < 11:
            boxes.append(Box((k - 5.5) * PITCH + 14.0, 0.0, 0.0, 1.5, 148.0, PALLET_TOP + 4.0, CELL_TOP - 0.5, 0.12))
    return boxes


def nominal_x(k):
    """Cell k (0-based) centre along the module."""
    return (k - 5.5) * PITCH


def roi_for(cam, x, y, half=(100.0, 100.0), z=(PALLET_TOP, CELL_TOP + 6.0)):
    """Pixel window covering a (2 half) box round (x, y) between heights z."""
    pts = np.array([[x + sx * half[0], y + sy * half[1], zz] for sx in (-1, 1) for sy in (-1, 1) for zz in z])
    uv = cam.project(pts)
    u0, v0 = np.floor(uv.min(0)).astype(int)
    u1, v1 = np.ceil(uv.max(0)).astype(int) + 1
    return max(u0, 0), max(v0, 0), min(u1, cam.width), min(v1, cam.height)

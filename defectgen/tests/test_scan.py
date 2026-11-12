"""Sensor models (scan.py). Small synthetic scenes, so each effect is checked
in isolation: noise on a flat plate, a shadow behind a step, dropout on bare
copper, and the frozen parameter hash."""

from __future__ import annotations

import numpy as np
import pytest
import trimesh

pytest.importorskip("open3d")

from defectgen.scan import Scene, line_scan, load_params, looking_down, params_hash, snapshot  # noqa: E402

# The sensor and material parameters as frozen for task 1.3. Changing any
# number in configs/sensors.yaml or configs/materials.yaml changes this --
# which is the point: do it before looking at detection results, and update
# this line in the same commit so the change is on the record.
FROZEN = "779e2255dc4c"

SENSORS, _ = load_params()


def plate(z=0.0, size=60.0, thick=2.0):
    return trimesh.creation.box(extents=[size, size, thick]).apply_translation([0, 0, z - thick / 2])


def test_sensor_parameters_are_frozen():
    assert params_hash()[:12] == FROZEN, (
        "sensors.yaml or materials.yaml changed. If that was deliberate and no "
        "detection results have been looked at since, update FROZEN."
    )


def test_profilometer_on_a_flat_diffuse_plate_has_datasheet_noise():
    """Cast aluminium (diffuse) at the reference distance: every ray returns,
    and the height scatter is the 1 um repeatability (signal is high, so the
    noise is not inflated)."""
    cfg = SENSORS["profilometer"]
    scene = Scene.from_groups({"end_plates": plate()})
    r = line_scan(scene, "profilometer", looking_down(0, 0, cfg["reference_distance"]), 2.0)
    assert r.dropped["shadow"] == 0 and r.dropped["dark"] == 0 and r.dropped["glare"] == 0
    err = r.points[:, 2] - r.truth[:, 2]
    assert err.std() == pytest.approx(cfg["sigma_z"], rel=0.3)
    assert np.abs(r.truth[:, 2]).max() < 1e-4


def test_a_step_casts_a_triangulation_shadow():
    """A 10 mm block on the plate: the camera, looking along the scan
    direction, cannot see the floor just behind it, so those points drop."""
    cfg = SENSORS["profilometer"]
    block = trimesh.creation.box(extents=[10, 60, 10]).apply_translation([0, 0, 5])
    scene = Scene.from_groups({"end_plates": plate(), "terminals_pos": block})
    r = line_scan(scene, "profilometer", looking_down(-20, 0, cfg["reference_distance"]), 40.0)
    assert r.dropped["shadow"] > 0
    # No floor points survive in the strip the block hides from the camera.
    floor = r.points[np.abs(r.points[:, 2]) < 0.1]
    behind = (floor[:, 0] > 5) & (floor[:, 0] < 5 + 10 * np.tan(np.radians(cfg["triangulation_angle"])) - 0.5)
    assert not behind.any()


def test_bare_copper_returns_far_less_light_than_diffuse_metal():
    """Same geometry, two materials: rolled copper's light goes mostly into a
    narrow lobe, so the returned signal is a fraction of cast aluminium's and
    noise grows -- the specular problem in one number."""
    cfg = SENSORS["line_scan"]
    pose = looking_down(0, 0, cfg["reference_distance"])
    tilt = trimesh.transformations.rotation_matrix(np.radians(20), [1, 0, 0])
    cu = line_scan(Scene.from_groups({"busbars": plate().apply_transform(tilt)}), "line_scan", pose, 2.0)
    al = line_scan(Scene.from_groups({"end_plates": plate().apply_transform(tilt)}), "line_scan", pose, 2.0)
    assert np.median(cu.signal) < 0.2 * np.median(al.signal)
    assert cu.dropped["dark"] > al.dropped["dark"]


def test_snapshot_glare_on_copper_facing_the_camera():
    """Flat copper square under the Zivid: around the mirror direction the
    lobe saturates the camera and points drop as glare."""
    cfg = SENSORS["snapshot"]
    scene = Scene.from_groups({"busbars": plate(size=200.0)})
    r = snapshot(scene, looking_down(0, 0, cfg["focus_distance"]))
    assert r.dropped["glare"] > 0


def test_snapshot_noise_grows_with_distance():
    near_d, far_d = 400.0, 900.0
    scene = Scene.from_groups({"end_plates": plate(size=400.0)})
    near = snapshot(scene, looking_down(0, 0, near_d))
    far = snapshot(scene, looking_down(0, 0, far_d))
    std = lambda r: (r.points - r.truth)[:, 2].std()  # noqa: E731
    assert std(far) / std(near) == pytest.approx((far_d / near_d) ** 2, rel=0.25)


def test_scans_are_reproducible_and_carry_the_hash():
    cfg = SENSORS["profilometer"]
    scene = Scene.from_groups({"end_plates": plate()})
    pose = looking_down(0, 0, cfg["reference_distance"])
    a = line_scan(scene, "profilometer", pose, 1.0, seed=3)
    b = line_scan(scene, "profilometer", pose, 1.0, seed=3, chunk=7)
    np.testing.assert_array_equal(a.points, b.points)
    assert a.params_hash == params_hash()


def test_threaded_sweep_is_identical_to_serial(monkeypatch):
    """Fixlist F5: a sweep's chunks run on a thread pool. Every array, the drop
    counts and their per-material split (key order too: it is saved) must match
    one thread exactly, on a scene that drops rays for several reasons."""
    cfg = SENSORS["profilometer"]
    step = trimesh.creation.box(extents=[10, 60, 3]).apply_translation([0, 0, 1.5])
    scene = Scene.from_groups({"end_plates": plate(), "busbars": step})
    pose = looking_down(-8, 0, cfg["reference_distance"])
    runs = []
    for threads in ("1", "8"):
        monkeypatch.setenv("DEFECTGEN_THREADS", threads)
        runs.append(line_scan(scene, "profilometer", pose, 16.0, seed=5, chunk=13))
    a, b = runs
    for f in ("points", "truth", "signal", "material", "index", "sigma"):
        np.testing.assert_array_equal(getattr(a, f), getattr(b, f), err_msg=f)
    assert list(a.dropped.items()) == list(b.dropped.items())
    assert list(a.dropped_by_material.items()) == list(b.dropped_by_material.items())
    assert sum(v for k, v in a.dropped.items() if k != "miss") > 0   # the scene does drop rays


def test_photos_take_four_close_shots_and_stitch_them(tmp_path):
    """A clean module through the photo capture: one shot per quadrant, as
    close as the Zivid allows, cropped to it, and a mosaic of the whole module
    at the shots' resolution (portrait: rows along the module's length)."""
    from PIL import Image

    from defectgen.config import load
    from defectgen.module import export
    from defectgen.photos import capture, shot_layout

    cfg = load()
    quads, distance, fit, mm_px = shot_layout(cfg)
    # Centred between pins 3-4, 9-10, 15-16, 21-22 (DIP numbering, module.terminal_pin).
    assert [q["between"] for q in quads] == [(3, 4), (9, 10), (15, 16), (21, 22)]
    assert [q["pins"] for q in quads] == [(1, 6), (7, 12), (13, 18), (19, 24)]
    assert fit < SENSORS["snapshot"]["working_distance"][0] == distance  # clamped to 300 mm

    run = tmp_path / "run"
    export(run, cfg=cfg)
    folder = tmp_path / "log"
    folder.mkdir()
    info = capture(run, "post_weld", folder, stitch=True, aa=1)
    shots = sorted((folder / "photos").glob("q*.png"))
    assert len(shots) == 4
    mosaic = Image.open(folder / "photos" / "stitched.png")
    assert mosaic.height == pytest.approx((cfg.module_l + 12) / mm_px, abs=2)
    assert info["defects_boxed"] == 0 and not list((folder / "defects").glob("*.png"))


def test_line_sensor_noise_grows_with_distance():
    """Triangulation: height noise scales with range squared, so the same
    plate 30 mm beyond the reference distance is noisier than 30 mm inside it."""
    cfg = SENSORS["profilometer"]
    scene = Scene.from_groups({"end_plates": plate()})
    d0 = cfg["reference_distance"]
    near = line_scan(scene, "profilometer", looking_down(0, 0, d0 - 30), 2.0)
    far = line_scan(scene, "profilometer", looking_down(0, 0, d0 + 30), 2.0)
    std = lambda r: (r.points - r.truth)[:, 2].std()  # noqa: E731
    assert std(far) / std(near) == pytest.approx(((d0 + 30) / (d0 - 30)) ** 2, rel=0.2)


def test_faster_profiles_get_less_light():
    """Ten times the reference rate is a tenth of the exposure: on tilted
    copper, far more points come back too dark to use."""
    cfg = SENSORS["line_scan"]
    tilt = trimesh.transformations.rotation_matrix(np.radians(20), [1, 0, 0])
    scene = Scene.from_groups({"busbars": plate().apply_transform(tilt)})
    pose = looking_down(0, 0, cfg["reference_distance"])
    slow = line_scan(scene, "line_scan", pose, 2.0, speed=100.0, rate=cfg["reference_rate"])
    fast = line_scan(scene, "line_scan", pose, 2.0, speed=1000.0, rate=cfg["reference_rate"] * 7)
    assert fast.dropped["dark"] > 2 * max(slow.dropped["dark"], 1)
    with pytest.raises(ValueError, match="maximum"):
        line_scan(scene, "line_scan", pose, 2.0, speed=100.0, rate=cfg["max_rate"] * 2)

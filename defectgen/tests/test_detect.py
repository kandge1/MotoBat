"""Detection (task 1.4): the fits on known answers, then the whole pipeline on
generated modules with one defect each, scanned for real.

The pipeline tests use one short profilometer sweep along the +Y terminal row,
over cells 3-8 -- enough to hold every feature tested, and quick. Each asserts
both halves: the defect is found, and nothing else is."""

from __future__ import annotations

import json

import numpy as np
import pytest

from defectgen.fit import fit_plane, min_area_rect, rigid_2d

# -- fits, against known answers -----------------------------------------------


def test_plane_fit_recovers_a_tilted_plane_through_outliers():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-9, 9, (4000, 2))
    z = 101.0 + 0.01 * xy[:, 0] - 0.004 * xy[:, 1] + rng.normal(0, 0.001, 4000)
    z[:200] += rng.uniform(0.2, 0.5, 200)  # FOD, spatter: proud of the face
    plane, keep = fit_plane(np.column_stack([xy, z]))
    assert plane.z(0, 0) == pytest.approx(101.0, abs=2e-4)
    assert plane.tilt_deg == pytest.approx(np.degrees(np.arctan(np.hypot(0.01, 0.004))), abs=0.01)
    assert not keep[:200].any()


@pytest.mark.parametrize("yaw", [-30.0, 0.0, 0.4, 12.0])
def test_min_area_rect_recovers_centre_yaw_and_size(yaw):
    rng = np.random.default_rng(1)
    uv = rng.uniform([-9.25, -11.0], [9.25, 11.0], (20000, 2))
    th = np.radians(yaw)
    xy = uv @ np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]]) + [14.0, 57.5]
    r = min_area_rect(xy)
    assert (r.cx, r.cy) == pytest.approx((14.0, 57.5), abs=0.02)
    assert r.yaw_deg == pytest.approx(yaw, abs=0.05)
    assert sorted((r.ex, r.ey)) == pytest.approx([18.5, 22.0], abs=0.05)


def test_rigid_2d_from_two_points():
    th = np.radians(0.15)
    r_true = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    src = np.array([[-42.0, -57.5], [-42.0, 57.5]])
    dst = src @ r_true.T + [0.3, 0.2]
    r, t = rigid_2d(src, dst)
    assert np.allclose(r, r_true) and np.allclose(t, [0.3, 0.2])


def test_evaluation_says_what_each_finding_and_label_was():
    """Per finding (found / knock-on / false alarm) and per label (found /
    missed, and what was found there instead): what show_detections.py draws."""
    from defectgen.evaluate import evaluate, to_json

    def box(c):
        return ((c - 1.0, -1.0, 0.0), (c + 1.0, 1.0, 1.0))

    labels = {"defects": [{"cls": "fod", "location": [0, 0, 0.5], "bounds": box(0), "magnitude": 0.5},
                          {"cls": "fod", "location": [10, 0, 0.5], "bounds": box(10), "magnitude": 0.5},
                          {"cls": "missing_pad", "location": [20, 0, 0.5], "bounds": box(20), "magnitude": 1}]}
    findings = [{"cls": "fod", "bounds": box(0), "location": (0, 0, 0.5)},
                {"cls": "handling_damage", "bounds": box(20), "location": (20, 0, 0.5)},
                {"cls": "fod", "bounds": box(40), "location": (40, 0, 0.5)}]
    r = evaluate({"stage": "pre_busbar", "findings": findings}, labels)
    assert r["finding_verdicts"] == ["tp", "other", "fp"]
    assert [d["verdict"] for d in r["labels"]] == ["tp", "fn", "fn"]
    assert r["labels"][2]["found_there"] == ["handling_damage"]
    assert json.loads(to_json(r))["total"]["fp"] == 1


# -- the pipeline ---------------------------------------------------------------

o3d = pytest.importorskip("open3d")

from defectgen import defects as D  # noqa: E402
from defectgen.config import load  # noqa: E402
from defectgen.detect import Capture, Reference, detect  # noqa: E402
from defectgen.evaluate import evaluate  # noqa: E402
from defectgen.module import collect_labels, injected_plan, realise_groups  # noqa: E402
from defectgen.parts import load_all  # noqa: E402
from defectgen.scan import Scene, line_scan, looking_down, without_groups  # noqa: E402

CFG = load()


@pytest.fixture(scope="module")
def parts():
    return load_all(CFG)


@pytest.fixture(scope="module")
def refs(parts):
    cache = {}

    def get(stage):
        if stage not in cache:
            cache[stage] = Reference(stage, CFG, parts=parts)
        return cache[stage]

    return get


def sweep(groups, x0=-85.0, length=170.0):
    """One profilometer sweep along the +Y terminal row, at its reference
    distance: cells 3-8, which carry busbar 1 whole."""
    start = looking_down(x0, CFG.terminal_y, CFG.z_terminal_top + 245.0)
    return Capture.from_scan(line_scan(Scene.from_groups(groups), "profilometer", start, length), "prof")


def run(stage, injectors, parts, refs):
    plan = injected_plan(stage, CFG, 0, injectors, parts)
    labels = json.loads(collect_labels(plan, 0, parts).to_json())
    groups = {g: m for g, m in realise_groups(plan, parts).items() if g not in without_groups(stage)}
    report = detect([sweep(groups)], stage, refs(stage))
    return report, evaluate(report, labels)


def classes(report):
    return sorted(f.cls for f in report.findings)


def test_scan_points_carry_the_ray_they_came_from(parts):
    """The organised index: a line sensor's point lies in its own profile's
    laser plane, so detection can compare scan and golden ray by ray."""
    plan = injected_plan("pre_busbar", CFG, 0, (), parts)
    cap = sweep(realise_groups(plan, parts), length=10.0)
    pose = cap.poses[cap.index // cap.grid[1]]
    off = np.einsum("ni,ni->n", cap.points - pose[:, :3, 3], pose[:, :3, 1])
    assert np.abs(off).max() < 1e-9
    assert len(np.unique(cap.index)) == len(cap.index)


def test_stale_scans_are_refused_with_the_reason(tmp_path):
    """A scan records what it scanned; detection skips one that no longer
    matches the module rather than scoring it against the new labels."""
    from defectgen.detect import scan_problem
    from defectgen.scan import ScanResult, labels_fingerprint

    (tmp_path / "pre_busbar.labels.json").write_text('{"defects": []}')

    def save(name, index=True, **source):
        r = ScanResult("line_scan", np.zeros((3, 3)), np.zeros((3, 3)), np.ones(3), np.zeros(3, int), ["x"],
                       np.eye(4)[None], index=np.arange(3) if index else None,
                       grid=(1, 3) if index else None, source=source)
        return r.save(tmp_path / f"{name}.npz")

    ok = save("ok", stage="pre_busbar", include=["cmc"], labels=labels_fingerprint(tmp_path, "pre_busbar"))
    assert scan_problem(ok, tmp_path, "pre_busbar") == ("", {"cmc"})
    assert "ray index" in scan_problem(save("old", index=False), tmp_path, "pre_busbar")[0]
    assert "which module" in scan_problem(save("nosource"), tmp_path, "pre_busbar")[0]
    assert "stage pre_busbar, not post_weld" in scan_problem(ok, tmp_path, "post_weld")[0]
    (tmp_path / "pre_busbar.labels.json").write_text('{"defects": [{"cls": "fod"}]}')
    assert "regenerated" in scan_problem(ok, tmp_path, "pre_busbar")[0]


@pytest.mark.parametrize("stage", ["pre_busbar", "pre_weld", "post_weld"])
def test_a_clean_module_raises_nothing(stage, parts, refs):
    report, _ = run(stage, [], parts, refs)
    assert report.findings == []
    # One sweep sees one terminal row: nothing found is not a pass.
    assert report.uncovered and report.result == "INCOMPLETE"


def test_terminal_height_is_found_and_measured(parts, refs):
    # Cell 6 is in a flipped group, so its positive pad is on the +Y row.
    report, ev = run("pre_busbar", [D.terminal_height(6, "pos", dz=0.15)], parts, refs)
    assert classes(report) == ["terminal_height"]
    assert report.findings[0].magnitude == pytest.approx(0.15, abs=0.01)
    assert ev["total"]["tp"] == 1


def test_terminal_tilt_is_not_also_reported_as_height(parts, refs):
    report, ev = run("pre_busbar", [D.terminal_tilt(4, "neg", deg=0.5)], parts, refs)
    assert classes(report) == ["terminal_tilt"]
    assert report.findings[0].magnitude == pytest.approx(0.5, abs=0.05)


def test_cell_placement_from_one_pad(parts, refs):
    """The sweep sees one pad per cell, so pose comes from that pad's
    outline alone: its centre and its yaw."""
    report, ev = run("pre_busbar", [D.cell_placement(5, dx=0.3, dy=0.2, dtheta_deg=0.15)], parts, refs)
    assert classes(report) == ["cell_placement"]
    cell = report.measurements["cells"][5]
    assert cell["dyaw_deg"] == pytest.approx(0.15, abs=0.03)
    assert np.hypot(cell["dx"], cell["dy"]) == pytest.approx(np.hypot(0.3, 0.2), abs=0.05)


def test_reversed_polarity_is_told_by_the_fill_plug(parts, refs):
    report, ev = run("pre_busbar", [D.reversed_polarity(4)], parts, refs)
    assert classes(report) == ["reversed_polarity"]
    assert ev["total"] == pytest.approx({**ev["total"], "tp": 1, "fn": 0, "fp": 0})


def test_busbar_gap_is_measured_inside_one_capture(parts, refs):
    """The pad shows either side of the busbar, so its plane and the busbar's
    come from the same sweep: the gap needs no registration between views."""
    report, ev = run("pre_weld", [D.busbar_liftoff(1, gap=0.3)], parts, refs)
    assert classes(report) == ["busbar_liftoff"]
    assert report.findings[0].magnitude == pytest.approx(0.3, abs=0.01)


def test_seam_deviation_and_missing_weld(parts, refs):
    report, ev = run("post_weld", [D.missing_weld(1, 5), D.seam_deviation(1, 7, dy=0.4)], parts, refs)
    assert classes(report) == ["missing_weld"] * 2 + ["seam_deviation"] * 2
    assert ev["total"]["tp"] == 4 and ev["total"]["fp"] == 0
    for f in report.findings:
        if f.cls == "seam_deviation":
            assert f.magnitude == pytest.approx(0.4, abs=0.05)


def test_alignment_recovers_a_fixture_error_from_the_housing(parts, refs):
    """The whole module sits off its fixture. The line-scan sees the housing,
    so X, Y and yaw come back from the end-plate outlines and Z from ICP on
    the housing tops -- and the module still reads clean. X and Y are good to
    half the 0.1 mm profile pitch, the limit of locating an edge across the
    scan direction."""
    from defectgen.plan import rotation_z, translation

    m = translation(0.15, -0.10, 0.03) @ rotation_z(0.05)
    plan = injected_plan("pre_busbar", CFG, 0, (), parts)
    groups = {g: x.copy().apply_transform(m) for g, x in realise_groups(plan, parts).items()}
    start = looking_down(-CFG.module_l / 2 - 5, 0.0, CFG.z_terminal_top + 500.0)
    cap = Capture.from_scan(line_scan(Scene.from_groups(groups), "line_scan", start, CFG.module_l + 10), "line")
    report = detect([cap], "pre_busbar", refs("pre_busbar"))
    got = np.array(report.captures[0]["icp"]["translation_mm"])
    want = np.linalg.inv(m)[:3, 3]
    assert got[2] == pytest.approx(want[2], abs=0.002)
    assert np.abs(got[:2] - want[:2]).max() < 0.06
    assert report.findings == []


def test_step_band_marks_steps_and_holes_not_slopes():
    """The discontinuity band: a depth step and a hole's rim are edges, a
    gently tilted surface is not, and the band widens with misregistration."""
    from defectgen.detect import Golden, load_detect_params, step_band

    rows, cols, sp = 20, 20, 0.1
    r, c = np.mgrid[0:rows, 0:cols]
    z = 0.3 * sp * c.astype(float)          # tilted ~17 deg: never an edge
    z[:, 10:] += 1.0                        # a 1 mm step between columns 9 and 10
    keep = np.ones((rows, cols), bool)
    keep[15:18, 3:6] = False                # a hole (shadow)
    idx = np.flatnonzero(keep.ravel())
    pts = np.column_stack([c.ravel() * sp, r.ravel() * sp, z.ravel()])[idx]
    axis = np.tile([0.0, 0.0, 1.0], (len(idx), 1))
    gold = Golden(pts, idx, np.zeros(len(idx), np.int16), ["x"], axis, np.full(len(idx), sp * sp), sp)
    q = load_detect_params()["residual"]
    band = step_band(gold, (rows, cols), misreg=0.0, q=q).reshape(rows, cols)
    assert band[5, 9] and band[5, 10]               # the step
    assert band[16, 2] and band[14, 4]              # the hole's rim
    assert not band[5, 3] and not band[5, 16]       # the tilted plane, away from both
    wide = step_band(gold, (rows, cols), misreg=0.3, q=q).reshape(rows, cols)
    assert wide.sum() > band.sum() and wide[5, 6] and not band[5, 6]


def _surface(z_of_x, rows=12, cols=24, sp=0.1):
    """A golden capture of the surface z = f(x), looked at straight down."""
    from defectgen.detect import Golden

    r, c = np.mgrid[0:rows, 0:cols]
    x, y = c.ravel() * sp, r.ravel() * sp
    idx = np.arange(rows * cols)
    pts = np.column_stack([x, y, z_of_x(x)])
    axis = np.tile([0.0, 0.0, -1.0], (len(idx), 1))
    return Golden(pts, idx, np.zeros(len(idx), np.int16), ["x"], axis, np.full(len(idx), sp * sp), sp), (rows, cols)


def test_sideways_gain_is_the_slope_and_the_steeper_face_at_a_crease():
    """Looking straight down, a sideways shift d of the surface z = s x reads
    s d: the gain is the slope. Zero on a level top; at a wall's foot, the wall's."""
    from defectgen.detect import sideways_gain

    level, grid = _surface(lambda x: np.zeros_like(x))
    assert np.allclose(sideways_gain(level, grid, np.arange(len(level.index))), 0.0)
    steep, grid = _surface(lambda x: 2.0 * x)
    assert np.allclose(sideways_gain(steep, grid, np.arange(len(steep.index))), 2.0)
    crease, grid = _surface(lambda x: 2.0 * np.maximum(x - 1.0, 0.0))     # floor, then a 63 deg wall at x = 1
    foot = int(np.flatnonzero(np.isclose(crease.points[:, 0], 1.0) & np.isclose(crease.points[:, 1], 0.5))[0])
    assert sideways_gain(crease, grid, np.array([foot]))[0] == pytest.approx(2.0)


def test_a_steep_face_moved_within_registration_is_not_a_region():
    """§4B.6, 12 Nov 2026: a close-up snapshot read a cap recess's walls ~0.08
    mm off from a 0.027 mm sideways shift (slope ~3). A slope-2 face shifted by
    0.04 mm (reads 0.08) within a 0.05 mm registration is not a defect; a 0.08
    mm bump on a level top is."""
    from defectgen.detect import Capture, load_detect_params, residuals

    p = load_detect_params()
    for f, shift, want in ((lambda x: 2.0 * x, 0.04, 0), (lambda x: np.zeros_like(x), 0.0, 1)):
        gold, grid = _surface(f)
        pts = gold.points.copy()
        pts[:, 2] = f(pts[:, 0] - shift)
        if want:
            pts[np.hypot(pts[:, 0] - 1.2, pts[:, 1] - 0.6) < 0.35, 2] += 0.08
        cap = Capture("toy", pts, gold.index, grid, np.eye(4)[None], {})
        regions, _ = residuals(cap, gold, p, misreg=0.05)
        assert len([g for g in regions if g.kind == "raised"]) == want


# -- registration from the housing strips (arm hardening, plan §4B.6) ----------

def _strip_obs(t, strip, at, axis, edge=100.0, whole=True, spacing=0.05):
    """What a strip outline would report for a module moved by `t` (scan =
    t @ design): the across-strip offset golden - scan at design point `at`."""
    g = np.array([*at, 1.0])
    s = t @ g
    yaw = np.degrees(np.arctan2(t[1, 0], t[0, 0]))
    return {"strip": strip, "axis": axis, "at": list(at), "offset": float(g[axis] - s[axis]), "whole": whole,
            "dyaw": float(-yaw), "edge": edge, "spacing": spacing}


def _planar_t(dx, dy, yaw_deg):
    th = np.radians(yaw_deg)
    t = np.eye(3)
    t[:2, :2] = [[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]
    t[:2, 2] = dx, dy
    return t


P_ICP = {"icp": {"datum_min_edge": 20.0}}


def test_whole_strips_recover_the_placement():
    from defectgen.detect import solve_datum

    t = _planar_t(0.15, -0.10, 0.05)
    obs = [_strip_obs(t, "end -1", (-172.375, 0.0), 0, 147), _strip_obs(t, "end +1", (172.375, 0.0), 0, 147),
           _strip_obs(t, "side -1", (0.0, -74.75), 1, 333), _strip_obs(t, "side +1", (0.0, 74.75), 1, 333)]
    d, axes = solve_datum(obs, P_ICP)
    assert set(axes) == {"x", "y", "yaw"}
    corners = np.array([[x, y, 1.0] for x in (-177, 177) for y in (-75, 75)])
    back = corners @ (np.delete(np.delete(d, 2, 0), 2, 1) @ t).T
    assert np.abs(back - corners).max() < 2e-3


def test_a_strip_cut_by_the_view_fixes_nothing_and_unfixed_axes_stay():
    """A sweep along X sees a 40 mm piece of each end plate: its X still
    counts (both edges across the plate are real), Y does not exist there,
    and with no side strip whole Y stays at the fixture pose."""
    from defectgen.detect import solve_datum

    t = _planar_t(0.15, -0.10, 0.0)
    cut = _strip_obs(t, "side +1", (0.0, 74.75), 1, 333, whole=False)
    d, axes = solve_datum([_strip_obs(t, "end -1", (-172.375, 56.0), 0, 34), cut], P_ICP)
    assert "y" not in axes and d[1, 3] == 0.0
    assert d[0, 3] == pytest.approx(-0.15, abs=1e-6)


def _strip_points(x_span, y_span, step=0.05):
    x, y = np.meshgrid(np.arange(*x_span, step), np.arange(*y_span, step), indexing="ij")
    return np.c_[x.ravel(), y.ravel()]


@pytest.mark.parametrize("kind", ["side", "end"])
def test_a_locally_bowed_strip_does_not_move_the_datum(kind):
    """A cell set off its pitch bows the side plate out over a stretch. The
    strip's centreline is fitted per bin and the bowed bins dropped, so the
    datum keeps the placement and does not take up the cell's own shift."""
    from defectgen.detect import _strip_offset

    th, dx, dy = np.radians(0.01), 0.03, -0.05
    if kind == "side":
        gold, across, at = _strip_points((-160, 160), (74.0, 75.5)), 1, 0.0
    else:
        gold, across, at = _strip_points((171.6, 173.1), (-70, 70)), 0, 0.0
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    scan = gold @ rot.T + [dx, dy]
    along = 1 - across
    bow = np.abs(scan[:, along] - 10.0) < 20.0
    scan[bow, across] += 0.2 * np.cos((scan[bow, along] - 10.0) / 20.0 * np.pi / 2)
    offset, slope = _strip_offset(gold, scan, across, at, 2.0, 0.05)
    # The bow's shallow flanks, within a sample spacing of the line, stay in:
    # a few um on a 200 um bow (one outline round the whole strip: 100 um).
    assert offset == pytest.approx(-(dx, dy)[across], abs=5e-3)
    # Yaw to within 20 urad: under 4 um at the module's far corner (177 mm).
    assert slope == pytest.approx(th if across == 0 else -th, abs=2e-5)


def test_the_load_is_registered_once_from_each_strips_best_view():
    from defectgen.detect import joint_datum

    t = _planar_t(0.15, -0.10, 0.05)
    a = [_strip_obs(t, "end -1", (-172.375, 0.0), 0, 147), _strip_obs(t, "side -1", (0.0, -74.75), 1, 333)]
    b = [_strip_obs(t, "end +1", (172.375, 0.0), 0, 147), _strip_obs(t, "side +1", (0.0, 74.75), 1, 333)]
    bad = dict(_strip_obs(t, "end -1", (-172.375, -21.0), 0, 30), offset=9.9)   # a corner seen at 60 deg
    d, used = joint_datum({"a": a, "b": b, "oblique": [bad]}, P_ICP)
    assert used == ["a", "b"]
    assert d[0, 3] == pytest.approx(-0.15, abs=2e-3)
    # One end strip alone fixes X and yaw but not Y: no joint registration.
    assert joint_datum({"a": a[:1]}, P_ICP) is None


def test_a_bead_stopped_short_within_tolerance_explains_its_bare_ends():
    from types import SimpleNamespace

    from defectgen.detect import _in_rect, short_bead_stretches

    q = {"bin": 0.25, "short_tol": 0.10}
    mids = np.arange(-6.875, 7.0, 0.25)              # bins over a 10 mm stitch plus 2 mm each side

    def prof(half):
        return {"mids": mids, "has": np.abs(mids) <= half, "u": [1.0, 0.0], "centre_xy": [10.0, 5.0]}

    gold = prof(5.0)
    seams = {0: SimpleNamespace(scan=prof(4.6), gold=gold)}           # 8 % short: in spec
    rects = short_bead_stretches(seams, q, 1.25)
    assert len(rects) == 2
    # The bare stretch, and the stop crater just inside the shortened bead ...
    inside = np.array([[10 + 4.8, 5.0], [10 - 4.8, 5.3], [10 + 4.0, 5.0]])
    # ... but not the bead's middle, nor beside the stitch.
    outside = np.array([[10 + 3.0, 5.0], [10 + 4.8, 7.0]])
    assert all(any(_in_rect(xy[None], r)[0] for r in rects) for xy in inside)
    assert not any(any(_in_rect(xy[None], r)[0] for r in rects) for xy in outside)
    # Past short_tol a short_weld finding must account for it, not this rule.
    assert short_bead_stretches({0: SimpleNamespace(scan=prof(4.0), gold=gold)}, q, 1.25) == []
    # A full-length bead leaves nothing bare.
    assert short_bead_stretches({0: SimpleNamespace(scan=prof(5.0), gold=gold)}, q, 1.25) == []

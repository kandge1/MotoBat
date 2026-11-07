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

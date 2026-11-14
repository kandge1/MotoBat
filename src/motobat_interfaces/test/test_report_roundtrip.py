"""Every field of a real defectgen DefectReport fits the message, and survives a
serialise / deserialise round trip.

The samples in test/reports are arm reports from the 1.7 v3 dataset (14 Nov 2026): a clean
pre-weld module, a shifted cell, spatter and a lifted busbar. (Not test/data: the repo
.gitignore drops every data/ folder.) `to_msg` is the conversion motobat_perception will use;
it lives here until that package exists.

Run: colcon test --packages-select motobat_interfaces  (system python3, ROS sourced)
"""
import json
import math
from pathlib import Path

import pytest
from rclpy.serialization import deserialize_message, serialize_message

from motobat_interfaces.msg import (CaptureInfo, DefectReport, Finding, KeyValue,
                                    Measurement, Uncovered)

DATA = Path(__file__).parent / "reports"
MM = 1e-3

# The unit each measured key is judged in. A key missing here fails the test, so a new
# detector output cannot reach the line without a stated unit.
UNIT = {
    # lengths, mm
    **dict.fromkeys(["dx", "dy", "dz", "z", "dh", "gap", "max_gap", "gap_here", "length",
                     "gold_length", "short", "offset", "sagitta", "drift", "width_change",
                     "crown_change", "undercut", "dz_vs_row", "peak_mm", "extent", "xy",
                     "gold_xy", "as_designed", "turned"], "mm"),
    # angles, deg
    **dict.fromkeys(["dyaw_deg", "tilt_deg"], "deg"),
    # areas, mm2
    **dict.fromkeys(["area_mm2", "width_change_area"], "mm2"),
    # counts and fractions
    **dict.fromkeys(["at_pin", "pin", "pins", "points", "terminals", "terminals_seen", "deep",
                     "coverage", "missing_fraction"], "1"),
}
NOT_NUMBERS = {"capture", "captures", "seen", "pose_measured", "capable"}

# Finding.magnitude unit per class (detect.py: what each Finding(...) passes as magnitude).
MAG_UNIT = {"terminal_tilt": "deg", "reversed_polarity": "deg", "missing_cell": "1",
            "missing_weld": "1"}


def _kv(d: dict) -> list:
    out = []
    for k, v in d.items():
        if k in NOT_NUMBERS or isinstance(v, str):
            continue
        assert k in UNIT, f"no unit stated for measured key {k!r}"
        vals = v if isinstance(v, list) else [v]
        for i, x in enumerate(_flat(vals)):
            out.append(KeyValue(key=k if not isinstance(v, list) else f"{k}[{i}]",
                                value=float(x), unit=UNIT[k]))
    return out


def _flat(xs):
    for x in xs:
        yield from (_flat(x) if isinstance(x, list) else [x])


def _pt(xyz):
    from geometry_msgs.msg import Point
    return Point(x=xyz[0] * MM, y=xyz[1] * MM, z=xyz[2] * MM)


def _mag_unit(f: dict) -> str:
    if f["cls"] in MAG_UNIT:
        return MAG_UNIT[f["cls"]]
    if "area_mm2" in f["measured"] and not math.isfinite(f["measured"].get("peak_mm", math.nan)):
        return "mm2"      # a residual region with only missing data: its area, not its peak
    return "mm"


def to_msg(r: dict, module_id: str = "") -> DefectReport:
    m = DefectReport(module_id=module_id, stage=r["stage"], detect_hash=r["detect_hash"])
    m.header.frame_id = "module"
    m.result = {"OK": DefectReport.OK, "NOK": DefectReport.NOK,
                "INCOMPLETE": DefectReport.INCOMPLETE}[r["result"]]
    for f in r["findings"]:
        m.findings.append(Finding(cls=f["cls"], magnitude=float(f["magnitude"]), unit=_mag_unit(f),
                                  location=_pt(f["location"]), bounds_min=_pt(f["bounds"][0]),
                                  bounds_max=_pt(f["bounds"][1]), capture=f["capture"],
                                  feature=f["feature"], measured=_kv(f["measured"])))
    for kind, entries in r["measurements"].items():
        if kind == "carrier":     # one flat dict
            entries = {"": entries}
        elif isinstance(entries, list):
            entries = dict(enumerate(entries))
        for i, e in entries.items():
            m.measurements.append(Measurement(kind=kind, id=str(i), capture=e.get("capture", ""),
                                              seen=bool(e.get("seen", True)), values=_kv(e)))
    for kind, ids in r["uncovered"].items():
        m.uncovered.append(Uncovered(kind=kind, ids=[str(i) for i in ids]))
    for c in r["captures"]:
        icp = c.get("icp", {})
        m.captures.append(CaptureInfo(
            name=c["name"], sensor=c["sensor"], params_hash=c["params_hash"], points=c["points"],
            icp_fitness=float(icp.get("fitness", math.nan)),
            icp_rmse=float(icp.get("rmse_mm", math.nan)) * MM,
            registered_axes=list(icp.get("registered_axes", [])),
            timings=[KeyValue(key=k.split(":", 1)[1], value=float(v), unit="s")
                     for k, v in r["timings_s"].items() if k.split(":", 1)[0] == c["name"]]))
    return m


SAMPLES = sorted(DATA.glob("*.report.json"))


def test_the_samples_are_there():
    assert len(SAMPLES) == 4


@pytest.mark.parametrize("path", SAMPLES, ids=[p.name.split(".report")[0] for p in SAMPLES])
def test_a_real_report_converts_and_round_trips(path):
    r = json.loads(path.read_text())
    m = to_msg(r, module_id=path.stem)
    back = deserialize_message(serialize_message(m), DefectReport)
    assert back == m
    assert len(back.findings) == len(r["findings"])
    assert {u.kind for u in back.uncovered} == set(r["uncovered"])
    assert back.result == {"OK": 0, "NOK": 1, "INCOMPLETE": 2}[r["result"]]
    # every timing lands on its capture
    assert sum(len(c.timings) for c in back.captures) == len(r["timings_s"])


def test_samples_cover_ok_nok_and_every_stage():
    rs = [json.loads(p.read_text()) for p in SAMPLES]
    assert {r["result"] for r in rs} >= {"OK", "NOK"}
    assert {r["stage"] for r in rs} == {"pre_busbar", "pre_weld", "post_weld"}


def test_an_unseen_feature_makes_the_report_incomplete():
    r = json.loads((DATA / "clean_pre_weld_m0.report.json").read_text())
    r["uncovered"] = {"busbar position": [3, 5]}
    r["result"] = "INCOMPLETE"
    m = deserialize_message(serialize_message(to_msg(r)), DefectReport)
    assert m.result == DefectReport.INCOMPLETE
    assert m.uncovered[0].kind == "busbar position" and list(m.uncovered[0].ids) == ["3", "5"]


def test_cell_placement_carries_what_the_correction_loop_needs():
    """1.12's input: per cell, dx dy (mm) and dyaw (deg), from the cell_shift case."""
    r = json.loads((DATA / "pod_cell_shift_L2_0.22_s0.report.json").read_text())
    m = to_msg(r)
    cells = [x for x in m.measurements if x.kind == "cells"]
    assert len(cells) == 12
    vals = {kv.key: (kv.value, kv.unit) for kv in cells[0].values}
    assert vals["dx"][1] == "mm" and vals["dyaw_deg"][1] == "deg"
    hit = [f for f in m.findings if f.cls == Finding.CELL_PLACEMENT]
    assert hit and 0.15 < hit[0].magnitude < 0.3 and hit[0].unit == "mm"

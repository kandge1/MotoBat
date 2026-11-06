"""One folder per scan run under data/scan_logs/, set up the same way for
every sensor. Layout (project_structure.md §2, "data/"):

    data/scan_logs/<YYYYMMDD_HHMMSS>_<run>_<stage>_<sensor>/
        scan.log        everything the run printed, timestamped
        summary.json    settings, sensor params hash, what was kept and lost
        photos/         2D captures (photos sensor): one per shot, plus the mosaic
        annotated/      the same images with the labelled defects boxed
        defects/        one close-up per labelled defect

Point clouds are not copied here -- they are hundreds of MB. They stay in
data/<run>/scans/, and summary.json records where.
"""

from __future__ import annotations

import datetime
import json
import logging
from pathlib import Path

LOG_ROOT = Path("data/scan_logs")


def start(run: Path, stage: str, sensor: str, root: Path = LOG_ROOT, quiet: bool = False) -> Path:
    """Make the run's folder and send defectgen's log lines to the console
    and to <folder>/scan.log. Returns the folder."""
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = Path(root) / f"{stamp}_{Path(run).name}_{stage}_{sensor}"
    folder.mkdir(parents=True, exist_ok=True)

    fmt = logging.Formatter("%(asctime)s.%(msecs)03d %(message)s", "%H:%M:%S")
    logger = logging.getLogger("defectgen")
    logger.setLevel(logging.INFO)
    for h in list(logger.handlers):
        logger.removeHandler(h)
    console = logging.StreamHandler()
    console.setFormatter(fmt)
    console.setLevel(logging.WARNING if quiet else logging.INFO)
    logfile = logging.FileHandler(folder / "scan.log")
    logfile.setFormatter(fmt)
    logger.addHandler(console)
    logger.addHandler(logfile)
    return folder


def write_summary(folder: Path, info: dict) -> Path:
    path = Path(folder) / "summary.json"
    path.write_text(json.dumps(info, indent=2))
    return path

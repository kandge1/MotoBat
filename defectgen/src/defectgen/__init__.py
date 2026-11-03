"""Procedural battery-module geometry for MotoBat.

Deliberately free of any ROS dependency: `pip install -e defectgen && pytest`
must work without a ROS 2 workspace (project_structure.md §2).
"""

from . import defects
from .config import Config, load
from .groundtruth import STAGES, Defect, Labels, Stage
from .module import build_plan, export, generate, realise
from .plan import Plan

__all__ = [
    "Config",
    "Defect",
    "Labels",
    "Plan",
    "STAGES",
    "Stage",
    "build_plan",
    "defects",
    "export",
    "generate",
    "load",
    "realise",
]

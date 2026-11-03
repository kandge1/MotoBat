"""Procedural battery-module geometry for MotoBat.

Deliberately free of any ROS dependency: `pip install -e defectgen && pytest`
must work without a ROS 2 workspace (project_structure.md §2).
"""

from .config import Config, load
from .groundtruth import STAGES, Labels, Stage
from .module import export, generate

__all__ = ["Config", "Labels", "STAGES", "Stage", "export", "generate", "load"]

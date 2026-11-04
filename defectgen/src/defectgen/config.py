"""Module parameters, loaded from YAML. One source of truth with Fusion (spec §7)."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).parent / "configs"
DEFAULT_CONFIG = CONFIG_DIR / "vda355_3p4s.yaml"


@dataclass(frozen=True)
class Config:
    name: str

    module_l: float
    module_w: float
    module_h: float

    n_cells: int
    n_parallel: int
    n_series: int
    cell_pitch: float

    cell_w: float
    cell_t: float
    cell_h: float

    terminal_d: float
    terminal_w: float
    terminal_h: float
    terminal_pitch_y: float
    terminal_neg_w: float

    pad_t: float
    pad_min_t: float
    pad_w: float
    pad_h: float

    end_plate_t: float
    end_plate_w: float
    end_plate_h: float
    side_plate_l: float
    side_plate_t: float
    side_plate_h: float
    side_plate_bulge_spread: float
    cooling_plate_l: float
    cooling_plate_w: float
    cooling_plate_t: float
    cover_l: float
    cover_w: float
    cover_t: float

    carrier_l: float
    carrier_w: float
    carrier_h: float
    carrier_deck_h: float

    busbar_t: float
    busbar_w: float
    busbar_end_l: float
    busbar_series_l: float

    cmc_l: float
    cmc_w: float
    cmc_t: float

    weld_stitches: int
    weld_stitch_l: float
    weld_stitch_w: float
    weld_stitch_offset_y: float
    weld_bead_h: float
    weld_edge_margin: float
    weld_speed: float
    weld_trace_step: float
    weld_melt_threshold: float
    weld_ripple_pitch: float
    weld_ripple_amp: float
    weld_crater_depth: float
    weld_grid: float
    weld_gap_ok: float
    weld_gap_fusion: float
    weld_gap_hole: float
    weld_fill_factor: float
    weld_undercut_factor: float

    # Z levels. Each surface is named once here so no caller re-derives it.

    @property
    def z_cell_bottom(self) -> float:
        return self.cooling_plate_t

    @property
    def z_cell_top(self) -> float:
        return self.z_cell_bottom + self.cell_h

    @property
    def z_terminal_top(self) -> float:
        return self.z_cell_top + self.terminal_h

    @property
    def z_busbar_top(self) -> float:
        return self.z_terminal_top + self.busbar_t

    @property
    def z_carrier_top(self) -> float:
        return self.z_cell_top + self.carrier_h

    @property
    def z_carrier_deck(self) -> float:
        """Where the CMC sits: on the deck between the pegs, not on top of them."""
        return self.z_cell_top + self.carrier_deck_h

    @property
    def z_wall_top(self) -> float:
        return self.z_cell_bottom + self.end_plate_h

    @property
    def z_cover_top(self) -> float:
        return self.z_wall_top + self.cover_t

    def cell_centre_x(self, k: int) -> float:
        """Centre of cell k (0-indexed) along the module length."""
        first = -self.module_l / 2 + self.end_plate_t + self.cell_t / 2
        return first + k * self.cell_pitch

    @property
    def terminal_y(self) -> float:
        """Terminal centre offset from the module centreline, +/- this value."""
        return self.terminal_pitch_y / 2

    def group_of(self, k: int) -> int:
        """Parallel group a cell belongs to. Odd groups are flipped (spec §2)."""
        return k // self.n_parallel


def load(path: Path | str | None = None) -> Config:
    raw = yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())

    flat: dict[str, object] = {}
    for key, value in raw.items():
        if isinstance(value, dict):
            flat.update(value)
        else:
            flat[key] = value

    expected = {f.name for f in fields(Config)}
    missing = expected - flat.keys()
    extra = flat.keys() - expected
    if missing or extra:
        raise ValueError(
            f"config {path or DEFAULT_CONFIG} does not match Config: "
            f"missing={sorted(missing)} unexpected={sorted(extra)}"
        )

    return Config(**flat)  # type: ignore[arg-type]

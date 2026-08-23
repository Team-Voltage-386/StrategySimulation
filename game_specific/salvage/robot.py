"""
The reference SALVAGE robot -- the design every bench in this game
measures against, and the defaults a GUI form would start from.

This lives in `game_specific/` on purpose, and that is a finding from
the dry run rather than a style choice. REEFSCAPE's equivalent numbers
(`DEFAULT_PIECE_CAPACITY` / `DEFAULT_INTAKE_TIMES` /
`DEFAULT_DEPOSIT_TIMES` and the `RobotCharacteristics` built from them)
exist twice, in `apps/reefscape_widgets.py` and again in
`apps/run_strategy_sweep.py` -- so "what does the reference robot look
like" is answered by an app rather than by the game, and answered twice.
A second game makes that immediately annoying: the bench, the sweep and
any future GUI all need the same object, and none of them should own it.

Note the capacities: this robot carries **two** CRATEs at once. Every
REEFSCAPE capacity is 1, so the multi-piece paths in
`planning.py`/`tactics.py` (chaining a second deposit off a virtual
position after the first) have never actually run in a match.
"""
from __future__ import annotations

from common_sim.robot.characteristics import RobotCharacteristics
from game_specific.salvage.game_pieces import CELL_TYPE, CRATE_TYPE, SCRAP_TYPE
from game_specific.salvage.scoring import DEFAULT_SCORING_RELIABILITY_BY_ACTION

DEFAULT_PIECE_CAPACITY = {CRATE_TYPE: 2, CELL_TYPE: 1, SCRAP_TYPE: 1}

# A CELL is the awkward one to handle and a SCRAP is the easy one.
DEFAULT_INTAKE_TIMES = {CRATE_TYPE: 0.4, CELL_TYPE: 0.7, SCRAP_TYPE: 0.3}

# Coupled to the point table on purpose: the deep hold pays 8 in TELEOP
# and takes 1.6s to place, the wall hold pays 2 and takes 0.3s. Neither
# ordering is obvious without the travel time, which is the whole point.
DEFAULT_DEPOSIT_TIMES = {
    "hold_low": 0.3, "hold_high": 1.6, "reactor": 0.9, "airlock": 1.1, "beacon": 0.6,
}

# How good this robot's mechanism is for each piece type, independent of
# how hard the target is (that is the per-action table in scoring.py).
DEFAULT_SCORING_RELIABILITY_BY_TYPE = {CRATE_TYPE: 1.0, CELL_TYPE: 0.95, SCRAP_TYPE: 0.90}

ACCEPTED_PIECE_TYPES = frozenset({CRATE_TYPE, CELL_TYPE, SCRAP_TYPE})


def build_characteristics(name: str = "salvage-bot") -> RobotCharacteristics:
    return RobotCharacteristics(
        name=name,
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES),
        station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5, deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=ACCEPTED_PIECE_TYPES,
        scoring_reliability_by_type=dict(DEFAULT_SCORING_RELIABILITY_BY_TYPE),
        scoring_reliability_by_action=dict(DEFAULT_SCORING_RELIABILITY_BY_ACTION),
    )

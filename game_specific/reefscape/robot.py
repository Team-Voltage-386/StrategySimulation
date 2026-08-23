"""
The reference REEFSCAPE robot -- the design every bench in this game
measures against, and the defaults a GUI form starts from.

This is the fix for DRY_RUN_LOG.md's F5: "what does this game's
benchmark robot look like" used to be answered twice, verbatim, by
`apps/reefscape_widgets.py`'s `build_demo_characteristics` and
`apps/run_strategy_sweep.py`'s `build_characteristics` -- neither of
which is where a second game would look. `game_specific/salvage/robot.py`
put SALVAGE's answer where the game itself lives; this is REEFSCAPE's
copy moved to match.

`build_characteristics` intentionally does *not* preload a piece by
default -- that matches the headless sweep/bench callers' long-standing
behavior, and changing it would move every benchmark number in this repo
as a side effect of a refactor that isn't supposed to change any of
them. `apps/reefscape_widgets.py`'s `build_demo_characteristics` is the
one caller that wants the Game Manual's preloaded CORAL, and it asks for
it explicitly via `starting_piece_count`/`preload_piece_type` overrides.
"""
from __future__ import annotations

from common_sim.robot.characteristics import RobotCharacteristics
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE
from game_specific.reefscape.scoring import DEFAULT_SCORING_RELIABILITY_BY_ACTION

DEFAULT_PIECE_CAPACITY = {CORAL_TYPE: 1, ALGAE_TYPE: 1}
DEFAULT_INTAKE_TIMES = {CORAL_TYPE: 0.4, ALGAE_TYPE: 0.4}
DEFAULT_DEPOSIT_TIMES = {"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8, "processor": 0.4, "net": 1.2}

# 100% (deterministic scoring, the legacy behavior) for both piece types.
DEFAULT_SCORING_RELIABILITY_BY_TYPE = {CORAL_TYPE: 1.0, ALGAE_TYPE: 1.0}

ACCEPTED_PIECE_TYPES = frozenset({CORAL_TYPE, ALGAE_TYPE})


def build_characteristics(name: str = "reefscape-bot", **overrides) -> RobotCharacteristics:
    defaults = dict(
        name=name,
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES), station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5, deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=ACCEPTED_PIECE_TYPES,
        scoring_reliability_by_type=dict(DEFAULT_SCORING_RELIABILITY_BY_TYPE),
        scoring_reliability_by_action=dict(DEFAULT_SCORING_RELIABILITY_BY_ACTION),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)

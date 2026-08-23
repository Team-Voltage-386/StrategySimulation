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

from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators
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


# -- specialised archetypes --------------------------------------------
#
# Two deliberately lopsided robots, for exercising what the sim does with
# an alliance whose members are not interchangeable. Both are additions
# beside `build_characteristics`, never replacements for it: the
# reference robot is what every existing bench number in this repo was
# measured against, and giving *it* side_manipulators would move all of
# them at once (and, measured, would drop its ALGAE collection to zero --
# see test_reefscape_heterogeneous.py).
#
# The thing worth understanding before reading either: a side with no
# SideManipulators entry can do nothing at all. `side_manipulators` is
# all-or-nothing per robot -- once any side is configured, every
# capability the robot has must be spelled out, since an absent side is
# "unconfigured", not "default". `intake_source` then splits *where* a
# side may take a piece from: "field" is a loose GamePiece lying on the
# carpet, "station" is a dwell-and-be-handed-one IntakeLocation, "both"
# (the default) is either.
#
# That distinction does real work in REEFSCAPE, because the ALGAE staged
# on the REEF is modeled as an IntakeLocation, not a loose piece (see
# field.py's reef_algae_staging_zones). So "floor pickup only" is not a
# flavour note -- it decides whether a robot can clear a REEF gate at
# all.

ALGAE_SWEEPER_SIDE = "right"
CORAL_STATION_SIDE = "back"


def build_algae_sweeper_characteristics(name: str = "algae-sweeper", **overrides) -> RobotCharacteristics:
    """ALGAE only, floor only, everything through the right side.

    Cannot touch CORAL, and cannot take ALGAE from an IntakeLocation --
    which in this game means it cannot clear a REEF face's staged ALGAE
    however long it sits there, only sweep up ALGAE already loose on the
    carpet (the CORAL MARK staging, or a piece another robot dropped).
    It is the alliance partner that looks like an ALGAE specialist and
    conspicuously fails to unblock anybody's L2/L3."""
    defaults = dict(
        name=name,
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type={ALGAE_TYPE: 1},
        intake_time_by_type={ALGAE_TYPE: 0.4}, station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5, deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=frozenset({ALGAE_TYPE}),
        scoring_reliability_by_type={ALGAE_TYPE: 1.0},
        scoring_reliability_by_action=dict(DEFAULT_SCORING_RELIABILITY_BY_ACTION),
        side_manipulators={
            ALGAE_SWEEPER_SIDE: SideManipulators(
                intake_piece_types=frozenset({ALGAE_TYPE}),
                score_piece_types=frozenset({ALGAE_TYPE}),
                intake_source="field",
            ),
        },
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def build_station_cycler_characteristics(name: str = "station-cycler", **overrides) -> RobotCharacteristics:
    """CORAL in through the back from a CORAL STATION only; CORAL out,
    and all ALGAE handling, through the front.

    The interesting one, because its two jobs point opposite directions:
    every CORAL cycle has to present the back to a station and then the
    front to a REEF face, so the turnaround is a real cost the sim charges
    rather than an assumption. Its front takes ALGAE from either source
    (intake_source defaults to "both"), so unlike the sweeper above it
    *can* clear a REEF gate."""
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
        side_manipulators={
            CORAL_STATION_SIDE: SideManipulators(
                intake_piece_types=frozenset({CORAL_TYPE}),
                intake_source="station",
            ),
            "front": SideManipulators(
                intake_piece_types=frozenset({ALGAE_TYPE}),
                score_piece_types=frozenset({CORAL_TYPE, ALGAE_TYPE}),
            ),
        },
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)

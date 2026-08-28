"""
Tunable robot design parameters -- the axes a Monte Carlo sweep varies
to compare design concepts. Every field here should be something a
design trade could plausibly change; behavior/strategy belongs in
control/behavior.py, not here.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# Robot-relative physical sides a manipulator can be mounted on. "front" is
# the chassis's own +x (forward) face; "left"/"right" are relative to that
# forward direction (a CCW heading rotation from +x reaches "left" first),
# not field-relative -- matches how a driver would describe their own robot.
SIDES: tuple[str, ...] = ("front", "back", "left", "right")

# Outward unit normal of each physical side, robot-relative (+x forward,
# CCW-positive heading) -- matches SIDES' convention. Shared by Robot
# (builds each side's intake sensor wedge / picks an eject direction) and
# Match (checks whether a deposit's side was actually facing the scoring
# region it landed in).
SIDE_OUTWARD: dict[str, tuple[float, float]] = {
    "front": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0),
}

# Where a side's intake can actually pick a piece up from: a loose piece
# sitting on the field, a human-player collection region (IntakeLocation
# station), or both. "both" is the default so a caller that never sets
# this keeps the old undifferentiated behavior.
INTAKE_SOURCES: tuple[str, ...] = ("field", "station", "both")


@dataclass(frozen=True)
class SideManipulators:
    """What a robot can physically do through one side. Both piece-type
    fields are explicit membership sets (unlike
    RobotCharacteristics.accepted_piece_types, an empty set here means
    "nothing", not "any type") -- this type only exists to describe a side
    a caller actually configured, so there is no ambiguous "unconfigured"
    state to default against."""
    intake_piece_types: frozenset[str] = frozenset()
    score_piece_types: frozenset[str] = frozenset()
    intake_source: str = "both"  # one of INTAKE_SOURCES


@dataclass(frozen=True)
class RobotCharacteristics:
    name: str = "robot"

    # Chassis
    max_speed: float = 150.0          # in/s
    max_accel: float = 250.0          # in/s^2
    max_angular_speed: float = 6.0    # rad/s
    max_angular_accel: float = 20.0   # rad/s^2
    width: float = 28.0               # in, bumper-to-bumper
    length: float = 28.0              # in, bumper-to-bumper
    mass: float = 15.0                # arbitrary pymunk mass units

    # Mechanisms
    piece_capacity: int = 1
    # Per-piece-type capacity override, e.g. {"coral": 1, "algae": 1} -- lets a
    # robot hold one of each simultaneously rather than a single shared pool.
    # Empty (default) falls back to the scalar `piece_capacity` above, shared
    # across all types, matching the legacy single-pool behavior.
    piece_capacity_by_type: dict[str, int] = field(default_factory=dict)
    starting_piece_count: int = 0
    # Which piece type Match.add_robot preloads starting_piece_count of,
    # e.g. "coral" for a robot that always starts holding CORAL regardless
    # of which types accepted_piece_types happens to iterate first (a
    # frozenset's iteration order isn't a type ordering). None (default)
    # falls back to that legacy first-accepted-type behavior.
    preload_piece_type: str | None = None
    intake_time: float = 0.5          # seconds to capture a field piece, for a type with no override below
    # Per-piece-type intake time override, e.g. {"coral": 0.4, "algae": 0.6} --
    # different piece types can plausibly take a mechanism different time to
    # secure. Types not listed here fall back to `intake_time`.
    intake_time_by_type: dict[str, float] = field(default_factory=dict)
    station_intake_time: float = 0.5  # seconds to capture from a human-player station
    deposit_time: float = 0.5         # seconds to complete a deposit, for an action with no override below
    # Per-action deposit time override, e.g. {"l1_coral": 0.4, "l4_coral": 1.6} --
    # a design trade like "reach a taller scoring location" costs more
    # cycle time on some robots than others, which is exactly the kind of
    # thing a Monte Carlo sweep over RobotCharacteristics wants to compare.
    # Actions not listed here fall back to `deposit_time`.
    deposit_time_by_action: dict[str, float] = field(default_factory=dict)
    intake_range: float = 4.0         # in, how far the intake sensor extends beyond the bumper
    accepted_piece_types: frozenset[str] = field(default_factory=frozenset)  # empty = accepts any type

    # Per-piece-type probability, in [0, 1], that a deliberate scoring attempt
    # (Match.step's explicit deposit-into-a-ready-region path) actually lands,
    # e.g. {"coral": 0.9, "algae": 0.5} for a robot whose algae mechanism is
    # less consistent than its coral one. A type missing from this dict scores
    # every time (1.0), matching the legacy deterministic behavior -- so an
    # empty dict (the default) never rolls any dice and costs nothing.
    scoring_reliability_by_type: dict[str, float] = field(default_factory=dict)

    # How often a given scoring *action* lands, as a multiplier on the
    # per-type reliability above -- e.g. {"l4": 0.82, "l1": 0.98} for a
    # robot that puts CORAL on the top branch less surely than in the
    # trough. An action missing from this dict does not scale its type's
    # number at all (1.0), so an empty dict (the default) leaves the
    # per-type behavior exactly as it was.
    #
    # A multiplier rather than an override so the two axes stay
    # independent and both stay meaningful: the type says how good this
    # robot's mechanism for that game piece is, the action says how hard
    # that particular target is. Overriding would silently kill the
    # per-type control the moment any action was configured.
    scoring_reliability_by_action: dict[str, float] = field(default_factory=dict)

    # Which physical side(s) can intake/score which piece types, e.g.
    # {"back": SideManipulators(intake_piece_types={"coral"}),
    #  "front": SideManipulators(score_piece_types={"coral"}),
    #  "right": SideManipulators(intake_piece_types={"algae"}, score_piece_types={"algae"})}.
    # A side missing from this dict has no capability at all. The default
    # (empty dict) means "not configured" and falls back to the legacy
    # single-front-manipulator layout below: a caller that constructs
    # RobotCharacteristics without this field keeps working exactly as
    # before. A caller that DOES configure this should always include an
    # entry (even an empty SideManipulators()) for every side it cares
    # about, since an empty dict is indistinguishable from "not configured".
    side_manipulators: dict[str, SideManipulators] = field(default_factory=dict)

    # How fast a released piece leaves the scoring side, in in/s, on top
    # of the chassis's own velocity. The default is the nudge that only
    # has to clear the robot's own footprint so an adjacent scoring
    # region can take the piece.
    #
    # It becomes a real design parameter the moment a game rewards
    # *throwing* a piece somewhere rather than placing it -- `Pass` is
    # the tactic that does, and at the default it moves a piece about a
    # foot, which is not a pass. A robot that can throw says so here.
    eject_speed: float = 24.0

    def deposit_duration(self, action: str) -> float:
        return self.deposit_time_by_action.get(action, self.deposit_time)

    def intake_duration(self, piece_type: str) -> float:
        return self.intake_time_by_type.get(piece_type, self.intake_time)

    def capacity_for(self, piece_type: str) -> int:
        if self.piece_capacity_by_type:
            return self.piece_capacity_by_type.get(piece_type, 0)
        return self.piece_capacity

    def reliability_for(self, piece_type: str, action: str | None = None) -> float:
        """How often a scoring attempt lands. `action`, when known,
        scales the piece type's own reliability by that action's -- see
        `scoring_reliability_by_action`. Both default to 1.0, so an
        unconfigured robot still scores every time."""
        base = self.scoring_reliability_by_type.get(piece_type, 1.0)
        if action is None:
            return base
        return base * self.scoring_reliability_by_action.get(action, 1.0)

    def side_intake_accepts(self, side: str, piece_type: str, source: str | None = None) -> bool:
        """`source`, when given ("field" or "station"), additionally
        requires that side's `intake_source` to match it (a side set to
        "both" always matches). None means "don't care about source" --
        used by callers that only need physical-capability gating, e.g.
        the intake sensor wedge and `active_intake_sides`."""
        if not self.side_manipulators:
            return side == "front" and (not self.accepted_piece_types or piece_type in self.accepted_piece_types)
        cfg = self.side_manipulators.get(side)
        if cfg is None or piece_type not in cfg.intake_piece_types:
            return False
        if source is not None and cfg.intake_source != "both" and cfg.intake_source != source:
            return False
        return True

    def side_score_accepts(self, side: str, piece_type: str) -> bool:
        if not self.side_manipulators:
            return side == "front" and (not self.accepted_piece_types or piece_type in self.accepted_piece_types)
        cfg = self.side_manipulators.get(side)
        return cfg is not None and piece_type in cfg.score_piece_types

    def active_intake_sides(self) -> tuple[str, ...]:
        """Every side with at least one intake-eligible piece type --
        Robot builds one intake sensor shape per side returned here."""
        if not self.side_manipulators:
            return ("front",)
        return tuple(s for s, cfg in self.side_manipulators.items() if cfg.intake_piece_types)

    def score_side_for(self, piece_type: str) -> str:
        """First side (in SIDES order) configured to score `piece_type`,
        or "front" if none is -- Robot uses this to pick which edge a
        completed deposit ejects the piece from."""
        for side in SIDES:
            if self.side_score_accepts(side, piece_type):
                return side
        return "front"

    def intake_side_for(self, piece_type: str) -> str:
        """First side (in SIDES order) configured to intake `piece_type`,
        or "front" if none is -- Collect uses this to pick which edge it
        rotates toward a target piece/station, mirroring score_side_for's
        role for the scoring side."""
        for side in SIDES:
            if self.side_intake_accepts(side, piece_type):
                return side
        return "front"

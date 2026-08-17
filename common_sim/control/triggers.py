"""
Declarative, serializable trigger conditions for the strategy arbiter
(strategy.py). A Trigger is a pure function of state -- dataclass fields
only, no internal timers -- so it round-trips cleanly through
strategy_io.py and stays trivially unit-testable. `for_duration`
hysteresis (must hold true for N seconds before firing) is bookkept by
the caller (StrategyController), not the trigger itself, so a trigger
never needs to know how long it's been ticked.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, ClassVar

from common_sim.control import world_view
from common_sim.control.param import Param

if TYPE_CHECKING:  # pragma: no cover
    from common_sim.control.behavior import BehaviorContext


@dataclass
class Trigger(ABC):
    for_duration: float | None = None

    # ClassVar so dataclass doesn't treat this as a field -- it's a
    # per-class schema declaration, not per-instance state. Every
    # Trigger also has `for_duration`, which a GUI shows once, outside
    # this schema, the same way it's declared once here rather than
    # repeated per subclass.
    PARAM_SCHEMA: ClassVar[tuple[Param, ...]] = ()

    @abstractmethod
    def evaluate(self, ctx: "BehaviorContext") -> bool:
        raise NotImplementedError

    def children(self) -> tuple["Trigger", ...]:
        """Sub-triggers this one composes, for a caller that has to walk
        the tree (StrategyController's `for_duration` check). Empty for
        every leaf trigger, which is most of them."""
        return ()

    def describe(self) -> str:
        return self.__class__.__name__


@dataclass
class Always(Trigger):
    PARAM_SCHEMA = ()

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        return True

    def describe(self) -> str:
        return "always"


@dataclass
class PiecesAvailable(Trigger):
    piece_type: str | None = None
    min_count: int | None = None
    max_count: int | None = None
    within: float | None = None  # inches from the robot; None = anywhere on field

    PARAM_SCHEMA = (
        Param("piece_type", kind="piece_type", default=None, optional=True),
        Param("min_count", kind="int", default=None, optional=True, min=0),
        Param("max_count", kind="int", default=None, optional=True, min=0),
        Param("within", kind="float", default=None, optional=True, min=0, suffix=" in"),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        pieces = world_view.collectable_pieces(ctx.match, piece_type=self.piece_type, robot=ctx.robot)
        if self.within is not None:
            origin = ctx.robot.pose.translation
            pieces = [p for p in pieces if origin.get_distance(p.position) <= self.within]
        count = len(pieces)
        if self.min_count is not None and count < self.min_count:
            return False
        if self.max_count is not None and count > self.max_count:
            return False
        return True

    def describe(self) -> str:
        parts = [self.piece_type or "pieces"]
        if self.min_count is not None:
            parts.append(f">= {self.min_count}")
        if self.max_count is not None:
            parts.append(f"<= {self.max_count}")
        if self.within is not None:
            parts.append(f"within {self.within}in")
        return " ".join(parts) + " available"


@dataclass
class MatchTime(Trigger):
    after: float | None = None
    before: float | None = None
    phase: str | None = None
    remaining_under: float | None = None  # convenience: elapsed >= total_duration - N

    PARAM_SCHEMA = (
        Param("after", kind="float", default=None, optional=True, min=0, suffix=" s"),
        Param("before", kind="float", default=None, optional=True, min=0, suffix=" s"),
        Param("phase", kind="choice", default=None, optional=True, choices=("auto", "teleop")),
        Param("remaining_under", kind="float", default=None, optional=True, min=0, suffix=" s"),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        match = ctx.match
        elapsed = match.elapsed
        if self.after is not None and elapsed < self.after:
            return False
        if self.before is not None and elapsed >= self.before:
            return False
        if self.phase is not None and match.phase.value != self.phase:
            return False
        if self.remaining_under is not None:
            remaining = match.config.total_duration - elapsed
            if remaining > self.remaining_under:
                return False
        return True

    def describe(self) -> str:
        parts = []
        if self.phase is not None:
            parts.append(f"phase={self.phase}")
        if self.after is not None:
            parts.append(f"t>={self.after}s")
        if self.before is not None:
            parts.append(f"t<{self.before}s")
        if self.remaining_under is not None:
            parts.append(f"remaining<{self.remaining_under}s")
        return " ".join(parts) if parts else "always"


@dataclass
class PiecesHeld(Trigger):
    piece_type: str | None = None
    min_count: int | None = None
    max_count: int | None = None

    PARAM_SCHEMA = (
        Param("piece_type", kind="piece_type", default=None, optional=True),
        Param("min_count", kind="int", default=None, optional=True, min=0),
        Param("max_count", kind="int", default=None, optional=True, min=0),
    )

    def _held(self, ctx: "BehaviorContext") -> int:
        held = ctx.robot.held_pieces
        if self.piece_type is not None:
            held = [p for p in held if p.piece_type == self.piece_type]
        return len(held)

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        count = self._held(ctx)
        if self.min_count is not None and count < self.min_count:
            return False
        if self.max_count is not None and count > self.max_count:
            return False
        return True

    def describe(self) -> str:
        parts = [self.piece_type or "pieces", "held"]
        if self.min_count is not None:
            parts.append(f">= {self.min_count}")
        if self.max_count is not None:
            parts.append(f"<= {self.max_count}")
        return " ".join(parts)


@dataclass
class AtCapacity(Trigger):
    """Sugar over PiecesHeld that survives a capacity edit: fires once
    held count for `piece_type` reaches whatever
    RobotCharacteristics.capacity_for currently says, rather than
    hardcoding a number that would silently drift out of sync."""
    piece_type: str | None = None

    PARAM_SCHEMA = (
        Param("piece_type", kind="piece_type", default=None, optional=True),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        # No piece_type means "at capacity for everything it can hold",
        # not "holding as many pieces as the shared pool allows" -- with
        # a per-type capacity those differ, and only the former is what
        # "at capacity" can sensibly mean.
        return ctx.robot.is_full_for(self.piece_type)

    def describe(self) -> str:
        return f"at capacity ({self.piece_type or 'any'})"


@dataclass
class ScoringAvailable(Trigger):
    min_value: float | None = None
    region: str | None = None

    PARAM_SCHEMA = (
        Param("min_value", kind="float", default=None, optional=True, min=0, suffix=" pts"),
        Param("region", kind="region_name", default=None, optional=True),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        options = world_view.scoring_options(ctx.match, ctx.robot)
        if self.region is not None:
            options = [o for o in options if o.region.name == self.region]
        if not options:
            return False
        if self.min_value is not None:
            values = [
                ctx.match.scoring_rules.points_for(o.action, ctx.match.phase.value)
                for o in options
            ]
            return max(values) >= self.min_value
        return True

    def describe(self) -> str:
        parts = ["scoring available"]
        if self.region is not None:
            parts.append(f"at {self.region}")
        if self.min_value is not None:
            parts.append(f">= {self.min_value}pts")
        return " ".join(parts)


@dataclass
class OpponentNear(Trigger):
    region: str | None = None
    within: float = 60.0

    PARAM_SCHEMA = (
        Param("region", kind="region_name", default=None, optional=True),
        Param("within", kind="float", default=60.0, min=0, suffix=" in"),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        match = ctx.match
        robot = ctx.robot
        if self.region is not None:
            target = world_view.region_by_name(match, self.region)
            if target is None:
                return False
            origin = world_view.region_centroid(target)
        else:
            origin = (robot.pose.x, robot.pose.y)
        for opponent in world_view.opponents(match, robot.alliance):
            dx = opponent.pose.x - origin[0]
            dy = opponent.pose.y - origin[1]
            if (dx * dx + dy * dy) ** 0.5 <= self.within:
                return True
        return False

    def describe(self) -> str:
        where = f"near {self.region}" if self.region is not None else "near us"
        return f"opponent {where} (within {self.within}in)"


@dataclass
class BeingDefended(Trigger):
    """True while an opponent has a *registered* defensive intent aimed
    at this robot -- see world_view.defenders_against and
    strategy.Intent.defending.

    The counterpart to OpponentNear, and worth having separately: an
    opponent near us might be cycling past on its own business, and
    reorganising the whole match plan around ordinary traffic is how a
    strategy ends up doing nothing. A robot that has *declared* it is
    there to deny us is a different fact, and one that stays true --
    which is what makes it worth reacting to at strategy level (drop to
    a cheaper scoring target, go feed instead of cycling, pull the
    defender away from a teammate) rather than inside a tactic.

    Pair with `for_duration` for "has been on me for N seconds", which is
    usually what you want: a defender takes a moment to commit, and a
    rule that fires the instant one glances at us will chatter.

    `within` requires the defender to actually be close enough to be
    doing something about it; `region` narrows to defenders denying one
    specific region."""
    within: float | None = None   # inches from us; None = anywhere on field
    region: str | None = None

    PARAM_SCHEMA = (
        Param("within", kind="float", default=None, optional=True, min=0, suffix=" in"),
        Param("region", kind="region_name", default=None, optional=True),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        robot = ctx.robot
        origin = robot.pose.translation
        for defender in world_view.defenders_against(ctx.match, robot):
            if self.within is not None and origin.get_distance(defender.pose.translation) > self.within:
                continue
            if self.region is not None and getattr(defender.intent, "target_region", None) != self.region:
                continue
            return True
        return False

    def describe(self) -> str:
        parts = ["being defended"]
        if self.region is not None:
            parts.append(f"at {self.region}")
        if self.within is not None:
            parts.append(f"within {self.within}in")
        return " ".join(parts)


@dataclass
class AllOf(Trigger):
    triggers: tuple[Trigger, ...] = field(default_factory=tuple)

    PARAM_SCHEMA = (
        Param("triggers", kind="trigger_list", default=()),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        return all(t.evaluate(ctx) for t in self.triggers)

    def children(self) -> tuple[Trigger, ...]:
        return tuple(self.triggers)

    def describe(self) -> str:
        return " AND ".join(t.describe() for t in self.triggers) or "always"


@dataclass
class AnyOf(Trigger):
    triggers: tuple[Trigger, ...] = field(default_factory=tuple)

    PARAM_SCHEMA = (
        Param("triggers", kind="trigger_list", default=()),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        return any(t.evaluate(ctx) for t in self.triggers)

    def children(self) -> tuple[Trigger, ...]:
        return tuple(self.triggers)

    def describe(self) -> str:
        return " OR ".join(t.describe() for t in self.triggers) or "never"


@dataclass
class Not(Trigger):
    trigger: Trigger | None = None

    PARAM_SCHEMA = (
        Param("trigger", kind="trigger", default=None, optional=True),
    )

    def evaluate(self, ctx: "BehaviorContext") -> bool:
        assert self.trigger is not None
        return not self.trigger.evaluate(ctx)

    def children(self) -> tuple[Trigger, ...]:
        return (self.trigger,) if self.trigger is not None else ()

    def describe(self) -> str:
        return f"NOT ({self.trigger.describe() if self.trigger else '?'})"

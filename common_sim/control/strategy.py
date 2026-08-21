"""
The arbiter: turns a data-described Strategy (Rule[] : trigger, tactic,
priority, dwell/cooldown) into a live decision each tick -- evaluate
every rule's trigger, pick the highest-priority satisfied one, tick its
tactic, and log every switch. Everything here is orchestration; the
actual decisions live in triggers.py/tactics.py.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.control.tactics import Collect, Defend, Idle, Score
from common_sim.control.triggers import Trigger

if TYPE_CHECKING:  # pragma: no cover
    from common_sim.robot.robot import Robot


@dataclass
class Rule:
    name: str
    trigger: Trigger
    tactic: Behavior
    priority: int = 0
    min_duration: float = 0.0   # hysteresis: can't be preempted by equal/lower priority before this
    cooldown: float = 0.0       # can't re-fire for N s after ending
    once: bool = False


@dataclass
class Strategy:
    name: str
    rules: list[Rule] = field(default_factory=list)
    fallback: Behavior = field(default_factory=Idle)


@dataclass
class Intent:
    """What the active tactic is currently trying to do -- read by
    Defend on opposing robots (`target == "opponent_intent"`) and by a
    GUI to draw the active target on the field canvas.

    `defending`/`marking` are the *counter*-defense half of that same
    channel: a denial tactic declares not just where it's going but that
    the reason is denial, and whom it's denying. Without the flag a
    defender parked on a scoring region is indistinguishable from a
    teammate about to score there, so the robot being denied can't tell
    "that spot is busy for a moment" from "that spot is being taken away
    from me for the rest of the match" -- and those want opposite
    responses (wait vs. go somewhere else)."""
    tactic_name: str
    target_region: str | None = None
    target_piece: object = None
    target_pose: object = None
    defending: bool = False
    marking: object = None  # the Robot this denial is aimed at, when known


def _reject_nested_for_duration(rule: Rule) -> None:
    """`for_duration` is bookkept here, against the rule's own trigger --
    see triggers.py's module docstring. A trigger nested inside an
    AllOf/AnyOf/Not is evaluated by its parent, which calls plain
    `evaluate()` and never consults the hysteresis clock, so a
    `for_duration` down there is silently ignored: the rule fires
    immediately and a strategy that reads as "once a defender has been on
    me for two seconds" behaves as "the instant one glances at me".

    Silently, and for as long as nobody thinks to measure it -- which is
    exactly how it was found. So refuse to build the controller at all
    rather than run a strategy that doesn't mean what it says. The fix is
    always to lift the duration onto the outermost trigger, which is what
    the author meant anyway."""
    pending = list(rule.trigger.children())
    while pending:
        node = pending.pop()
        if node.for_duration:
            raise ValueError(
                f"rule {rule.name!r}: for_duration={node.for_duration} on a nested "
                f"{type(node).__name__} is never applied -- move it to the rule's "
                f"outermost trigger ({type(rule.trigger).__name__})"
            )
        pending.extend(node.children())


# How long a rule whose tactic reported FAILURE is held out of the running
# so a lower-priority rule can have the robot.
#
# Without this a FAILURE is indistinguishable from a rule that never ran:
# `_best_candidate` re-picks the highest-priority *satisfied* rule, status
# is not consulted, so the rule that just declared it cannot do its job is
# immediately handed the robot again -- and stays there for as long as its
# trigger holds, which for a supply-availability trigger is the rest of the
# match. There is no other edge in the arbiter by which "this job is not
# working" can become "do the other job", and a tactic cannot make that
# call from inside its own scope: a Collect(piece_type="algae") that cannot
# reach any ALGAE has no way to know the strategy also has a CORAL rule.
#
# Measured on blue={algae_processor, cycle_coral} vs red={full_defense,
# cycle_coral}: the ALGAE cycler committed to the last ALGAE on the field,
# at rest inside a corner CORAL STATION with the defender denying that
# station parked on top of it, and blue's REEF already stripped. The
# trigger (`PiecesAvailable(algae) >= 1`) was true the whole time -- the
# piece existed, it just could not be had -- so the robot held the job it
# could not do for 22s while its CORAL rule sat one priority below,
# satisfied and ready.
#
# Short on purpose. This is a yield, not a ban: long enough for the next
# rule to be picked and commit to a target of its own, and it re-arms every
# time the failure repeats, so a persistently blocked job stays yielded
# while a transient one costs a single tick. Making it outlast the cause
# instead (e.g. matching Collect's piece cooldown) would mean guessing here
# at how long somebody else's give-up lasts.
_FAILED_RULE_SUPPRESSION = 1.0


def _delegate(tactic: Behavior) -> Behavior:
    """The tactic actually driving the robot, following through any
    arbiter that runs a child tactic instead of doing the work itself
    (Pursue).

    Intent is not decoration: `Collect._station_has_room_for` races two
    robots for one feeder on it, `Collect._piece_contenders` splits them
    off one piece with it, `world_view.region_occupants` and
    `region_denied_by` and `triggers.BeingDefended` read it to tell
    denial from a teammate passing through. A tactic that publishes
    nothing is invisible to every one of those, so an arbiter has to
    report what its child is doing rather than its own name -- otherwise
    two Pursue robots on an alliance would converge on the same feeder,
    which is precisely the failure all that machinery exists to prevent.

    Followed here rather than having the arbiter forge an Intent of its
    own, so `_update_intent` below stays the single place that knows how
    each kind of tactic describes itself. The depth cap is a guard against
    a cycle, not an expectation of nesting."""
    for _ in range(4):
        child = getattr(tactic, "active_tactic", None)
        if child is None:
            break
        tactic = child
    return tactic


class _RuleState:
    __slots__ = ("duration_true", "cooldown_remaining", "fired_once", "failure_suppressed")

    def __init__(self):
        self.duration_true = 0.0
        self.cooldown_remaining = 0.0
        self.fired_once = False
        self.failure_suppressed = 0.0


class StrategyController:
    def __init__(self, strategy: Strategy, robot: "Robot"):
        self.strategy = strategy
        self.robot = robot

        for rule in strategy.rules:
            _reject_nested_for_duration(rule)

        self._states: dict[str, _RuleState] = {rule.name: _RuleState() for rule in strategy.rules}
        self._active_rule: Rule | None = None
        self._active_tactic: Behavior = strategy.fallback
        self._active_elapsed = 0.0
        self._last_status: Status = Status.RUNNING
        self.intent = Intent(tactic_name=type(strategy.fallback).__name__)

    @property
    def active_rule_name(self) -> str | None:
        """Name of the currently active Rule, or None while the fallback
        tactic is active -- for a GUI (strategy_graph.py) to know which
        node to highlight without reaching into the private
        `_active_rule`."""
        return self._active_rule.name if self._active_rule is not None else None

    def _evaluate_all(self, ctx: BehaviorContext) -> dict[str, bool]:
        satisfied: dict[str, bool] = {}
        for rule in self.strategy.rules:
            state = self._states[rule.name]
            raw = rule.trigger.evaluate(ctx)

            if rule.trigger.for_duration:
                state.duration_true = state.duration_true + ctx.dt if raw else 0.0
                ok = raw and state.duration_true >= rule.trigger.for_duration
            else:
                ok = raw

            if state.cooldown_remaining > 0.0:
                state.cooldown_remaining = max(0.0, state.cooldown_remaining - ctx.dt)
                ok = False
            # Decayed here rather than where it is charged, so it ticks down
            # on every rule every tick exactly like the author's cooldown
            # beside it (see `_FAILED_RULE_SUPPRESSION`).
            if state.failure_suppressed > 0.0:
                state.failure_suppressed = max(0.0, state.failure_suppressed - ctx.dt)
                ok = False
            if rule.once and state.fired_once:
                ok = False

            satisfied[rule.name] = ok
        return satisfied

    def _best_candidate(self, satisfied: dict[str, bool]) -> Rule | None:
        best: Rule | None = None
        for rule in self.strategy.rules:
            if not satisfied[rule.name]:
                continue
            if best is None or rule.priority > best.priority:
                best = rule
        return best

    def _switch_to(self, ctx: BehaviorContext, rule: Rule | None) -> None:
        outgoing = self._active_rule
        if outgoing is not None:
            self._active_tactic.reset()
            # A preempted tactic (trigger went false, or a higher-priority
            # rule won) never gets another tick to run its own SUCCESS/
            # FAILURE cleanup -- e.g. Collect's "just captured" branch is
            # what turns robot.set_intake_active back off. Without this,
            # a mid-collect preemption leaves intake commanded on forever,
            # and the robot keeps scooping up whatever compatible piece it
            # next drives near. The incoming tactic (ticked immediately
            # below, same frame) re-asserts whatever it actually wants.
            if ctx.robot is not None:
                ctx.robot.set_intake_active(False)
                ctx.robot.set_deposit_active(False)
            if outgoing.cooldown > 0.0:
                self._states[outgoing.name].cooldown_remaining = outgoing.cooldown

        self._active_rule = rule
        self._active_tactic = rule.tactic if rule is not None else self.strategy.fallback
        self._active_elapsed = 0.0
        self._last_status = Status.RUNNING
        if rule is not None:
            self._states[rule.name].fired_once = True

        match = ctx.match
        if match is not None:
            match.events.log(match.elapsed, "behavior_change", {
                "robot": ctx.robot,
                "from": outgoing.name if outgoing is not None else "fallback",
                "to": rule.name if rule is not None else "fallback",
                "trigger": rule.trigger.describe() if rule is not None else None,
            })

    def tick(self, ctx: BehaviorContext) -> None:
        # Charged before the triggers are evaluated, because suppression has
        # to be in place for the candidate scan that happens further down
        # this same tick -- charge it at switch time instead and the rule
        # that just failed wins that scan and is re-selected, which is the
        # behavior this exists to prevent.
        if self._active_rule is not None and self._last_status is Status.FAILURE:
            self._states[self._active_rule.name].failure_suppressed = _FAILED_RULE_SUPPRESSION

        satisfied = self._evaluate_all(ctx)
        best = self._best_candidate(satisfied)

        if self._active_rule is None:
            if best is not None:
                self._switch_to(ctx, best)
        else:
            active = self._active_rule
            active_still_satisfied = satisfied.get(active.name, False)
            should_switch = False
            target: Rule | None = active

            if best is not None and best is not active:
                if best.priority > active.priority:
                    should_switch, target = True, best
                elif self._active_elapsed >= active.min_duration:
                    should_switch, target = True, best

            if not should_switch and not active_still_satisfied:
                should_switch, target = True, best  # best may be None -> fallback
            if not should_switch and self._last_status in (Status.SUCCESS, Status.FAILURE):
                should_switch, target = True, best  # best may be None -> fallback

            if should_switch:
                self._switch_to(ctx, target)

        self._active_elapsed += ctx.dt
        self._last_status = self._active_tactic.tick(ctx)
        self._update_intent()

    def _update_intent(self) -> None:
        tactic = _delegate(self._active_tactic)
        tactic_name = type(tactic).__name__
        target_region: str | None = None
        target_piece = None
        defending = False
        marking = None

        if isinstance(tactic, Score) and tactic._current is not None:
            target_region = tactic._current.region.name
            target_piece = tactic._current.piece
        elif isinstance(tactic, Collect):
            target_piece = tactic._target_piece
            if tactic._target_station is not None:
                target_region = tactic._target_station.name
        elif isinstance(tactic, Defend):
            target_region = tactic.target_region_name
            defending = True
            marking = tactic.marked_robot

        self.intent = Intent(
            tactic_name=tactic_name, target_region=target_region, target_piece=target_piece,
            defending=defending, marking=marking,
        )

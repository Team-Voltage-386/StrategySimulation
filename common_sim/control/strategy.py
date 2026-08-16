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
    GUI to draw the active target on the field canvas."""
    tactic_name: str
    target_region: str | None = None
    target_piece: object = None
    target_pose: object = None


class _RuleState:
    __slots__ = ("duration_true", "cooldown_remaining", "fired_once")

    def __init__(self):
        self.duration_true = 0.0
        self.cooldown_remaining = 0.0
        self.fired_once = False


class StrategyController:
    def __init__(self, strategy: Strategy, robot: "Robot"):
        self.strategy = strategy
        self.robot = robot

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
        tactic = self._active_tactic
        tactic_name = type(tactic).__name__
        target_region: str | None = None
        target_piece = None

        if isinstance(tactic, Score) and tactic._current is not None:
            target_region = tactic._current.region.name
            target_piece = tactic._current.piece
        elif isinstance(tactic, Collect):
            target_piece = tactic._target_piece
            if tactic._target_station is not None:
                target_region = tactic._target_station.name
        elif isinstance(tactic, Defend):
            target_region = tactic.target_region_name

        self.intent = Intent(tactic_name=tactic_name, target_region=target_region, target_piece=target_piece)

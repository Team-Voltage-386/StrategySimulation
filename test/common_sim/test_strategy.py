"""
strategy.py arbiter tests: priority preemption, min_duration hysteresis,
cooldown, once, fallback, and event logging -- using small fake
triggers/tactics so the tests exercise only the arbiter's own logic, not
navigation/physics.
"""
from dataclasses import dataclass, field

from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.control.strategy import Rule, Strategy, StrategyController
from common_sim.control.triggers import Trigger
from common_sim.match.events import EventLog


class FakeMatch:
    def __init__(self):
        self.elapsed = 0.0
        self.events = EventLog()


@dataclass
class FlagTrigger(Trigger):
    """Fires whenever `flag()` returns True -- lets a test flip
    conditions on demand without a real world_view query."""
    flag: callable = field(default=lambda: False)

    def evaluate(self, ctx) -> bool:
        return self.flag()

    def describe(self) -> str:
        return "flag"


class CountingTactic(Behavior):
    """Runs forever unless `finish_after` ticks have elapsed, then
    returns `terminal_status`. Tracks tick/reset counts for assertions."""

    def __init__(self, finish_after: int | None = None, terminal_status: Status = Status.SUCCESS):
        self.finish_after = finish_after
        self.terminal_status = terminal_status
        self.ticks = 0
        self.total_ticks = 0
        self.resets = 0

    def tick(self, ctx: BehaviorContext) -> Status:
        self.ticks += 1
        self.total_ticks += 1
        if self.finish_after is not None and self.ticks >= self.finish_after:
            return self.terminal_status
        return Status.RUNNING

    def reset(self) -> None:
        self.resets += 1
        self.ticks = 0


def ctx_for(match, robot=None, dt=1.0 / 60.0) -> BehaviorContext:
    return BehaviorContext(robot=robot, dt=dt, match=match)


def test_fallback_used_when_no_rule_satisfied():
    fallback = CountingTactic()
    strategy = Strategy(name="s", rules=[], fallback=fallback)
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))
    assert fallback.ticks == 1
    assert controller.intent.tactic_name == "CountingTactic"


def test_higher_priority_immediately_preempts():
    low_tactic = CountingTactic()
    high_tactic = CountingTactic()
    high_flag = [False]
    low = Rule(name="low", trigger=FlagTrigger(flag=lambda: True), tactic=low_tactic, priority=1)
    high = Rule(name="high", trigger=FlagTrigger(flag=lambda: high_flag[0]), tactic=high_tactic, priority=10)
    strategy = Strategy(name="s", rules=[low, high])
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))
    assert controller._active_rule is low
    assert low_tactic.ticks == 1

    high_flag[0] = True
    controller.tick(ctx_for(match))
    assert controller._active_rule is high
    assert high_tactic.ticks == 1
    assert low_tactic.resets == 1


def test_min_duration_blocks_equal_priority_preemption_until_elapsed():
    # "a" is listed first, so once both are satisfied at equal priority
    # the list-order tie-break would always favor "a" -- the only way to
    # observe min_duration hysteresis is to have the *later*-listed rule
    # ("b") active first, then have "a" (list-order winner) become
    # satisfied and have to wait out "b"'s min_duration before taking over.
    a_tactic = CountingTactic()
    b_tactic = CountingTactic()
    a_flag = [False]
    a = Rule(name="a", trigger=FlagTrigger(flag=lambda: a_flag[0]), tactic=a_tactic, priority=1)
    b = Rule(name="b", trigger=FlagTrigger(flag=lambda: True), tactic=b_tactic, priority=1, min_duration=1.0)
    strategy = Strategy(name="s", rules=[a, b])
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match, dt=0.5))
    assert controller._active_rule is b

    a_flag[0] = True
    controller.tick(ctx_for(match, dt=0.5))  # only 0.5s elapsed on "b" so far -- still under min_duration
    assert controller._active_rule is b

    controller.tick(ctx_for(match, dt=0.5))  # now >= 1.0s -- "a" can take over
    assert controller._active_rule is a


def test_terminal_status_restarts_tactic_still_the_best_candidate():
    """A terminal status forces a re-arbitration, not just a no-op
    continuation -- if the same rule is still the best satisfied
    candidate, that shows up as the tactic being reset and re-run (the
    natural way a one-shot Collect/Score keeps cycling under a
    still-true trigger without needing `once`)."""
    tactic = CountingTactic(finish_after=2)
    rule = Rule(name="r", trigger=FlagTrigger(flag=lambda: True), tactic=tactic, priority=1)
    strategy = Strategy(name="s", rules=[rule])
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))  # tick 1 -- still running
    assert controller._active_rule is rule
    controller.tick(ctx_for(match))  # tick 2 -- tactic finishes (SUCCESS) on this very tick
    assert tactic.resets == 0
    controller.tick(ctx_for(match))  # re-arbitration: still the best candidate -> reset and restart
    assert controller._active_rule is rule
    assert tactic.resets == 1
    assert tactic.ticks == 1
    assert tactic.total_ticks == 3


def test_rule_trigger_going_false_switches_away():
    tactic = CountingTactic()
    flag = [True]
    rule = Rule(name="r", trigger=FlagTrigger(flag=lambda: flag[0]), tactic=tactic, priority=1)
    strategy = Strategy(name="s", rules=[rule], fallback=CountingTactic())
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))
    assert controller._active_rule is rule

    flag[0] = False
    controller.tick(ctx_for(match))
    assert controller._active_rule is None


def test_cooldown_prevents_immediate_refire():
    tactic_a = CountingTactic()
    tactic_b = CountingTactic()
    a_flag = [True]
    a = Rule(name="a", trigger=FlagTrigger(flag=lambda: a_flag[0]), tactic=tactic_a, priority=1, cooldown=1.0)
    b = Rule(name="b", trigger=FlagTrigger(flag=lambda: True), tactic=tactic_b, priority=1)
    strategy = Strategy(name="s", rules=[a, b])
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))
    assert controller._active_rule is a

    a_flag[0] = False  # a's own trigger goes false -> switches to b, a enters cooldown
    controller.tick(ctx_for(match))
    assert controller._active_rule is b

    a_flag[0] = True  # a would fire again, but it's on cooldown
    controller.tick(ctx_for(match, dt=0.1))
    assert controller._active_rule is b


def test_once_only_fires_a_single_time():
    tactic = CountingTactic(finish_after=1)
    fallback = CountingTactic()
    rule = Rule(name="r", trigger=FlagTrigger(flag=lambda: True), tactic=tactic, priority=1, once=True)
    strategy = Strategy(name="s", rules=[rule], fallback=fallback)
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))  # fires once, immediately finishes (SUCCESS)
    assert controller._active_rule is rule
    controller.tick(ctx_for(match))  # switches away (terminal), rule now "fired_once"
    assert controller._active_rule is None
    controller.tick(ctx_for(match))  # rule's trigger is still true, but once blocks re-firing
    assert controller._active_rule is None
    assert tactic.total_ticks == 1


def test_switch_logs_behavior_change_event():
    tactic = CountingTactic()
    rule = Rule(name="r", trigger=FlagTrigger(flag=lambda: True), tactic=tactic, priority=1)
    strategy = Strategy(name="s", rules=[rule])
    controller = StrategyController(strategy, robot=None)
    match = FakeMatch()

    controller.tick(ctx_for(match))
    events = match.events.of_kind("behavior_change")
    assert len(events) == 1
    assert events[0].data["from"] == "fallback"
    assert events[0].data["to"] == "r"

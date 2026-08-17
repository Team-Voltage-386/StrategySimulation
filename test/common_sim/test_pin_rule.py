"""
The pin rule: how long one robot may prevent another from *moving*
(field_config.PinRule). Covers Match's detection and accounting, the
distinction between denying motion and denying access, and Defend's
obligation to let a mark go before the clock runs out.
"""
from common_sim.control import tactics, world_view
from common_sim.control.behavior import BehaviorContext
from common_sim.field.field_config import FieldConfig, PinRule, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_field(rule=PinRule(max_seconds=3.0, release_seconds=1.0, foul_points=6.0)) -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((240, 40), (299, 40), (299, 160), (240, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,), pin_rule=rule)


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(field=None):
    rules = TableScoringRules({("score_widget", "auto"): 3.0, ("score_widget", "teleop"): 1.0})
    return Match(field or make_field(), rules, MatchConfig(auto_duration=1000, teleop_duration=1000))


def _shove(match, seconds: float, victim_command=(-150.0, 0.0), dt=1.0 / 60.0, victim_x=15.0):
    """A red defender pressing a blue robot into the x=0 wall, with the
    victim driving into that wall to get away. Backed against a field
    element on purpose: that is what separates a pin from a shove (see
    Match.is_pinning). Returns (defender, victim)."""
    victim = match.add_robot(make_characteristics(), Pose2d(victim_x, 100, 0), alliance="blue")
    defender = match.add_robot(make_characteristics(), Pose2d(victim_x + 28, 100, 0), alliance="red")
    for _ in range(int(seconds / dt)):
        defender.drive_field_relative(dt, -150.0, 0.0, 0.0)
        victim.drive_field_relative(dt, victim_command[0], victim_command[1], 0.0)
        match.step(dt)
    return defender, victim


# -- detection --------------------------------------------------------


def test_holding_a_robot_still_for_the_limit_is_a_pin():
    match = make_match()
    _shove(match, seconds=3.5)
    assert match.pin_fouls == {"red": 1}
    assert match.scores.get("blue") == 6.0
    assert len(match.events.of_kind("pin_foul")) == 1


def test_a_shove_shorter_than_the_limit_is_not_a_pin():
    """Defense is allowed to make contact. The rule is about duration,
    so a defender that leans and lets go owes nothing."""
    match = make_match()
    _shove(match, seconds=2.0)
    assert match.pin_fouls == {}
    assert not match.scores


def test_a_pin_that_never_breaks_is_charged_once_per_limit():
    match = make_match()
    _shove(match, seconds=9.5)
    assert match.pin_fouls["red"] == 3


def test_blocking_the_way_is_not_pinning():
    """The distinction the rule turns on, and the reason a defender may
    camp a feeder mouth all match: the victim here can go anywhere it
    likes -- it just cannot go *through*. It drives away instead, and
    a robot that is moving is not being pinned however long the defender
    sits there."""
    match = make_match()
    defender, victim = _shove(match, seconds=10.0, victim_command=(0.0, 150.0), victim_x=150.0)
    assert victim.pose.y > 150.0, "victim should have driven around, not stayed put"
    assert match.pin_fouls == {}


def test_a_robot_that_is_not_trying_to_move_is_not_being_pinned():
    """A robot parked to score with an opponent leaning on it is being
    fouled under other rules, perhaps, but it is not pinned -- nothing
    is being prevented. Without this the sim would charge a pin every
    time a defender pressed a stationary scorer."""
    match = make_match()
    _shove(match, seconds=10.0, victim_command=(0.0, 0.0))
    assert match.pin_fouls == {}


def test_two_robots_shoving_in_open_space_pin_nobody():
    """Both are stopped and both are asking to move, so a rule that only
    asked "commanded but not achieved" would charge each of them with
    pinning the other -- one collision, two alliances fouled. Neither has
    trapped anything: either can back out by stopping."""
    match = make_match()
    a = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    b = match.add_robot(make_characteristics(), Pose2d(178, 100, 0), alliance="red")
    for _ in range(600):
        a.drive_field_relative(1.0 / 60.0, 150.0, 0.0, 0.0)
        b.drive_field_relative(1.0 / 60.0, -150.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
    assert a.speed < 6.0 and b.speed < 6.0, "expected a deadlock, not a push-past"
    assert match.pin_fouls == {}


def test_teammates_cannot_pin_each_other():
    match = make_match()
    victim = match.add_robot(make_characteristics(), Pose2d(15, 100, 0), alliance="blue")
    defender = match.add_robot(make_characteristics(), Pose2d(43, 100, 0), alliance="blue")
    for _ in range(600):
        defender.drive_field_relative(1.0 / 60.0, -150.0, 0.0, 0.0)
        victim.drive_field_relative(1.0 / 60.0, -150.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
    assert match.pin_fouls == {}


def test_a_field_with_no_pin_rule_never_charges_one():
    match = make_match(make_field(rule=None))
    _shove(match, seconds=10.0)
    assert match.pin_fouls == {}
    assert not match.scores


def test_a_rule_with_no_foul_points_still_records_the_violation():
    """Some games answer a pin with a card. The hold is still a fact."""
    match = make_match(make_field(rule=PinRule(max_seconds=3.0)))
    _shove(match, seconds=3.5)
    assert match.pin_fouls == {"red": 1}
    assert not match.scores


def test_releasing_and_re_engaging_restarts_the_clock():
    """Two 2-second holds with a real release between them are two legal
    plays, not one 4-second pin."""
    match = make_match()
    victim = match.add_robot(make_characteristics(), Pose2d(15, 100, 0), alliance="blue")
    defender = match.add_robot(make_characteristics(), Pose2d(43, 100, 0), alliance="red")

    def run(seconds, defender_vx):
        for _ in range(int(seconds * 60)):
            defender.drive_field_relative(1.0 / 60.0, defender_vx, 0.0, 0.0)
            victim.drive_field_relative(1.0 / 60.0, -150.0, 0.0, 0.0)
            match.step(1.0 / 60.0)

    run(2.0, -150.0)          # hold
    assert match.pin_seconds(defender, victim) > 1.5
    run(1.5, 150.0)           # get out of the way for longer than release_seconds
    assert match.pin_seconds(defender, victim) == 0.0
    assert match.pin_fouls == {}


# -- defense respects it ----------------------------------------------


class _FakeIntent:
    def __init__(self, target_region):
        self.target_region = target_region
        self.target_piece = None


class _FakeController:
    """Publishes a fixed target region the way a real controller would;
    the test drives the robot itself."""

    def __init__(self, target_region):
        self.intent = _FakeIntent(target_region)

    def tick(self, ctx):
        pass


def test_defend_releases_a_mark_it_has_been_holding_too_long():
    """A defender that presses until the whistle trades a denial worth a
    few seconds of cycle for a tech foul worth more than the cycle. The
    mark here is backed onto the x=0 wall and driving at the goal, which
    is the one geometry where a defender's normal job becomes a pin."""
    match = make_match()
    mark = match.add_robot(make_characteristics(), Pose2d(15, 100, 0), alliance="blue")
    defender = match.add_robot(make_characteristics(), Pose2d(60, 100, 0), alliance="red")
    mark.controller = _FakeController("goal")

    tactic = tactics.Defend(target="goal", mode="shadow", standoff=0.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(int(12.0 * 60)):
        tactic.tick(ctx)
        mark.drive_field_relative(1.0 / 60.0, 150.0, 0.0, 0.0)
        match.step(1.0 / 60.0)

    assert match.pin_fouls.get("red", 0) == 0
    # And it has not run away: it is still the thing standing between the
    # mark and the goal it wants.
    assert defender.pose.x > mark.pose.x


def test_pin_pressure_is_zero_on_a_field_without_the_rule():
    """Callers read it unconditionally, so it must answer for any game."""
    match = make_match(make_field(rule=None))
    defender, victim = _shove(match, seconds=5.0)
    assert world_view.pin_pressure(match, defender, victim) == 0.0

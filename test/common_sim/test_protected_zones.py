"""
Protected zones: areas where a robot may not be contacted by opponents
(field_config.ProtectedZone). Covers the geometry primitive, Match's
foul accounting, and Defend's obligation to back off a mark that has
reached one.
"""
import math

from common_sim.control import tactics, world_view
from common_sim.control.behavior import BehaviorContext
from common_sim.field.field_config import (
    FieldConfig, ProtectedZone, ScoringRegion, polygons_intersect,
)
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"

# A blue-owned safe box around the goal's mouth, plus the goal itself.
SAFE_BOX = ((60, 20), (140, 20), (140, 180), (60, 180))


def make_field(zones=None) -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((80, 40), (250, 40), (250, 160), (80, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    if zones is None:
        zones = (ProtectedZone(name="safe", vertices=SAFE_BOX, alliance="blue", foul_points=2.0),)
    return FieldConfig(width=300, height=200, scoring_regions=(region,), protected_zones=zones)


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(field=None, **config_overrides):
    field = field or make_field()
    rules = TableScoringRules({("score_widget", "auto"): 3.0, ("score_widget", "teleop"): 1.0})
    config_overrides.setdefault("auto_duration", 1000)
    config_overrides.setdefault("teleop_duration", 1000)
    return Match(field, rules, MatchConfig(**config_overrides))


# -- geometry ---------------------------------------------------------


def test_polygons_intersect_finds_overlap_with_no_vertex_inside_either():
    """The case a vertex-containment test alone misses: a plus sign, where
    each bar crosses the other but every corner is outside it."""
    vertical = ((-1, -5), (1, -5), (1, 5), (-1, 5))
    horizontal = ((-5, -1), (5, -1), (5, 1), (-5, 1))
    assert polygons_intersect(vertical, horizontal)
    assert polygons_intersect(horizontal, vertical)


def test_polygons_intersect_handles_containment_and_separation():
    outer = ((0, 0), (10, 0), (10, 10), (0, 10))
    inner = ((4, 4), (6, 4), (6, 6), (4, 6))
    away = ((20, 20), (22, 20), (22, 22), (20, 22))
    assert polygons_intersect(outer, inner)
    assert polygons_intersect(inner, outer)
    assert not polygons_intersect(outer, away)


# -- who is protected -------------------------------------------------


def test_a_robot_only_partly_inside_the_zone_is_protected():
    """"BUMPERS in the zone" means partially in -- a robot straddling the
    boundary is as protected as one parked in the middle, which matters
    because the whole point of the zone is to cover an approach."""
    match = make_match()
    inside = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    straddling = match.add_robot(make_characteristics(), Pose2d(145, 100, 0), alliance="blue")
    outside = match.add_robot(make_characteristics(), Pose2d(200, 100, 0), alliance="blue")

    assert match.protecting_zone(inside) is not None
    assert match.protecting_zone(straddling) is not None  # x=145, zone ends at 140
    assert match.protecting_zone(outside) is None


def test_a_zone_protects_only_its_own_alliance():
    """An opponent may stand in the zone -- it just gets no protection
    there. Denying the approach is legal; that asymmetry is the point."""
    match = make_match()
    ours = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    theirs = match.add_robot(make_characteristics(), Pose2d(100, 60, 0), alliance="red")

    assert match.protecting_zone(ours) is not None
    assert match.protecting_zone(theirs) is None


def test_an_unowned_zone_protects_whoever_is_standing_in_it():
    zones = (ProtectedZone(name="neutral", vertices=SAFE_BOX),)
    match = make_match(make_field(zones))
    blue = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    red = match.add_robot(make_characteristics(), Pose2d(100, 60, 0), alliance="red")

    assert match.protecting_zone(blue) is not None
    assert match.protecting_zone(red) is not None


# -- fouls ------------------------------------------------------------


def _park_touching(match, alliance_a="blue", alliance_b="red", x=100.0):
    """Two robots overlapping at `x`, so they are unambiguously in
    contact on the very first step regardless of solver push-apart."""
    a = match.add_robot(make_characteristics(), Pose2d(x, 100, 0), alliance=alliance_a)
    b = match.add_robot(make_characteristics(), Pose2d(x + 20, 100, 0), alliance=alliance_b)
    return a, b


def test_contacting_a_protected_robot_scores_a_foul_for_the_other_alliance():
    match = make_match()
    victim, offender = _park_touching(match)
    match.step(1.0 / 60.0)

    assert match.protection_fouls == {"red": 1}
    assert match.scores.get("blue") == 2.0
    assert [e.data["zone"] for e in match.events.of_kind("protection_foul")] == ["safe"]


def test_a_continuous_shove_is_charged_once_per_foul_period_not_per_tick():
    """A referee calls a foul, then keeps watching. Without the period a
    two-second lean would be scored 120 times."""
    zones = (ProtectedZone(name="safe", vertices=SAFE_BOX, alliance="blue",
                           foul_points=2.0, foul_period=1.0),)
    match = make_match(make_field(zones))
    _park_touching(match)

    for _ in range(150):  # 2.5s of unbroken contact
        match.step(1.0 / 60.0)

    # One at t=0, then one per second: 3 calls, not 150.
    assert match.protection_fouls["red"] == 3
    assert match.scores["blue"] == 6.0


def test_no_foul_for_contact_outside_the_zone_or_between_teammates():
    match = make_match()
    # Both well clear of the zone (which ends at x=140).
    _park_touching(match, x=200.0)
    # And a same-alliance pair inside it.
    _park_touching(match, alliance_a="blue", alliance_b="blue", x=90.0)

    for _ in range(120):
        match.step(1.0 / 60.0)

    assert match.protection_fouls == {}
    assert not match.scores


def test_a_zone_with_no_foul_points_still_records_the_violation():
    """Some games answer a violation with a card rather than points. The
    contact is still a fact worth counting."""
    zones = (ProtectedZone(name="safe", vertices=SAFE_BOX, alliance="blue"),)
    match = make_match(make_field(zones))
    _park_touching(match)
    match.step(1.0 / 60.0)

    assert match.protection_fouls == {"red": 1}
    assert not match.scores


# -- defense respects it ----------------------------------------------


class _FakeIntent:
    def __init__(self, target_region):
        self.target_region = target_region
        self.target_piece = None


class _FakeController:
    """Publishes a fixed target region, the way a real controller would,
    and otherwise leaves its robot parked."""

    def __init__(self, target_region):
        self.intent = _FakeIntent(target_region)

    def tick(self, ctx):
        pass


def test_defend_backs_off_a_mark_that_has_reached_its_safe_zone():
    """Blocking the approach is legal; bumping the robot once it arrives
    is not. The defender holds station just outside touching distance."""
    match = make_match()
    defender = match.add_robot(make_characteristics(), Pose2d(200, 100, 0), alliance="red")
    mark = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    mark.controller = _FakeController("goal")

    tactic = tactics.Defend(target="opponent_intent", mode="shadow", standoff=4.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(600):
        tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt

    assert tactic.marked_robot is mark  # it did not wander off
    keepout = world_view.protection_keepout(defender, mark)
    assert defender.pose.distance_to(mark.pose) >= keepout - 1.0
    assert match.protection_fouls.get("red", 0) == 0


def test_defend_releases_a_mark_that_is_about_to_be_protected():
    """A defender leaning on a mark is carried over the line by it, so
    waiting for the mark to actually be inside means always being late.
    It disengages while the mark is still short of the zone."""
    match = make_match()
    defender = match.add_robot(make_characteristics(), Pose2d(250, 100, 0), alliance="red")
    # Footprint reaches x=151, so 11in short of the zone's x=140 edge --
    # not protected yet, but inside _PROTECTION_RELEASE_MARGIN of it.
    mark = match.add_robot(make_characteristics(), Pose2d(165, 100, 0), alliance="blue")
    mark.controller = _FakeController("goal")
    assert match.protecting_zone(mark) is None
    assert 0 < world_view.protection_distance(match, mark) < tactics._PROTECTION_RELEASE_MARGIN

    tactic = tactics.Defend(target="opponent_intent", mode="shadow", standoff=4.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(600):
        tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt

    assert defender.pose.distance_to(mark.pose) >= world_view.protection_keepout(defender, mark) - 1.0
    assert match.protection_fouls.get("red", 0) == 0


def test_defend_still_presses_a_mark_out_in_the_open():
    """The release is a rule, not timidity: with the zone far away the
    same defender closes to its configured standoff."""
    match = make_match()
    defender = match.add_robot(make_characteristics(), Pose2d(280, 100, 0), alliance="red")
    mark = match.add_robot(make_characteristics(), Pose2d(240, 100, 0), alliance="blue")
    mark.controller = _FakeController("goal")

    tactic = tactics.Defend(target="opponent_intent", mode="shadow", standoff=4.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(600):
        tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt

    assert defender.pose.distance_to(mark.pose) < world_view.protection_keepout(defender, mark)


def test_defend_prefers_a_mark_it_can_still_affect():
    """An opponent already inside a safe zone has arrived somewhere we
    may not touch it, so spending our one defender on it denies nothing
    -- even though it is closer and it is the one holding a piece."""
    match = make_match()
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="red")
    safe = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    open_field = match.add_robot(make_characteristics(), Pose2d(260, 100, 0), alliance="blue")
    for robot in (safe, open_field):
        robot.controller = _FakeController("goal")
    safe.held_pieces.append(match.spawn_piece(WIDGET, (100, 100)))

    tactic = tactics.Defend(target="opponent_intent")
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    tactic.tick(ctx)

    assert tactic.marked_robot is open_field


def test_protection_keepout_covers_every_relative_heading():
    """Half-diagonals, not half-widths: a keepout that only holds when
    both robots are square-on is a keepout that gets you fouled."""
    match = make_match()
    a = match.add_robot(make_characteristics(width=28.0, length=34.0), Pose2d(50, 100, 0))
    b = match.add_robot(make_characteristics(width=28.0, length=34.0), Pose2d(250, 100, 0))
    keepout = world_view.protection_keepout(a, b, margin=0.0)
    assert keepout == math.hypot(28.0, 34.0)

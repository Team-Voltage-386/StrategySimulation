"""Tests for the extra robots -- the five that are not the robot code.

Three things are guarded here, and the third is the one worth having.

**The roster** is geometry, and it is checked against `bridge.arena`
rather than against literals. A start pose inside the HUB is a body dyn4j
has to push out of an obstacle, and the symptom is a robot that begins
the match by being flung across the field; a start pose in the wrong
alliance zone is an opponent that spends the match shooting into our
goal. Both are cheap to assert and expensive to notice live.

**The wire** is an encoding, so it is checked exactly: metres out,
inches in, one key per robot per thing. The units are the part that has
already gone wrong once in this project.

**The contract with the strategy layer** is the third, and it is the
reason for the whole module. The extras are only worth having if the
tactics see them -- if `world_view.opponents` returns them, if a piece
one of them is holding stops being collectable, if `Defend` can pick one
as a mark. A field full of robots the strategy layer cannot see is
indistinguishable, from every report this project produces, from a field
with nobody on it.

No pyntcore and no JVM: `OpponentLink` is given a fake `RobotStateLink`,
which is the same trick `test_bridge_match_view.py` plays on the reader,
and for the same reason -- this has to run in CI.
"""
from __future__ import annotations

import math

import pytest

from bridge import arena
from bridge import drive_model as dm
from bridge import match_view as mv
from bridge import opponents as opp
from bridge import world_state as ws
from bridge.robot_state import Pose2d
from common_sim.control import world_view
from common_sim.field.field_config import point_in_polygon

LIMITS = dm.DriveLimits(max_speed_mps=4.0, max_omega_rad_s=8.0)


class FakeState:
    """A `RobotStateLink` that answers from memory and remembers writes."""

    def __init__(self):
        self.published: dict[str, object] = {}
        self.flushes = 0
        self.values: dict[str, object] = {}
        self.poses: list[Pose2d] = []
        self.speeds: list[float] = []
        self.held: list[int] = []

    # -- writes
    def publish_double_array(self, name, value):
        self.published[name] = list(value)

    def publish_boolean(self, name, value):
        self.published[name] = bool(value)

    def publish_integer(self, name, value):
        self.published[name] = int(value)

    def flush(self):
        self.flushes += 1

    # -- reads
    def integer(self, name, default=0):
        return int(self.values.get(name, default))

    def pose2d_array(self, name):
        return list(self.poses) if name == opp.POSES else None

    def double_array(self, name):
        return list(self.speeds) if name == opp.SPEEDS else []

    def integer_array(self, name):
        return list(self.held) if name == opp.HELD else []


class FakeReader:
    """Just enough `WorldStateReader` for a `MapleMatchView`."""

    def __init__(self, *, robot=None, fuel=()):
        self.robot = robot if robot is not None else Pose2d(8.0, 2.0, 0.0)
        self.fuel = tuple(fuel)
        self.link = None

    def read(self) -> ws.WorldState:
        return ws.WorldState(
            robot=self.robot, fuel=self.fuel, held=0,
            match_clock=10.0, phase_clock=20.0,
            hub_active={"blue": True, "red": False}, score={"blue": 0.0, "red": 0.0},
        )

    def measured_chassis_speeds(self):
        return (0.0, 0.0, 0.0)


def a_view():
    robot = mv.MapleRobot(_NoLink(), LIMITS, alliance="blue")
    return mv.MapleMatchView(robot, FakeReader()), robot


class _NoLink:
    def set_axis(self, *a, **k):
        pass

    def set_button(self, *a, **k):
        pass


# -- the roster ---------------------------------------------------------


def test_the_default_roster_is_a_match_minus_us():
    roster = opp.default_roster()
    assert len(roster) == 5
    assert sum(1 for e in roster if e.alliance == "red") == 3
    assert sum(1 for e in roster if e.alliance == "blue") == 2
    assert sum(1 for e in roster if e.role == "defend") == 1


def test_nobody_starts_inside_an_obstacle():
    """A start pose in the HUB is a body dyn4j has to resolve out of a wall.

    Tested against the robot's *footprint* and not its centre: a pose 20
    inches clear of a 47-inch HUB is a 30-inch robot half inside it, and
    the overlap is what gets resolved, not the point.
    """
    obstacles = arena.build_arena().obstacles
    half = mv.BUMPER_IN / 2.0
    for entry in opp.default_roster():
        corners = [
            (entry.x_in + dx, entry.y_in + dy)
            for dx in (-half, half) for dy in (-half, half)
        ]
        for obstacle in obstacles:
            hits = [c for c in corners if point_in_polygon(c, obstacle.vertices)]
            assert not hits, (
                f"{entry.name} starts with {len(hits)} corners inside {obstacle.name}"
            )


def test_everyone_starts_on_their_own_side():
    for entry in opp.default_roster():
        assert arena.in_alliance_zone(entry.x_in, entry.alliance), (
            f"{entry.name} starts outside the {entry.alliance} alliance zone, so its first "
            f"act would be to pass the fuel instead of shooting it"
        )


def test_nobody_starts_on_top_of_anybody():
    roster = opp.default_roster()
    for i, a in enumerate(roster):
        for b in roster[i + 1:]:
            assert math.dist((a.x_in, a.y_in), (b.x_in, b.y_in)) > mv.BUMPER_IN


def test_a_lone_opponent_still_defends():
    """`defenders` counts from the front, so turning the field down to one
    opponent leaves a defender rather than silently leaving a collector."""
    roster = opp.default_roster(opponents=1, partners=0)
    assert [e.role for e in roster] == ["defend"]


def test_an_alliance_is_three_robots():
    with pytest.raises(ValueError, match="at most"):
        opp.default_roster(opponents=4, partners=0)
    with pytest.raises(ValueError, match="at most"):
        opp.default_roster(opponents=0, partners=3)


# -- the wire -----------------------------------------------------------


def test_the_roster_goes_out_in_metres():
    state = FakeState()
    link = opp.OpponentLink(state)
    roster = opp.default_roster(opponents=1, partners=0)
    link.publish_roster(roster)

    flat = state.published[opp.ROSTER]
    assert len(flat) == 3
    assert flat[0] == pytest.approx(roster[0].x_in / arena.M_TO_IN)
    assert flat[1] == pytest.approx(roster[0].y_in / arena.M_TO_IN)
    # A roster published and not flushed is a roster the JVM sees up to a
    # network period later, and the caller is about to block waiting for it.
    assert state.flushes == 1


def test_each_robot_gets_its_own_three_keys():
    state = FakeState()
    link = opp.OpponentLink(state)
    link.command(2, 1.0, -0.5, 0.25, intake=True, release=False)

    assert state.published[opp.speeds_key(2)] == [1.0, -0.5, 0.25]
    assert state.published[opp.intake_key(2)] is True
    assert state.published[opp.release_key(2)] is False
    assert opp.speeds_key(2) != opp.speeds_key(3)


def test_the_heartbeat_counts_up_and_flushes():
    """The far side watches this number change, so it must change.

    A heartbeat that republishes the same value is a heartbeat the JVM
    reads as silence -- which would stop the whole field half a second
    into every match.
    """
    state = FakeState()
    link = opp.OpponentLink(state)
    link.beat()
    first = state.published[opp.TICK]
    link.beat()
    assert state.published[opp.TICK] == first + 1
    assert state.flushes == 2


def test_standing_down_zeroes_everyone_and_says_so():
    state = FakeState()
    link = opp.OpponentLink(state)
    link.command(0, 2.0, 2.0, 1.0, intake=True)
    link.stand_down(2)

    assert state.published[opp.speeds_key(0)] == [0.0, 0.0, 0.0]
    assert state.published[opp.speeds_key(1)] == [0.0, 0.0, 0.0]
    assert state.published[opp.intake_key(0)] is False
    # And the beat goes with it: a stand-down nobody hears is a watchdog
    # timeout instead of a stop, which is half a second of coasting.
    assert opp.TICK in state.published


def test_a_cast_that_never_appears_is_an_error_and_not_a_quiet_zero():
    state = FakeState()
    link = opp.OpponentLink(state)
    with pytest.raises(TimeoutError, match="BridgeRobots"):
        link.wait_for_cast(3, timeout=0.2, poll=0.05)


def test_speeds_come_back_as_triples():
    state = FakeState()
    state.speeds = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    link = opp.OpponentLink(state)
    assert link.speeds() == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


# -- one robot ----------------------------------------------------------


def a_robot(role="cycle", alliance="red", index=0):
    state = FakeState()
    link = opp.OpponentLink(state)
    entry = opp.RosterEntry(
        x_in=500.0, y_in=150.0, heading=0.0, alliance=alliance, role=role, name="red1"
    )
    return state, opp.OpponentRobot(link, index, entry, LIMITS)


def test_driving_converts_to_metres_and_commands_on_publish():
    state, robot = a_robot()
    robot.drive_field_relative(0.05, 39.37007874015748, 0.0, 0.5)
    robot.publish()
    assert state.published[opp.speeds_key(0)] == pytest.approx([1.0, 0.0, 0.5])


def test_driving_is_clamped_to_the_drivetrain_that_exists():
    """Otherwise `commanded_velocity` describes a velocity that cannot
    happen, and every stall check reading `commanded_speed` compares the
    robot against a fiction."""
    state, robot = a_robot()
    robot.drive_field_relative(0.05, 1000.0, 0.0, 99.0)
    robot.publish()
    vx, vy, omega = state.published[opp.speeds_key(0)]
    assert math.hypot(vx, vy) == pytest.approx(LIMITS.max_speed_mps)
    assert omega == pytest.approx(LIMITS.max_omega_rad_s)


def test_commanding_a_drive_still_moves_commanded_speed():
    _, robot = a_robot()
    robot.drive_field_relative(0.05, 50.0, 0.0, 0.0)
    assert robot.commanded_speed > 0.0


def test_depositing_is_a_dump_and_is_not_pretending_otherwise():
    state, robot = a_robot()
    robot.set_deposit_active(True, mv.SHOOT)
    robot.publish()
    assert state.published[opp.release_key(0)] is True
    # And the base class's own bookkeeping still happened, so anything
    # asking the robot what it is doing gets the same answer it would in
    # the pure sim.
    assert robot.deposit_active is True


def test_held_pieces_leave_from_the_front():
    """Same queue discipline as our own robot, and for the same reason --
    `RunManipulator` names `held_pieces[0]` and waits for that object to
    stop being held. See `match_view.reconcile_held`."""
    _, robot = a_robot()
    robot.sync(Pose2d(12.7, 3.8, 0.0), None, 3)
    first, second, third = robot.held_pieces
    robot.sync(Pose2d(12.7, 3.8, 0.0), None, 2)
    assert robot.held_pieces == [second, third]
    assert first.held_by is None


def test_a_synced_robot_knows_where_it_is_and_how_fast():
    _, robot = a_robot()
    robot.sync(Pose2d(12.7, 3.8, 1.0), (1.0, 0.0, 0.5), 0)
    assert robot.pose.x == pytest.approx(12.7 * arena.M_TO_IN)
    assert robot.pose.heading == pytest.approx(1.0)
    # Field-relative already: the JVM did the rotation where the true
    # heading is, so a heading of 1.0 rad must not rotate it again.
    assert robot.speed == pytest.approx(arena.M_TO_IN, rel=1e-6)


# -- the whole field ----------------------------------------------------


def test_the_view_reports_one_robot_until_a_cast_is_attached():
    view, ours = a_view()
    assert view.robots == [ours]
    assert world_view.opponents(view, "blue") == []


def test_the_strategy_layer_sees_the_extras_as_opposition():
    """The point of the whole module. Robots the tactics cannot see are
    indistinguishable, from every report this project produces, from an
    empty field."""
    view, ours = a_view()
    state = FakeState()
    link = opp.OpponentLink(state)
    roster = opp.default_roster()
    cast = opp.OpponentCast(link, roster, LIMITS)
    cast.attach(view, lambda strategy, robot: _StubController())

    assert len(view.robots) == 6
    assert view.robots[0] is ours
    assert len(world_view.opponents(view, "blue")) == 3
    # `partners` is "everyone on this alliance", our own robot included.
    assert len(world_view.partners(view, "blue")) == 3


class _StubController:
    """A controller that is only asked to tick."""

    def __init__(self):
        self.ticks = 0
        self.intent = None

    def tick(self, context):
        self.ticks += 1
        context.robot.drive_field_relative(context.dt, 10.0, 0.0, 0.0)


def test_a_tick_syncs_everyone_then_commands_everyone():
    view, ours = a_view()
    state = FakeState()
    roster = opp.default_roster(opponents=2, partners=0)
    state.poses = [Pose2d(12.0, 2.0, 0.0), Pose2d(13.0, 6.0, 0.0)]
    state.speeds = [0.0] * 6
    state.held = [4, 0]

    cast = opp.OpponentCast(opp.OpponentLink(state), roster, LIMITS)
    cast.attach(view, lambda strategy, robot: _StubController())

    from common_sim.control.behavior import BehaviorContext

    cast.tick(BehaviorContext(robot=ours, dt=0.05, elapsed=1.0, match=view))

    assert [c.ticks for c in cast.controllers] == [1, 1]
    assert cast.robots[0].pose.x == pytest.approx(12.0 * arena.M_TO_IN)
    assert len(cast.robots[0].held_pieces) == 4
    assert state.published[opp.speeds_key(0)][0] > 0.0
    assert state.published[opp.speeds_key(1)][0] > 0.0
    # One heartbeat per tick, not one per robot.
    assert state.published[opp.TICK] == 1


def test_every_robot_decides_against_the_same_instant():
    """`_with_robot` swaps the robot and keeps everything else, so five
    robots cannot disagree about what time it is within one tick."""
    view, ours = a_view()
    state = FakeState()
    roster = opp.default_roster(opponents=2, partners=0)
    state.poses = [Pose2d(12.0, 2.0, 0.0), Pose2d(13.0, 6.0, 0.0)]

    seen: list[tuple] = []

    class Recorder:
        intent = None

        def tick(self, context):
            seen.append((context.elapsed, context.dt, context.match, context.robot))

    cast = opp.OpponentCast(opp.OpponentLink(state), roster, LIMITS)
    cast.attach(view, lambda strategy, robot: Recorder())

    from common_sim.control.behavior import BehaviorContext

    cast.tick(BehaviorContext(robot=ours, dt=0.05, elapsed=7.5, match=view))

    assert [s[0] for s in seen] == [7.5, 7.5]
    assert [s[2] for s in seen] == [view, view]
    assert [s[3] for s in seen] == cast.robots
    # And our own robot was not left pointed at somebody else's body.
    assert ours not in [s[3] for s in seen]


def test_a_short_readback_does_not_crash_a_tick():
    """The pose array arrives an AdvantageKit cycle after the count does,
    so the first few ticks of a match legitimately see fewer poses than
    robots. That must be a robot that has not moved yet, not a crash."""
    view, ours = a_view()
    state = FakeState()
    roster = opp.default_roster(opponents=2, partners=0)
    state.poses = []

    cast = opp.OpponentCast(opp.OpponentLink(state), roster, LIMITS)
    cast.attach(view, lambda strategy, robot: _StubController())

    from common_sim.control.behavior import BehaviorContext

    cast.tick(BehaviorContext(robot=ours, dt=0.05, elapsed=0.0, match=view))
    assert [c.ticks for c in cast.controllers] == [1, 1]


# -- the strategies -----------------------------------------------------


def test_a_defender_defends_and_a_cycler_cycles():
    defend = opp.default_strategy_for(
        opp.RosterEntry(0.0, 0.0, 0.0, "red", role="defend", name="red1")
    )
    cycle = opp.default_strategy_for(
        opp.RosterEntry(0.0, 0.0, 0.0, "red", role="cycle", name="red2")
    )
    assert [type(r.tactic).__name__ for r in defend.rules] == ["Defend"]
    assert "Score" in [type(r.tactic).__name__ for r in cycle.rules]


def test_a_defenders_trigger_is_always_on():
    """A `Defend` rule that can go false drops its robot to `Idle` in the
    middle of a match, which looks like a defender that gave up."""
    from common_sim.control.triggers import Always

    defend = opp.default_strategy_for(
        opp.RosterEntry(0.0, 0.0, 0.0, "red", role="defend", name="red1")
    )
    assert isinstance(defend.rules[0].trigger, Always)

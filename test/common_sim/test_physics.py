import math

import pymunk

from common_sim.geometry import Pose2d
from common_sim.physics.engine import SimEngine
from common_sim.physics.swerve import SwerveChassis, SwerveLimits


def make_chassis(engine, accel: float = 200.0, **overrides):
    limits = SwerveLimits(
        max_speed=120.0,
        max_accel=accel,
        max_angular_speed=6.0,
        max_angular_accel=20.0,
    )
    defaults = dict(width=28.0, length=28.0, start_pose=Pose2d(0, 0, 0))
    defaults.update(overrides)
    return SwerveChassis(engine.space, limits, **defaults)


def test_engine_step_advances_elapsed_time():
    engine = SimEngine(substep=1.0 / 240.0)
    engine.step(1.0)
    assert math.isclose(engine.elapsed, 1.0, abs_tol=1e-6)


def test_engine_step_is_frame_rate_independent():
    """Same total dt, different call granularity, should land at ~the same
    physics state -- this is the whole point of fixed substeps."""
    limits = SwerveLimits(max_speed=100, max_accel=1e9, max_angular_speed=10, max_angular_accel=1e9)

    engine_a = SimEngine()
    chassis_a = SwerveChassis(engine_a.space, limits, width=28, length=28)
    for _ in range(60):
        chassis_a.drive_field_relative(1.0 / 60.0, 50, 0, 0)
        engine_a.step(1.0 / 60.0)

    engine_b = SimEngine()
    chassis_b = SwerveChassis(engine_b.space, limits, width=28, length=28)
    for _ in range(600):
        chassis_b.drive_field_relative(1.0 / 600.0, 50, 0, 0)
        engine_b.step(1.0 / 600.0)

    assert math.isclose(chassis_a.pose.x, chassis_b.pose.x, rel_tol=0.02)


def test_chassis_accelerates_toward_commanded_velocity():
    engine = SimEngine()
    chassis = make_chassis(engine)
    # The command latches and is ramped in by the physics substeps, so it
    # takes a step to show up in the body -- the drivetrain is a force the
    # solver arbitrates, not a velocity written straight into the body.
    chassis.drive_field_relative(1.0 / 60.0, 120.0, 0.0, 0.0)
    engine.step(1.0 / 60.0)
    vx, vy = chassis.body.velocity
    assert vx > 0
    assert math.isclose(vy, 0.0, abs_tol=1e-6)


def test_chassis_respects_max_speed():
    engine = SimEngine()
    chassis = make_chassis(engine)
    for _ in range(600):
        chassis.drive_field_relative(1.0 / 60.0, 500.0, 0.0, 0.0)
        engine.step(1.0 / 60.0)
    speed = pymunk.Vec2d(*chassis.body.velocity).length
    assert speed <= chassis.limits.max_speed + 1e-6


def test_chassis_moves_in_commanded_direction_over_time():
    engine = SimEngine()
    chassis = make_chassis(engine)
    for _ in range(120):
        chassis.drive_field_relative(1.0 / 60.0, 0.0, 100.0, 0.0)
        engine.step(1.0 / 60.0)
    assert chassis.pose.y > 10
    assert math.isclose(chassis.pose.x, 0.0, abs_tol=1e-3)


def test_chassis_rotates_toward_commanded_omega():
    engine = SimEngine()
    chassis = make_chassis(engine)
    for _ in range(120):
        chassis.drive_field_relative(1.0 / 60.0, 0.0, 0.0, 3.0)
        engine.step(1.0 / 60.0)
    assert chassis.body.angular_velocity > 0
    assert chassis.pose.heading != 0.0


def test_drive_robot_relative_uses_current_heading():
    engine = SimEngine()
    chassis = make_chassis(engine, start_pose=Pose2d(0, 0, math.pi / 2))
    chassis.drive_robot_relative(1.0 / 60.0, 100.0, 0.0, 0.0)
    engine.step(1.0 / 60.0)
    vx, vy = chassis.body.velocity
    # robot-relative "forward" (+x) at heading=90deg should move in field +y
    assert vy > 0
    assert math.isclose(vx, 0.0, abs_tol=1e-6)


def _pushing_match(engine, pusher_accel: float, victim_accel: float, seconds: float = 5.0):
    """Bumper-to-bumper from rest: `pusher` floors it in +x, `victim`
    commands zero (which is not passivity -- commanding zero is braking,
    and braking is the victim spending its whole traction budget on
    holding position). Returns (pusher, victim)."""
    pusher = make_chassis(engine, accel=pusher_accel, start_pose=Pose2d(100, 100, 0))
    victim = make_chassis(engine, accel=victim_accel, start_pose=Pose2d(128, 100, 0))
    for _ in range(int(seconds * 60)):
        pusher.drive_field_relative(1.0 / 60.0, 120.0, 0.0, 0.0)
        victim.drive_field_relative(1.0 / 60.0, 0.0, 0.0, 0.0)
        engine.step(1.0 / 60.0)
    return pusher, victim


def test_equally_powered_robots_deadlock_in_a_pushing_match():
    """Both spend `mass * max_accel` of traction on opposite directions,
    so the net force on the pair is zero and neither wins. This is the
    property the pin rule is built on: a defender *can* stop a robot it
    is square against, and cannot take it anywhere."""
    engine = SimEngine()
    pusher, victim = _pushing_match(engine, pusher_accel=200.0, victim_accel=200.0)
    assert victim.pose.x - 128.0 < 10.0
    # The pusher is stopped too -- it is not driving past, it is stuck on
    # the victim at a small fraction of the speed it is asking for.
    assert pusher.body.velocity[0] < 10.0


def test_a_weaker_drivetrain_loses_the_pushing_match():
    """The deadlock above is traction, not geometry: give the victim a
    fifth of the pusher's traction and it gets driven the length of the
    field while the pusher barely notices."""
    engine = SimEngine()
    pusher, victim = _pushing_match(engine, pusher_accel=200.0, victim_accel=40.0)
    assert victim.pose.x - 128.0 > 200.0
    assert pusher.body.velocity[0] > 100.0


def test_module_states_returns_four_modules():
    engine = SimEngine()
    chassis = make_chassis(engine)
    chassis.drive_field_relative(1.0 / 60.0, 50.0, 0.0, 1.0)
    states = chassis.module_states()
    assert len(states) == 4
    for s in states:
        assert s.speed >= 0.0

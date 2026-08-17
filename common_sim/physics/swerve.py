"""
Generic swerve-drive chassis model. Owns a pymunk.Body plus a bumper
shape and converts field-relative (vx, vy, omega) commands into chassis
motion under acceleration limits. Game-agnostic: no notion of
mechanisms, held pieces, or scoring -- see robot/robot.py for that
layer.

Modules are not independently simulated. The whole chassis is one rigid
body driven directly by commanded velocity (acceleration-limited), which
is the right fidelity for comparing robot *concepts* -- this sim is not
trying to validate a swerve module's own control loop. `module_states()`
derives per-corner angle/speed purely for visualization, from chassis
velocity + rotation about each module's offset.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import pymunk

from common_sim.geometry import Pose2d, Vec2d, wrap_angle


@dataclass(frozen=True)
class SwerveLimits:
    max_speed: float          # in/s
    max_accel: float          # in/s^2
    max_angular_speed: float  # rad/s
    max_angular_accel: float  # rad/s^2


@dataclass(frozen=True)
class ModuleState:
    offset: Vec2d   # module position relative to chassis center, robot-relative
    angle: float    # field-relative heading the module is pointed, radians
    speed: float    # unsigned wheel speed, in/s


class SwerveChassis:
    def __init__(
        self,
        space: pymunk.Space,
        limits: SwerveLimits,
        *,
        width: float,
        length: float,
        mass: float = 15.0,
        start_pose: Pose2d = Pose2d(0.0, 0.0, 0.0),
        collision_type: int = 0,
        shape_filter: pymunk.ShapeFilter | None = None,
    ):
        self.limits = limits
        self.width = width
        self.length = length

        moment = pymunk.moment_for_box(mass, (length, width))
        self.body = pymunk.Body(mass, moment)
        self.body.position = (start_pose.x, start_pose.y)
        self.body.angle = start_pose.heading

        half_l, half_w = length / 2.0, width / 2.0
        self.bumper_shape = pymunk.Poly(self.body, [
            (-half_l, -half_w), (half_l, -half_w),
            (half_l, half_w), (-half_l, half_w),
        ])
        self.bumper_shape.elasticity = 0.2
        self.bumper_shape.friction = 0.6
        self.bumper_shape.collision_type = collision_type
        if shape_filter is not None:
            self.bumper_shape.filter = shape_filter

        space.add(self.body, self.bumper_shape)

        # The drivetrain is a *traction-limited* velocity source, and the
        # limit is enforced per physics substep rather than per control
        # tick. Both spellings accelerate identically in free space --
        # `max_accel * dt` summed over a tick's substeps is the same
        # `max_accel * tick` -- but they differ completely in contact.
        # Slewing once per control tick lets a robot re-assert its
        # commanded velocity only every 20ms, and does nothing at all in
        # between, so contact impulses have 4-5 substeps of uncontested
        # authority over a robot that is braced and trying to hold still.
        # Enforcing it every substep means a robot pushes and resists
        # with at most `mass * max_accel` of force at every instant,
        # which is what makes a pushing match resolve on traction rather
        # than on control-loop phase.
        self._target_velocity = Vec2d(0.0, 0.0)
        self._target_omega = 0.0
        self.body.velocity_func = self._integrate_velocity

        # Nominal module positions inset slightly from the bumper corners.
        inset = 3.0
        self._module_offsets = [
            Vec2d(half_l - inset, half_w - inset),
            Vec2d(half_l - inset, -(half_w - inset)),
            Vec2d(-(half_l - inset), -(half_w - inset)),
            Vec2d(-(half_l - inset), half_w - inset),
        ]

    @property
    def pose(self) -> Pose2d:
        p = self.body.position
        return Pose2d(p.x, p.y, wrap_angle(self.body.angle))

    def drive_field_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        """Command field-relative chassis velocity (in/s, in/s, rad/s).
        Acceleration-limited toward the target, so callers can set a raw
        joystick-derived target every frame without ramping it
        themselves. The command latches: it stays in force until
        superseded, and is applied by `_integrate_velocity` every physics
        substep. `dt` is accepted for signature compatibility and is not
        used -- the ramp is paced by the physics clock, not the caller's."""
        target = Vec2d(vx, vy)
        if target.length > self.limits.max_speed:
            target = target.scale_to_length(self.limits.max_speed)
        self._target_velocity = target
        self._target_omega = _clamp(omega, self.limits.max_angular_speed)

    def _integrate_velocity(self, body, gravity, damping, dt) -> None:
        """pymunk velocity_func: ramp toward the latched command under
        the acceleration limit, then hand off to the default integrator
        so externally applied forces and impulses still land. Running
        before the contact solver each substep is what gives the
        drivetrain finite authority -- the ramp caps what it can add,
        and the solver then decides how much of that survives whatever
        the robot is pressed against."""
        current = Vec2d(*body.velocity)
        body.velocity = _slew_vector(current, self._target_velocity, self.limits.max_accel, dt)
        body.angular_velocity = _slew_scalar(
            body.angular_velocity, self._target_omega, self.limits.max_angular_accel, dt
        )
        pymunk.Body.update_velocity(body, gravity, damping, dt)

    def drive_robot_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        """Command chassis velocity in the robot's own frame (e.g. from a
        driver joystick where +x is always "forward" regardless of
        heading)."""
        field_v = Vec2d(vx, vy).rotated(self.body.angle)
        self.drive_field_relative(dt, field_v.x, field_v.y, omega)

    def module_states(self) -> list[ModuleState]:
        vx, vy = self.body.velocity
        omega = self.body.angular_velocity
        states = []
        for offset in self._module_offsets:
            world_offset = offset.rotated(self.body.angle)
            mod_vx = vx - world_offset.y * omega
            mod_vy = vy + world_offset.x * omega
            speed = math.hypot(mod_vx, mod_vy)
            angle = math.atan2(mod_vy, mod_vx) if speed > 1e-3 else self.body.angle
            states.append(ModuleState(offset=offset, angle=angle, speed=speed))
        return states


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _slew_vector(current: Vec2d, target: Vec2d, max_accel: float, dt: float) -> Vec2d:
    delta = target - current
    max_delta = max_accel * dt
    if delta.length <= max_delta:
        return target
    return current + delta.scale_to_length(max_delta)


def _slew_scalar(current: float, target: float, max_accel: float, dt: float) -> float:
    delta = target - current
    max_delta = max_accel * dt
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)

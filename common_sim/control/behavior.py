"""
Small composable behavior-tree-style scripting framework. One node set,
reused three ways: autonomous routines for the robot under test,
scripted alliance-partner robots, and scripted opponent robots -- there
is no separate "AI opponent" system, just a Behavior attached to a
different Robot.

A Behavior does not step the simulation itself -- it's ticked once per
frame by whatever owns the loop (an app, a test, or the Monte Carlo
runner), the same way a human driver's InputSource is polled once per
frame. A typical loop looks like:

    ctx = BehaviorContext(robot, dt)
    for _ in range(n_ticks):
        ctx.dt = dt
        routine.tick(ctx)
        match.step(dt)
        ctx.elapsed += dt

Nodes call the same Robot API (drive_field_relative, set_intake_active,
set_deposit_active) a human InputSource-driven loop would -- a Behavior
is just a scripted driver.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from common_sim.geometry import Pose2d, wrap_angle
from common_sim.robot.robot import Robot


class Status(Enum):
    RUNNING = auto()
    SUCCESS = auto()
    FAILURE = auto()


@dataclass
class BehaviorContext:
    robot: Robot
    dt: float
    elapsed: float = 0.0
    match: object = None  # duck-typed (not `Match`) so this module never imports match.py


class Behavior(ABC):
    @abstractmethod
    def tick(self, ctx: BehaviorContext) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        """Restart internal state (timers, chosen branch, ...). Called by
        a parent node before re-running this node from scratch (e.g.
        Repeat looping, or Sequence being reset for a fresh run)."""


# -- leaves --------------------------------------------------------------


class Wait(Behavior):
    def __init__(self, duration: float):
        self.duration = duration
        self._elapsed = 0.0

    def tick(self, ctx: BehaviorContext) -> Status:
        self._elapsed += ctx.dt
        return Status.SUCCESS if self._elapsed >= self.duration else Status.RUNNING

    def reset(self) -> None:
        self._elapsed = 0.0


class RunIntake(Behavior):
    """Commands intake active until the robot's held-piece count
    increases, or `timeout` seconds elapse (FAILURE) if given."""

    def __init__(self, timeout: Optional[float] = None):
        self.timeout = timeout
        self._elapsed = 0.0
        self._start_count: Optional[int] = None

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        if self._start_count is None:
            self._start_count = len(robot.held_pieces)
        robot.set_intake_active(True)
        self._elapsed += ctx.dt

        if len(robot.held_pieces) > self._start_count:
            robot.set_intake_active(False)
            return Status.SUCCESS
        if self.timeout is not None and self._elapsed >= self.timeout:
            robot.set_intake_active(False)
            return Status.FAILURE
        return Status.RUNNING

    def reset(self) -> None:
        self._elapsed = 0.0
        self._start_count = None


class RunManipulator(Behavior):
    """Commands a deposit targeting `action` until the robot's
    currently-held lead piece is released, or `timeout` seconds elapse.
    FAILURE immediately if the robot is holding nothing when first
    ticked -- there is nothing to deposit."""

    def __init__(self, action: str, timeout: Optional[float] = None):
        self.action = action
        self.timeout = timeout
        self._elapsed = 0.0
        self._started = False
        self._target_piece = None

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        if not self._started:
            self._started = True
            if not robot.held_pieces:
                return Status.FAILURE
            self._target_piece = robot.held_pieces[0]

        robot.set_deposit_active(True, action=self.action)
        self._elapsed += ctx.dt

        if self._target_piece not in robot.held_pieces:
            robot.set_deposit_active(False)
            return Status.SUCCESS
        if self.timeout is not None and self._elapsed >= self.timeout:
            robot.set_deposit_active(False)
            return Status.FAILURE
        return Status.RUNNING

    def reset(self) -> None:
        self._elapsed = 0.0
        self._started = False
        self._target_piece = None


class DriveToPose(Behavior):
    """Proportional drive toward a field-relative target pose. Not a
    motion-profiled trajectory follower -- this sim compares design
    concepts, not tunes real drive controllers, so "drives there
    smoothly enough" is the right fidelity."""

    def __init__(
        self,
        target: Pose2d,
        position_tolerance: float = 2.0,
        heading_tolerance: float = 0.05,
        speed_gain: float = 3.0,
        heading_gain: float = 4.0,
    ):
        self.target = target
        self.position_tolerance = position_tolerance
        self.heading_tolerance = heading_tolerance
        self.speed_gain = speed_gain
        self.heading_gain = heading_gain

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        pose = robot.pose
        delta = self.target.translation - pose.translation
        distance = delta.length
        heading_error = wrap_angle(self.target.heading - pose.heading)

        if distance <= self.position_tolerance and abs(heading_error) <= self.heading_tolerance:
            robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
            return Status.SUCCESS

        vx, vy = 0.0, 0.0
        if distance > 1e-6:
            direction = delta / distance
            speed = min(robot.characteristics.max_speed, distance * self.speed_gain)
            vx, vy = direction.x * speed, direction.y * speed
        max_omega = robot.characteristics.max_angular_speed
        omega = max(-max_omega, min(max_omega, heading_error * self.heading_gain))

        robot.drive_field_relative(ctx.dt, vx, vy, omega)
        return Status.RUNNING

    def reset(self) -> None:
        pass  # stateless


# -- composites ------------------------------------------------------------


class Sequence(Behavior):
    """Runs children in order; advances on SUCCESS, stops and reports
    FAILURE if any child fails."""

    def __init__(self, children: list[Behavior]):
        self.children = children
        self._index = 0

    def tick(self, ctx: BehaviorContext) -> Status:
        while self._index < len(self.children):
            status = self.children[self._index].tick(ctx)
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.FAILURE:
                return Status.FAILURE
            self._index += 1
        return Status.SUCCESS

    def reset(self) -> None:
        self._index = 0
        for child in self.children:
            child.reset()


class Parallel(Behavior):
    """Ticks every not-yet-finished child every tick. Succeeds once
    `succeed_on` children have succeeded (default: all). Any child
    failure fails the whole node immediately."""

    def __init__(self, children: list[Behavior], succeed_on: Optional[int] = None):
        self.children = children
        self.succeed_on = succeed_on if succeed_on is not None else len(children)
        self._done = [False] * len(children)

    def tick(self, ctx: BehaviorContext) -> Status:
        succeeded = 0
        for i, child in enumerate(self.children):
            if self._done[i]:
                succeeded += 1
                continue
            status = child.tick(ctx)
            if status == Status.SUCCESS:
                self._done[i] = True
                succeeded += 1
            elif status == Status.FAILURE:
                return Status.FAILURE
        return Status.SUCCESS if succeeded >= self.succeed_on else Status.RUNNING

    def reset(self) -> None:
        self._done = [False] * len(self.children)
        for child in self.children:
            child.reset()


class Branch(Behavior):
    """Evaluates `condition(ctx)` once, the first time this node ticks
    after being (re)constructed or reset, and commits to `if_true` or
    `if_false` for the rest of its run -- a condition based on transient
    state (e.g. "do I currently have a piece") does not get re-checked
    mid-branch. `if_false=None` means "do nothing and succeed" if the
    condition is false."""

    def __init__(self, condition: Callable[[BehaviorContext], bool], if_true: Behavior, if_false: Optional[Behavior] = None):
        self.condition = condition
        self.if_true = if_true
        self.if_false = if_false
        self._chosen: Optional[Behavior] = None
        self._evaluated = False

    def tick(self, ctx: BehaviorContext) -> Status:
        if not self._evaluated:
            self._evaluated = True
            self._chosen = self.if_true if self.condition(ctx) else self.if_false
        if self._chosen is None:
            return Status.SUCCESS
        return self._chosen.tick(ctx)

    def reset(self) -> None:
        if self._chosen is not None:
            self._chosen.reset()
        self._chosen = None
        self._evaluated = False


class Repeat(Behavior):
    """Runs `child` to completion (any terminal status), resets it, and
    runs it again -- forever. Never itself returns a terminal status, so
    it's meant as the outermost node of a routine meant to run for the
    rest of the match (a scripted opponent's endless cycle), not as a
    step inside a Sequence expecting completion."""

    def __init__(self, child: Behavior):
        self.child = child

    def tick(self, ctx: BehaviorContext) -> Status:
        status = self.child.tick(ctx)
        if status != Status.RUNNING:
            self.child.reset()
        return Status.RUNNING

    def reset(self) -> None:
        self.child.reset()

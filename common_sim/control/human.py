"""
Wraps a human's polled input as a Robot controller, so Match.step's
uniform `if robot.controller is not None: robot.controller.tick(ctx)`
drives a human-piloted robot exactly the same way it drives an AI one --
no special case in the app's tick loop for "this one has no controller,
drive it here instead."

Deliberately does NOT poll an InputSource itself. OperatorCommand's
`pause_toggle`/`cycle_level` are edge-triggered -- True for exactly one
poll() per physical press -- and those are app-level concerns (pause the
whole match, cycle a GUI selector), not robot-level ones. A second poller
reading the same device the app already polls once per frame would
randomly race it and drop presses. Instead the app polls once per frame
and hands this the already-polled (DriveCommand, OperatorCommand) pair
through `command_provider`.
"""
from __future__ import annotations

import math
from typing import Callable

from common_sim.control import world_view
from common_sim.control.behavior import BehaviorContext
from common_sim.control.input_sources import DriveCommand, OperatorCommand
from common_sim.control.strategy import Intent
from common_sim.field.field_config import polygon_centroid


class HumanController:
    def __init__(
        self,
        command_provider: Callable[[], tuple[DriveCommand, OperatorCommand]],
        deposit_action_provider: Callable[[], str | None] = lambda: None,
    ):
        """`command_provider` returns the current frame's already-polled
        (DriveCommand, OperatorCommand) -- see module docstring for why
        this doesn't poll an InputSource directly. `deposit_action_provider`
        picks which scoring action (e.g. a REEF level) a commanded deposit
        targets; it's a callable rather than a fixed value because it
        typically depends on live GUI state (a selected level, or an
        auto-switch when the robot is sitting in a single-action zone)."""
        self.command_provider = command_provider
        self.deposit_action_provider = deposit_action_provider
        # Robot.intent unconditionally reads self.controller.intent once a
        # controller is assigned (see robot.py) -- without this attribute
        # a human-driven robot would raise AttributeError the first time
        # anything (an opposing Defend tactic, the field canvas) reads it
        # before the first tick(). tick() replaces this with a synthesized
        # guess every frame after that -- see _synthesize_intent -- so
        # None here only ever covers the gap before the first tick.
        self.intent: Intent | None = None

    def tick(self, ctx: BehaviorContext) -> None:
        drive, operator = self.command_provider()
        robot = ctx.robot
        c = robot.characteristics
        robot.drive_field_relative(ctx.dt, drive.vx * c.max_speed, drive.vy * c.max_speed, drive.omega * c.max_angular_speed)
        robot.set_intake_active(operator.intake_active)
        robot.set_deposit_active(operator.deposit_active, action=self.deposit_action_provider())
        self.intent = _synthesize_intent(ctx)


def _synthesize_intent(ctx: BehaviorContext) -> Intent:
    """Best-effort Intent for a human driver, who has no internal tactic
    for `StrategyController._update_intent` to read one off of. Built
    from what the robot is physically doing instead: carrying a piece
    declares the nearest reachable scoring region for it (mirrors
    Score); empty-handed declares whichever of the nearest reachable
    piece or station is closer, ties going to the station (mirrors
    Collect._pick_target's own tie-break -- a station never runs dry
    mid-cycle the way a contested piece can be beaten to).

    This is the same class of guess world_view.likely_scoring_region and
    likely_denial_target already fall back to for *any* robot with no
    declared intent, so a human guessing about itself this way is no
    better than what an opponent would already infer. The one thing
    those fallbacks cannot see at all is target_piece: there is no
    piece-contention fallback (see Collect._piece_contenders), so
    without a published target_piece an AI teammate or opponent has no
    way to know a human has committed to a loose piece until the human
    is already holding it -- always finding out one beat too late to
    react.

    Deliberately not a plan: no patience clocks, no commit hysteresis,
    no obstacle-routed ETA -- it re-guesses fresh every tick from
    whatever is nearest right now, the same crude way a defender
    guessing at an undeclared AI intent already does. Good enough for
    another robot to react a beat sooner is the whole bar; the human
    behind the wheel already has a much better plan than this one."""
    match, robot = ctx.match, ctx.robot
    if match is None:
        return Intent(tactic_name="Human")

    if robot.held_pieces:
        region = world_view.likely_scoring_region(match, robot)
        return Intent(
            tactic_name="Human",
            target_region=region.name if region is not None else None,
            target_piece=robot.held_pieces[0],
        )

    origin = (robot.pose.x, robot.pose.y)
    station, station_dist = _nearest(
        world_view.station_options(match, robot), origin, lambda s: polygon_centroid(s.vertices)
    )
    piece, piece_dist = _nearest(
        world_view.collectable_pieces(match, robot=robot), origin, lambda p: (p.position.x, p.position.y)
    )

    if station is not None and station_dist <= piece_dist:
        return Intent(tactic_name="Human", target_region=station.name)
    if piece is not None:
        return Intent(tactic_name="Human", target_piece=piece)
    return Intent(tactic_name="Human")


def _nearest(items, origin: tuple[float, float], locate: Callable[[object], tuple[float, float]]):
    """The item in `items` closest to `origin` by straight-line distance
    from whatever point `locate` reads off it, and that distance -- or
    (None, inf) for an empty sequence, so a caller can compare two of
    these calls without a separate emptiness check."""
    best, best_dist = None, math.inf
    for item in items:
        px, py = locate(item)
        dist = math.hypot(px - origin[0], py - origin[1])
        if dist < best_dist:
            best, best_dist = item, dist
    return best, best_dist

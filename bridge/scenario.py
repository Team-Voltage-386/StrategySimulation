"""A seeded virtual operator: what the robot gets asked to do for 150 seconds.

Not random button mashing. Random input finds shallow bugs, because it almost
never assembles a valid prefix -- it never gets the robot loaded, aligned and
committed, which is where the interesting failures live. What this generates is
*purposeful* input with seeded variation: recognisable driver moves, in a
plausible order, at plausible magnitudes.

Then it makes the operator slightly bad on purpose. A driver who releases a
trigger early, presses things in an unusual order, or re-requests something
already running is a better fuzzer than a perfect one, because those are exactly
the sequences no test author thought to script. The bar is "a real human could
plausibly do this", not "an expert driver would do this".

The one thing this operator is *not* allowed to do is hold the stick into a
wall. A real driver backs off; a fuzzer that does not spends the match wedged,
trips `frozen-robot` on every run, and turns the morning report into noise
nobody reads. Wall recovery is handled by the runner, which can see the pose --
see `ScenarioRunner`.

Reproducibility, stated honestly: a seed reproduces the *script*, not the run.
The recovery is closed-loop and the physics is not deterministic across
processes, so match 4711 will do the same things in the same order but not land
on the same coordinates. The WPILOG is the authoritative record of what
actually happened. That is the same division the feasibility brief settled on:
reproducibility comes from the recorded input trace, not from deterministic
stepping.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from bridge import operator as op

# Field extents in metres, blue origin, from the arena the robot project builds
# (BlankSimArena) -- FIELD is only used for staying away from walls.
FIELD_LENGTH = 17.548
FIELD_WIDTH = 8.052
FIELD_CENTRE = (FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0)

# How close to a wall counts as "about to get wedged". A 30" bumper is 0.762 m
# across, so its half-diagonal is ~0.54 m; the rest is reaction distance.
WALL_MARGIN = 1.2


@dataclass(frozen=True)
class Action:
    """One operator intent, held for `seconds`.

    An Action is a *complete* description of the operator's hands, not a delta.
    Anything it does not mention is released. That is what makes the boundary
    between two Actions produce real button edges -- and what makes two
    consecutive Actions holding the same button produce *no* edge, which is the
    "re-requested while already running" case worth reaching.
    """

    label: str
    seconds: float
    axes: dict[int, float] = field(default_factory=dict)
    buttons: frozenset[int] = frozenset()
    manip_buttons: frozenset[int] = frozenset()

    def truncated(self, fraction: float) -> "Action":
        """The same intent, abandoned partway through."""
        return Action(
            label=f"{self.label}(cut)",
            seconds=max(0.15, self.seconds * fraction),
            axes=self.axes,
            buttons=self.buttons,
            manip_buttons=self.manip_buttons,
        )

    @property
    def commands_drive(self) -> bool:
        """Whether this action asks the robot to translate.

        Used to decide whether "the robot did not move" means anything: it says
        nothing at all during a `spin-up` or an `idle`.
        """
        return any(
            abs(self.axes.get(axis, 0.0)) > 0.1
            for axis in (op.AXIS_LEFT_X, op.AXIS_LEFT_Y)
        )


def _drive_axes(direction: tuple[float, float], magnitude: float) -> dict[int, float]:
    """Stick values for a field-relative direction.

    The drive binding is `xSupplier = -getLeftY()`, `ySupplier = -getLeftX()`,
    and `joystickDrive` is field-relative (confirmed by observation: pushing
    leftX positive moves the robot toward -y regardless of heading).
    """
    dx, dy = direction
    norm = math.hypot(dx, dy) or 1.0
    return {
        op.AXIS_LEFT_Y: -(dx / norm) * magnitude,
        op.AXIS_LEFT_X: -(dy / norm) * magnitude,
    }


class ScenarioGenerator:
    """Produces a seeded stream of Actions for one match."""

    def __init__(self, seed: int):
        self.seed = seed
        self.rng = random.Random(seed)

    # -- individual moves --------------------------------------------------

    def drive(self) -> Action:
        angle = self.rng.uniform(0, 2 * math.pi)
        return Action(
            label="drive",
            seconds=self.rng.uniform(0.6, 1.6),
            axes=_drive_axes((math.cos(angle), math.sin(angle)), self.rng.uniform(0.3, 0.9)),
        )

    def turn(self) -> Action:
        return Action(
            label="turn",
            seconds=self.rng.uniform(0.4, 1.2),
            axes={op.AXIS_RIGHT_X: self.rng.choice([-1, 1]) * self.rng.uniform(0.3, 0.8)},
        )

    def drive_and_turn(self) -> Action:
        angle = self.rng.uniform(0, 2 * math.pi)
        axes = _drive_axes((math.cos(angle), math.sin(angle)), self.rng.uniform(0.3, 0.7))
        axes[op.AXIS_RIGHT_X] = self.rng.choice([-1, 1]) * self.rng.uniform(0.2, 0.6)
        return Action(label="drive+turn", seconds=self.rng.uniform(0.6, 1.5), axes=axes)

    def spin_up(self) -> Action:
        return Action(
            label="spin-up",
            seconds=self.rng.uniform(0.8, 2.2),
            buttons=frozenset({op.BTN_LEFT_BUMPER}),
        )

    def shoot(self) -> Action:
        """Flywheel plus feed -- leftTrigger runs the spindexer into it."""
        axes = {op.AXIS_LEFT_TRIGGER: 1.0}
        if self.rng.random() < 0.4:  # a driver often creeps while shooting
            angle = self.rng.uniform(0, 2 * math.pi)
            axes.update(_drive_axes((math.cos(angle), math.sin(angle)), self.rng.uniform(0.15, 0.4)))
        return Action(
            label="shoot",
            seconds=self.rng.uniform(1.0, 2.5),
            axes=axes,
            buttons=frozenset({op.BTN_LEFT_BUMPER}),
        )

    def intake(self) -> Action:
        angle = self.rng.uniform(0, 2 * math.pi)
        return Action(
            label="intake",
            seconds=self.rng.uniform(0.8, 2.0),
            axes={
                op.AXIS_RIGHT_TRIGGER: 1.0,
                **_drive_axes((math.cos(angle), math.sin(angle)), self.rng.uniform(0.2, 0.6)),
            },
        )

    def eject(self) -> Action:
        return Action(
            label="eject",
            seconds=self.rng.uniform(0.3, 0.9),
            buttons=frozenset({op.BTN_RIGHT_BUMPER}),
        )

    def deploy_intake(self) -> Action:
        return Action(label="deploy-intake", seconds=0.2, manip_buttons=frozenset({op.BTN_Y}))

    def retract_intake(self) -> Action:
        return Action(label="retract-intake", seconds=0.2, manip_buttons=frozenset({op.BTN_B}))

    def lock_x(self) -> Action:
        return Action(label="lock-x", seconds=0.3, buttons=frozenset({op.BTN_X}))

    def idle(self) -> Action:
        return Action(label="idle", seconds=self.rng.uniform(0.3, 1.0))

    def contradict(self) -> Action:
        """Two mechanism requests in the same tick.

        Intake and eject at once, or deploy and retract at once. A driver does
        this by fumbling; a scripted test never does it at all. The brief lists
        it as a category that essentially never gets written down.
        """
        if self.rng.random() < 0.5:
            return Action(
                label="contradict(in+out)",
                seconds=self.rng.uniform(0.3, 0.8),
                axes={op.AXIS_RIGHT_TRIGGER: 1.0},
                buttons=frozenset({op.BTN_RIGHT_BUMPER}),
            )
        return Action(
            label="contradict(deploy+retract)",
            seconds=0.2,
            manip_buttons=frozenset({op.BTN_Y, op.BTN_B}),
        )

    def mash(self) -> Action:
        """A short burst of unusual-but-possible simultaneous presses."""
        pool = [op.BTN_A, op.BTN_X, op.BTN_Y, op.BTN_LEFT_BUMPER, op.BTN_RIGHT_BUMPER]
        chosen = self.rng.sample(pool, self.rng.randint(2, 3))
        return Action(label="mash", seconds=self.rng.uniform(0.15, 0.5), buttons=frozenset(chosen))

    # -- composition -------------------------------------------------------

    #: Weighted so the operator spends most of its time doing recognisable
    #: things. The awkward moves are seasoning, not the meal -- an operator that
    #: is mostly fumbling never reaches a deep state to fumble *in*.
    WEIGHTS = (
        (drive, 22),
        (drive_and_turn, 14),
        (intake, 14),
        (shoot, 12),
        (turn, 8),
        (spin_up, 7),
        (deploy_intake, 5),
        (retract_intake, 5),
        (eject, 4),
        (idle, 4),
        (contradict, 3),
        (mash, 2),
        (lock_x, 2),
    )

    #: Chance that any given action is abandoned partway. This is the single
    #: most productive knob for reaching interruption-and-re-entry states: a
    #: sequence cut in its third step is the thing scripted tests never build.
    TRUNCATE_CHANCE = 0.22

    def next_action(self) -> Action:
        moves, weights = zip(*self.WEIGHTS)
        move = self.rng.choices(moves, weights=weights, k=1)[0]
        action = move(self)
        if self.rng.random() < self.TRUNCATE_CHANCE:
            action = action.truncated(self.rng.uniform(0.2, 0.6))
        return action

    def recover_toward_centre(self, x: float, y: float) -> Action:
        """Drive back toward the middle of the field. Not a random move."""
        cx, cy = FIELD_CENTRE
        return Action(
            label="recover",
            seconds=self.rng.uniform(0.5, 1.0),
            axes=_drive_axes((cx - x, cy - y), self.rng.uniform(0.5, 0.8)),
        )

    def back_off(self, stuck_on: Action, attempt: int = 0) -> Action:
        """Reverse out of whatever the robot has run into, escalating.

        The wall margin only knows about the perimeter, but the arena has
        physical field elements in the middle of it -- the hub especially, and
        it is big. A driver who bumps one backs off within a moment; a fuzzer
        that leans on it for two seconds trips `frozen-robot` and fills the
        morning report with detections that are true and useless.

        Attempts escalate the way a person does: straight back, then back with
        more rotation, then sideways along the obstacle. Being *more* patient
        here sharpens the detector rather than blunting it -- a robot held by
        geometry comes free eventually, and one whose code has wedged never
        does, no matter how many times it is asked.

        Each attempt stays shorter than the detector's frozen-robot window, so
        ordinary contact never reaches it. What survives is the finding worth
        having: commanded, tried repeatedly to reverse, still not moving.
        """
        reverse = {
            axis: -value
            for axis, value in stuck_on.axes.items()
            if axis in (op.AXIS_LEFT_X, op.AXIS_LEFT_Y)
        }
        if attempt >= 2:
            # Straight back has not worked twice. Slide along the obstacle
            # instead -- swap the axes to push perpendicular to the last push.
            reverse = {
                op.AXIS_LEFT_X: reverse.get(op.AXIS_LEFT_Y, 0.0),
                op.AXIS_LEFT_Y: -reverse.get(op.AXIS_LEFT_X, 0.0),
            }
        magnitude = min(1.0, 0.7 + 0.15 * attempt)
        scale = magnitude / (max(abs(v) for v in reverse.values()) or 1.0)
        axes = {axis: max(-1.0, min(1.0, value * scale)) for axis, value in reverse.items()}
        # Rotation too: a robot wedged on a corner often has to turn off it.
        axes[op.AXIS_RIGHT_X] = self.rng.choice([-1, 1]) * min(0.9, 0.4 + 0.2 * attempt)
        return Action(
            label=f"back-off{attempt + 1}",
            seconds=self.rng.uniform(0.5, 0.5 + 0.25 * (attempt + 1)),
            axes=axes,
        )


def near_wall(x: float, y: float, margin: float = WALL_MARGIN) -> bool:
    return (
        x < margin
        or x > FIELD_LENGTH - margin
        or y < margin
        or y > FIELD_WIDTH - margin
    )

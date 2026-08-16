"""
Normalized driver input, decoupled from whichever GUI framework or
device library actually captures it. `Robot.drive_field_relative`/
`drive_robot_relative` want a chassis-speed-scale command; behavior
scripts (control/behavior.py) construct DriveCommand/OperatorCommand
directly rather than going through an InputSource. An InputSource is
just how a *human* drives -- keyboard/mouse or an Xbox controller.
"""
from __future__ import annotations

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Protocol

# pygame.event.pump() -- needed to refresh joystick axis/button state --
# requires SDL's video subsystem to be initialized, even though this sim
# never opens an SDL window (the GUI is PyQt; see ARCHITECTURE.md). The
# "dummy" driver satisfies that without creating any visible window.
# Must be set before pygame.init() runs; setdefault so a caller that has
# already chosen a real video driver (e.g. to also use pygame for
# rendering) is not overridden.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


@dataclass(frozen=True)
class DriveCommand:
    """Normalized chassis command, each axis in [-1, 1]. The caller
    (typically Robot.drive_field_relative) scales these by the robot's
    actual max_speed/max_angular_speed."""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass(frozen=True)
class OperatorCommand:
    intake_active: bool = False
    deposit_active: bool = False
    deposit_action: str | None = None
    # Edge-triggered (True for exactly one poll() per physical press, not
    # held-down state) -- a GUI/window wires this to pause/resume rather
    # than treating the Start button as a live "paused" level, since the
    # button itself doesn't know which state to go to next.
    pause_toggle: bool = False
    # Edge-triggered, same shape as pause_toggle -- a GUI/window wires this
    # to advancing to the next deposit-action choice (e.g. cycling REEF
    # levels L1-L4), one step per physical press.
    cycle_level: bool = False


class InputSource(ABC):
    @abstractmethod
    def poll(self) -> tuple[DriveCommand, OperatorCommand]:
        raise NotImplementedError

    @property
    def available(self) -> bool:
        """False for a device that failed to initialize (e.g. no gamepad
        plugged in) -- callers should fall back to another InputSource
        rather than treat an unavailable one as all-zero input forever."""
        return True


def _deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class KeyBindings:
    """Key codes are whatever the host GUI framework uses (e.g. Qt's
    Qt.Key_W ints) -- KeyboardInput only ever does set-membership checks
    on them, so it stays framework-agnostic."""
    forward: object
    backward: object
    left: object
    right: object
    rotate_ccw: object
    rotate_cw: object
    intake: object
    deposit: object


class KeyboardInput(InputSource):
    def __init__(
        self,
        pressed_keys: Callable[[], set],
        bindings: KeyBindings,
        deposit_action: str | None = None,
    ):
        """`pressed_keys` is a zero-arg callable returning the currently
        held-down key set -- a callable rather than a fixed set so it can
        read live state (e.g. a Qt widget's tracked key-press set) each
        poll() rather than a snapshot taken at construction time."""
        self._pressed_keys = pressed_keys
        self.bindings = bindings
        self.deposit_action = deposit_action

    def poll(self) -> tuple[DriveCommand, OperatorCommand]:
        keys = self._pressed_keys()
        b = self.bindings
        drive = DriveCommand(
            vx=float((b.right in keys) - (b.left in keys)),
            vy=float((b.forward in keys) - (b.backward in keys)),
            omega=float((b.rotate_ccw in keys) - (b.rotate_cw in keys)),
        )
        operator = OperatorCommand(
            intake_active=b.intake in keys,
            deposit_active=b.deposit in keys,
            deposit_action=self.deposit_action,
        )
        return drive, operator


class _JoystickLike(Protocol):
    def get_numaxes(self) -> int: ...
    def get_axis(self, index: int) -> float: ...
    def get_numbuttons(self) -> int: ...
    def get_button(self, index: int) -> int: ...


class GamepadInput(InputSource):
    """Xbox controller input via pygame's joystick subsystem, initialized
    headless (no pygame display window needed -- see ARCHITECTURE.md's
    tech-stack rationale). Standard Xbox axis/button layout as SDL2
    reports it: left stick = axes 0/1 (strafe/forward), right stick X =
    axis 3 (rotate), A = button 0 (intake), right trigger (RT) = axis 5
    (deposit) -- a trigger rather than a button since it's a more natural
    hold-to-score analog to the keyboard's held-F binding.

    A `joystick` object can be injected directly (anything satisfying
    get_axis/get_button/get_numaxes) -- this is what makes the class
    testable without a physical controller attached; production code
    should leave it as None and let the class auto-detect device
    `index` via pygame."""

    DEADBAND = 0.12
    # Raw pygame.joystick axis indices for a standard Xbox controller as
    # SDL2 reports them via Windows' XInput backend: 0=LX, 1=LY, 2=RX,
    # 3=RY, 4=LT, 5=RT. (Note this differs from some other
    # platforms/backends where RX/RY sit at 3/4 instead -- there is no
    # single cross-platform-guaranteed raw joystick layout, which is why
    # rotation ended up wired to RY/up-down here originally.) Buttons 0-3
    # are A/B/X/Y and button 7 is Start on the same standard mapping.
    ROTATE_AXIS = 2
    START_BUTTON = 7
    X_BUTTON = 2
    # SDL2/XInput reports the right trigger as its own axis (not a
    # button), 0.0 released to 1.0 fully pressed -- 0.5 reads as a
    # deliberate half-press rather than requiring the trigger bottomed out.
    RIGHT_TRIGGER_AXIS = 5
    RIGHT_TRIGGER_THRESHOLD = 0.5

    def __init__(
        self,
        index: int = 0,
        deposit_action: str | None = None,
        joystick: _JoystickLike | None = None,
    ):
        self.deposit_action = deposit_action
        self._joystick = joystick
        # An injected joystick (test doubles) manages its own state and
        # was never opened through pygame, so polling it should never
        # touch pygame's event queue -- only a real, auto-detected
        # pygame.joystick.Joystick needs pump() to refresh axis state.
        self._owns_pygame_device = joystick is None
        if joystick is None:
            self._joystick = _open_pygame_joystick(index)
        self._prev_start_pressed = False
        self._prev_x_pressed = False

    @property
    def available(self) -> bool:
        return self._joystick is not None

    def poll(self) -> tuple[DriveCommand, OperatorCommand]:
        if self._joystick is None:
            return DriveCommand(), OperatorCommand()

        if self._owns_pygame_device:
            _pump_pygame_events()
        j = self._joystick
        vx = _clamp(_deadband(j.get_axis(0), self.DEADBAND))
        vy = _clamp(_deadband(-j.get_axis(1), self.DEADBAND))
        omega = 0.0
        if j.get_numaxes() > self.ROTATE_AXIS:
            omega = _clamp(_deadband(-j.get_axis(self.ROTATE_AXIS), self.DEADBAND))
        drive = DriveCommand(vx=vx, vy=vy, omega=omega)

        intake = j.get_numbuttons() > 0 and bool(j.get_button(0))
        deposit = (
            j.get_numaxes() > self.RIGHT_TRIGGER_AXIS
            and j.get_axis(self.RIGHT_TRIGGER_AXIS) > self.RIGHT_TRIGGER_THRESHOLD
        )

        start_pressed = j.get_numbuttons() > self.START_BUTTON and bool(j.get_button(self.START_BUTTON))
        pause_toggle = start_pressed and not self._prev_start_pressed
        self._prev_start_pressed = start_pressed

        x_pressed = j.get_numbuttons() > self.X_BUTTON and bool(j.get_button(self.X_BUTTON))
        cycle_level = x_pressed and not self._prev_x_pressed
        self._prev_x_pressed = x_pressed

        operator = OperatorCommand(
            intake_active=intake, deposit_active=deposit, deposit_action=self.deposit_action,
            pause_toggle=pause_toggle, cycle_level=cycle_level,
        )
        return drive, operator


def _open_pygame_joystick(index: int):
    try:
        import pygame
    except ImportError:
        return None

    if not pygame.get_init():
        pygame.init()
    if not pygame.joystick.get_init():
        pygame.joystick.init()
    if pygame.joystick.get_count() <= index:
        return None

    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return joystick


def _pump_pygame_events() -> None:
    import pygame

    pygame.event.pump()

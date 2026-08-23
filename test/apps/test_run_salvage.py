"""The SALVAGE driving window.

Two things are worth pinning and the rest is Qt's problem:

* a human can actually complete a cycle -- drive, load, drive, score.
  That path has no controller in it, so nothing else in the suite
  exercises it, and it is the entire point of the app;
* the keyboard works when a gamepad is plugged in. That is not
  hypothetical: the machine this was written on has a controller
  attached, `GamepadInput.available` was True, and the first scripted
  play-through moved the robot exactly zero inches with no error and no
  message.
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets                                       # noqa: E402

from common_sim.control.input_sources import DriveCommand, OperatorCommand  # noqa: E402
from game_specific.salvage.field import cargo_bay_positions, hold_low_position  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def window(app):
    from apps.run_salvage import SalvageWindow

    view = SalvageWindow()
    view.timer.stop()  # the test drives _tick itself
    return view


class _FakePad:
    """A gamepad that is present and being held hard over to one side --
    the thing `CombinedInput` has to not let take the keyboard away."""

    def __init__(self, available=True, vx=0.0, intake=False):
        self.available = available
        self._vx = vx
        self._intake = intake

    def poll(self):
        return (
            DriveCommand(vx=self._vx, vy=0.0, omega=0.0),
            OperatorCommand(intake_active=self._intake),
        )


def _drive_to(window, target, hold_key=None, ticks=1200, tolerance=10.0):
    """Press the keys a person would to get to `target`, one tick at a
    time, optionally holding intake/deposit on the way."""
    from apps.run_salvage import KEY_BINDINGS

    for _ in range(ticks):
        pose = window.robot.pose
        keys = set()
        if target[0] - pose.x > 3:
            keys.add(KEY_BINDINGS.right)
        elif target[0] - pose.x < -3:
            keys.add(KEY_BINDINGS.left)
        if target[1] - pose.y > 3:
            keys.add(KEY_BINDINGS.forward)
        elif target[1] - pose.y < -3:
            keys.add(KEY_BINDINGS.backward)
        if hold_key is not None:
            keys.add(hold_key)
        window._pressed_keys = keys
        window._tick()
        if abs(window.robot.pose.x - target[0]) < tolerance and abs(window.robot.pose.y - target[1]) < tolerance:
            return True
    return False


def test_a_human_can_drive_load_and_score(window):
    from apps.run_salvage import KEY_BINDINGS

    assert window.robot.controller is None, "the driven robot must have no AI attached"

    bay = cargo_bay_positions("blue")[0]
    assert _drive_to(window, bay), "could not drive to a CARGO BAY"
    assert "IN blue_cargo_bay_0" in window.field_label.text()

    for _ in range(400):
        window._pressed_keys = {KEY_BINDINGS.intake}
        window._tick()
        if window.robot.held_pieces:
            break
    assert [p.piece_type for p in window.robot.held_pieces] == ["crate"]

    hold = hold_low_position("blue")
    assert _drive_to(window, hold, tolerance=8.0), "could not drive to the wall hold"
    assert "hold F to score hold_low" in window.field_label.text()

    before = window.match.scores.get("blue", 0.0)
    for _ in range(500):
        window._pressed_keys = {KEY_BINDINGS.deposit}
        window._tick()
        if not window.robot.held_pieces:
            break
    assert not window.robot.held_pieces
    assert window.match.scores.get("blue", 0.0) > before


def test_the_keyboard_still_drives_with_a_gamepad_connected(window):
    """The trap this app exists not to repeat: `available` means pygame
    found a device, not that a human is holding it."""
    from apps.run_salvage import KEY_BINDINGS

    window.input_source.gamepad = _FakePad(available=True)
    start = window.robot.pose.x
    for _ in range(60):
        window._pressed_keys = {KEY_BINDINGS.right}
        window._tick()
    assert window.robot.pose.x > start + 5.0


def test_a_held_stick_still_drives_with_the_keyboard_idle(window):
    window.input_source.gamepad = _FakePad(available=True, vx=1.0)
    start = window.robot.pose.x
    for _ in range(60):
        window._pressed_keys = set()
        window._tick()
    assert window.robot.pose.x > start + 5.0


def test_the_scoring_panel_reports_the_current_phase_not_a_fixed_table(window):
    """The panel is the game's explanation of itself, so it has to track
    the phase rather than print the AUTO column forever."""
    window._tick()
    assert "AUTO" in window.scoring_label.text()
    auto_text = window.scoring_label.text()

    while window.match.phase.value == "auto":
        window._pressed_keys = set()
        window._tick()
    window._tick()

    teleop_text = window.scoring_label.text()
    assert teleop_text != auto_text
    # HOLD HIGH is the one that goes up rather than down: 3 -> 8.
    assert "HOLD HIGH (crate)     8" in teleop_text
    assert "HOLD LOW  (crate)     2" in teleop_text


def test_reset_rebuilds_a_drivable_match(window):
    window._tick()
    first = window.match
    window._reset_match()
    assert window.match is not first
    assert window.robot.controller is None
    assert window.match.elapsed == 0.0

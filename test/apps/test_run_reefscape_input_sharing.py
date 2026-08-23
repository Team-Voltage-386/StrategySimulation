"""The trap this window fell into for real: it used to pick one input
source -- `gamepad if gamepad.available else keyboard` -- and
`available` means "pygame found a joystick device", not "a human is
holding it". So a controller merely plugged into the machine made
W A S D do nothing at all, with no error and no message, which is a
confusing five minutes alone at a desk and a lost demo in front of a
room. `run_salvage.py` was written not to repeat it; this pins the
same guarantee here now that both share `CombinedInput`.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from common_sim.control.input_sources import DriveCommand, OperatorCommand  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _FakePad:
    """Present, and doing whatever the test says."""

    available = True

    def __init__(self, vx=0.0, intake=False):
        self._vx = vx
        self._intake = intake

    def poll(self):
        return (
            DriveCommand(vx=self._vx, vy=0.0, omega=0.0),
            OperatorCommand(intake_active=self._intake),
        )


def _view(pad):
    view = run_reefscape.MatchView()
    view.timer.stop()  # the test drives _tick itself
    view.input_source.gamepad = pad
    # MatchView opens paused behind its transport bar (SalvageWindow does
    # not), and a paused tick polls input but never steps the match.
    if view.paused:
        view._toggle_paused()
    return view


def _drive(view, keys, ticks=60):
    """Poll input the way the real window does, then step physics by a
    fixed dt. `_tick` alone advances the match by ~0 here: unpaused it
    uses the wall-clock accumulator in `_advance_realtime`, and the fixed
    dt path behind it is gated on `fast_forward_enabled()`, which
    requires AI to be driving PRIMARY -- the one thing these tests can't
    turn on. HumanController reads `_latest_commands`, which `_tick` has
    already filled in by then, so the input path under test is still the
    real one end to end."""
    for _ in range(ticks):
        view._pressed_keys = set(keys)
        view._tick()
        view._advance_one_step(1.0 / view.TICK_HZ)


def test_the_keyboard_still_drives_with_a_gamepad_connected(app):
    view = _view(_FakePad())
    start = view.robot.pose.x
    _drive(view, {run_reefscape.KEY_BINDINGS.right})
    assert view.robot.pose.x > start + 5.0


def test_a_held_stick_still_drives_with_the_keyboard_idle(app):
    view = _view(_FakePad(vx=1.0))
    start = view.robot.pose.x
    _drive(view, set())
    assert view.robot.pose.x > start + 5.0


def test_either_device_can_command_intake(app):
    """Buttons OR rather than sum, so the pad's A and the keyboard's
    Space are both live rather than the last one polled winning."""
    view = _view(_FakePad(intake=True))
    view._pressed_keys = set()
    _, operator = view.input_source.poll()
    assert operator.intake_active

    view = _view(_FakePad(intake=False))
    view._pressed_keys = {run_reefscape.KEY_BINDINGS.intake}
    _, operator = view.input_source.poll()
    assert operator.intake_active


def test_the_controls_panel_switches_to_the_gamepad_reference_once_one_is_connected(app):
    """Both devices stay live either way (see the keyboard/gamepad tests
    above) -- this only covers which reference list the panel shows, and
    showing both at once made the panel too wide."""
    panel = run_reefscape.ControlsPanel()

    panel.set_available(False)
    assert "W A S D" in panel.label.text()
    assert "GAMEPAD" not in panel.label.text()

    panel.set_available(True)
    assert "W A S D" not in panel.label.text()
    assert "Left Stick" in panel.label.text()

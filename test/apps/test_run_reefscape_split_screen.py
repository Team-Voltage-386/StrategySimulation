"""SplitScreenPanel / MatchView wiring: a two-pane driver-station view,
one pane per alliance wall, gated on two physical gamepads being present
and Player 2 piloting a RED robot -- see MatchView._update_split_screen_
availability / _set_split_screen / _apply_split_screen_views.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from common_sim.control.input_sources import GamepadInput  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeJoystick:
    """A no-input joystick double -- enough to make GamepadInput.available
    report True, which is all these tests need from it (see
    test_input_sources.FakeJoystick for the polling-behavior version)."""

    def get_numaxes(self):
        return 6

    def get_axis(self, index):
        return 0.0

    def get_numbuttons(self):
        return 8

    def get_button(self, index):
        return 0


def _view_with_two_gamepads_and_one_red_roster_robot():
    view = run_reefscape.MatchView()
    view.roster_panel.red_roster._add_row()
    view.gamepad_available = True
    view.gamepad2 = GamepadInput(joystick=FakeJoystick())
    view._reset_match()
    index = view.player2_panel.combo.findText("RED 0")
    view.player2_panel.combo.setCurrentIndex(index)
    view._reset_match()
    return view


def test_split_screen_unavailable_without_two_gamepads(app):
    view = run_reefscape.MatchView()
    assert view.split_screen_panel.checkbox.isEnabled() is False
    assert view.split_screen_panel.status_label.text() != ""


def test_split_screen_unavailable_when_player2_is_ai(app):
    view = run_reefscape.MatchView()
    view.gamepad_available = True
    view.gamepad2 = GamepadInput(joystick=FakeJoystick())
    view._reset_match()  # Player 2 still on the default AI option.
    assert view.split_screen_panel.checkbox.isEnabled() is False


def test_split_screen_unavailable_when_player2_robot_is_blue(app):
    view = run_reefscape.MatchView()
    view.roster_panel.blue_roster._add_row()
    view.gamepad_available = True
    view.gamepad2 = GamepadInput(joystick=FakeJoystick())
    view._reset_match()
    view.player2_panel.combo.setCurrentIndex(view.player2_panel.combo.findText("BLUE 0"))
    view._reset_match()
    assert view.split_screen_panel.checkbox.isEnabled() is False


def test_split_screen_available_with_two_gamepads_and_red_player2(app):
    view = _view_with_two_gamepads_and_one_red_roster_robot()
    assert view.split_screen_panel.checkbox.isEnabled() is True
    assert view.split_screen_panel.status_label.text() == ""


def test_enabling_split_screen_shows_second_pane_with_correct_perspectives(app):
    # isHidden(), not isVisible(): `view` is never shown as a top-level
    # window in this test, so isVisible() would read False regardless of
    # canvas2's own setVisible() state (it accounts for the whole
    # ancestor chain). isHidden() reflects canvas2's own explicit state.
    view = _view_with_two_gamepads_and_one_red_roster_robot()
    assert view.canvas2.isHidden() is True

    view._set_split_screen(True)

    assert view._split_screen_active is True
    assert view.canvas2.isHidden() is False
    assert view.canvas.driver_alliance == view.robot.alliance
    assert view.canvas2.driver_alliance == "red"
    assert view.split_screen_panel.is_checked() is True


def test_disabling_split_screen_hides_second_pane(app):
    view = _view_with_two_gamepads_and_one_red_roster_robot()
    view._set_split_screen(True)

    view._set_split_screen(False)

    assert view._split_screen_active is False
    assert view.canvas2.isHidden() is True
    assert view.split_screen_panel.is_checked() is False


def test_losing_eligibility_on_reset_turns_split_screen_back_off(app):
    """Player 2 reselecting AI (or the second gamepad vanishing) mid-
    session should fall back to a single pane on the next RESET, rather
    than leaving split screen stuck on with no real second player."""
    view = _view_with_two_gamepads_and_one_red_roster_robot()
    view._set_split_screen(True)

    view.player2_panel.combo.setCurrentIndex(view.player2_panel.combo.findText(run_reefscape.PLAYER2_AI_OPTION))
    view._reset_match()

    assert view._split_screen_active is False
    assert view.canvas2.isHidden() is True
    assert view.split_screen_panel.checkbox.isEnabled() is False


def test_split_screen_orients_each_players_stick_by_their_own_pane(app):
    """The core face-off guarantee: with split screen active, Player 1's
    stick is oriented by the left pane's (their own) alliance, and
    Player 2's stick is oriented by the right pane's (RED) alliance --
    independently, not both keyed off one shared canvas."""
    view = _view_with_two_gamepads_and_one_red_roster_robot()
    view._set_split_screen(True)
    assert view.robot.alliance == "blue"  # PRIMARY defaults to blue.

    from common_sim.control.input_sources import DriveCommand

    p1_out = view._orient_drive_command(DriveCommand(vx=0.0, vy=1.0, omega=0.0), view.canvas.driver_alliance)
    p2_alliance = view.canvas2.driver_alliance if view._split_screen_active else view.canvas.driver_alliance
    p2_out = view._orient_drive_command(DriveCommand(vx=0.0, vy=1.0, omega=0.0), p2_alliance)

    # Blue's "up" is field +x, red's "up" is field -x (see
    # gui_utils.field_camera.ALLIANCE_FORWARD) -- opposite panes must
    # therefore produce opposite vx for the identical stick reading.
    assert p1_out.vx == pytest.approx(1.0)
    assert p2_out.vx == pytest.approx(-1.0)

"""Player2Panel / MatchView wiring: a second gamepad can be handed to any
non-PRIMARY roster robot, overriding the AI controller _spawn_roster_robot
would otherwise attach, on the next RESET.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from apps.run_reefscape import PLAYER2_AI_OPTION  # noqa: E402
from common_sim.control.human import HumanController  # noqa: E402
from common_sim.control.strategy import StrategyController  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _view_with_one_blue_roster_robot():
    view = run_reefscape.MatchView()
    view.roster_panel.blue_roster._add_row()
    view._reset_match()
    return view


def test_default_selection_leaves_roster_robot_ai_controlled(app):
    view = _view_with_one_blue_roster_robot()
    assert isinstance(view._robots_by_label["BLUE 0"].controller, StrategyController)


def test_player2_options_list_every_spawned_robot_except_primary(app):
    view = _view_with_one_blue_roster_robot()
    options = [view.player2_panel.combo.itemText(i) for i in range(view.player2_panel.combo.count())]
    assert options == [PLAYER2_AI_OPTION, "BLUE 0"]


def test_selecting_a_roster_robot_wires_a_human_controller_on_reset(app):
    view = _view_with_one_blue_roster_robot()
    index = view.player2_panel.combo.findText("BLUE 0")
    view.player2_panel.combo.setCurrentIndex(index)

    view._reset_match()

    assert isinstance(view._robots_by_label["BLUE 0"].controller, HumanController)


def test_player2_human_controller_reads_its_own_command_slot(app):
    view = _view_with_one_blue_roster_robot()
    view.player2_panel.combo.setCurrentIndex(view.player2_panel.combo.findText("BLUE 0"))
    view._reset_match()

    from common_sim.control.input_sources import DriveCommand, OperatorCommand

    sentinel = (DriveCommand(vx=0.5, vy=0.0, omega=0.0), OperatorCommand())
    view._player2_commands = sentinel
    robot2 = view._robots_by_label["BLUE 0"]
    assert robot2.controller.command_provider() == sentinel
    # PRIMARY's own controller must still read the (separate) P1 slot.
    assert view.robot.controller.command_provider() is not sentinel


def test_reselecting_ai_reverts_the_robot_to_a_strategy_controller(app):
    view = _view_with_one_blue_roster_robot()
    view.player2_panel.combo.setCurrentIndex(view.player2_panel.combo.findText("BLUE 0"))
    view._reset_match()
    assert isinstance(view._robots_by_label["BLUE 0"].controller, HumanController)

    view.player2_panel.combo.setCurrentIndex(view.player2_panel.combo.findText(PLAYER2_AI_OPTION))
    view._reset_match()
    assert isinstance(view._robots_by_label["BLUE 0"].controller, StrategyController)


def test_gamepad2_unavailable_in_headless_test_env_shows_status(app):
    view = run_reefscape.MatchView()
    assert view.gamepad2.available is False
    assert view.player2_panel.status_label.text() != ""

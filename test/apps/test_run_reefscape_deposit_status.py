"""MatchView's Deposit telemetry line -- the driver-facing explanation of
why a deposit would or wouldn't score.

The canvas rings the robot red for every kind of refusal at once, so
until this line existed a blocked REEF face and an out-of-position robot
looked exactly the same from the driver's seat. That matters most for
the ALGAE gate, which is a rule nothing else on screen states: the
staged ALGAE draws as a small box with a 1 in it, and never says it is
the reason the L2/L3 behind it is refusing CORAL.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from common_sim.geometry import Pose2d  # noqa: E402
from game_specific.reefscape.field import (  # noqa: E402
    REEF_HEX_APOTHEM, reef_algae_blocked_level, reef_algae_location_name, reef_center,
)
from game_specific.reefscape.game_pieces import CORAL_TYPE  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


BLUE_FACE_INDEX = 3  # the REEF's -x face; gated at l3 (see Figure 6-3)


def _view():
    view = run_reefscape.MatchView()
    view.timer.stop()
    return view


def _park_at_blue_face(view):
    """Put the primary robot in front of the -x REEF face with that
    face's gated level selected. It already starts holding a preloaded
    CORAL, which is exactly the piece under discussion."""
    cx, cy = reef_center("blue")
    face_x = cx - REEF_HEX_APOTHEM
    # Standing off the face so the robot's FRONT edge lands inside the
    # zone -- side_engages_polygon tests the scoring side's reach points,
    # not the chassis centre, so parking the centre in the zone puts the
    # front through the far side of it and reads "no zone here".
    view.robot.chassis.body.position = (face_x - 17.0, cy)
    view.robot.chassis.body.angle = 0.0
    assert [p.piece_type for p in view.robot.held_pieces] == [CORAL_TYPE]

    view._selected_deposit_action = reef_algae_blocked_level(BLUE_FACE_INDEX)
    return view


def _open_the_gate(view):
    name = reef_algae_location_name("blue", BLUE_FACE_INDEX)
    for location in list(view.match.station_supply):
        if location.name == name:
            view.match.station_supply[location] = 0


def test_empty_handed_says_so(app):
    view = _view()
    view.robot.held_pieces.clear()  # the primary robot starts preloaded
    assert view._deposit_status() == "nothing held"


def test_holding_a_coral_away_from_any_zone_says_no_zone(app):
    view = _view()
    view.robot.chassis.body.position = (view.match.field.width / 2.0, 8.0)

    status = view._deposit_status()
    assert "no zone here" in status
    assert CORAL_TYPE.upper() in status


def test_a_gated_face_reports_the_algae_not_just_a_refusal(app):
    """The line this whole file exists for."""
    view = _park_at_blue_face(_view())
    assert view._deposit_status() == "L3: BLOCKED by ALGAE"


def test_the_same_face_reads_ready_once_the_algae_is_gone(app):
    """Paired with the test above so the message is shown to track the
    gate, rather than being whatever that spot always says."""
    view = _park_at_blue_face(_view())
    _open_the_gate(view)
    assert view._deposit_status() == "L3: ready"


def test_an_ungated_level_on_a_gated_face_is_still_ready(app):
    """Only the one staged level is blocked -- L4 above it is not, and a
    driver needs to be told that's the way out."""
    view = _park_at_blue_face(_view())
    view._selected_deposit_action = "l4"
    assert view._deposit_status() == "L4: ready"


def test_the_status_line_reaches_the_telemetry_panel(app):
    view = _park_at_blue_face(_view())
    view._update_telemetry()
    text = view.telemetry_panel.body_label.text()
    assert "Deposit" in text
    assert "BLOCKED by ALGAE" in text

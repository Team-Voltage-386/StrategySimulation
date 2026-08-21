"""MatchView._orient_drive_command -- the remap that makes WASD/stick
axes driver-relative (up = away from the driver, right = the driver's
own right hand) while a driver-station camera is active, and a
no-op pass-through in TOP-DOWN, which keeps its established
screen-relative convention (up=+y, right=+x) unchanged.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from common_sim.control.input_sources import DriveCommand  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_top_down_leaves_drive_command_unchanged(app):
    view = run_reefscape.MatchView()
    drive = DriveCommand(vx=0.3, vy=-0.7, omega=0.1)
    assert view._orient_drive_command(drive) == drive


def test_driver_blue_up_becomes_field_plus_x(app):
    view = run_reefscape.MatchView()
    view.canvas.set_driver_view("blue")
    out = view._orient_drive_command(DriveCommand(vx=0.0, vy=1.0, omega=0.0))
    assert out.vx == pytest.approx(1.0)
    assert out.vy == pytest.approx(0.0)


def test_driver_red_up_becomes_field_minus_x(app):
    view = run_reefscape.MatchView()
    view.canvas.set_driver_view("red")
    out = view._orient_drive_command(DriveCommand(vx=0.0, vy=1.0, omega=0.0))
    assert out.vx == pytest.approx(-1.0)
    assert out.vy == pytest.approx(0.0)


def test_driver_view_right_stick_moves_right_on_that_alliances_screen(app):
    """End-to-end version of field_camera's orient_drive test: drive the
    remapped command through the real camera projection and check the
    robot's apparent screen position actually moves right."""
    for alliance in ("blue", "red"):
        view = run_reefscape.MatchView()
        view.resize(1000, 500)
        view.canvas.set_driver_view(alliance)
        scale = view.canvas._field_scale()
        camera = view.canvas._build_camera()

        cx, cy = view.match.field.width / 2.0, view.match.field.height / 2.0
        near_px, _, _ = camera.project(cx, cy, 0.0)

        out = view._orient_drive_command(DriveCommand(vx=1.0, vy=0.0, omega=0.0))
        nudged = (cx + out.vx, cy + out.vy)
        far_px, _, _ = camera.project(*nudged, 0.0)

        assert far_px > near_px, f"{alliance}: 'right' input should move right on screen"


def test_omega_passes_through_unchanged_in_top_down(app):
    view = run_reefscape.MatchView()
    out = view._orient_drive_command(DriveCommand(vx=0.0, vy=0.0, omega=0.42))
    assert out.omega == pytest.approx(0.42)


def test_omega_is_negated_in_driver_view(app):
    """A ground-level driver view is left-handed relative to the top-down
    map's screen convention (see _orient_drive_command's docstring), so
    the same field-CCW spin the map draws as counter-clockwise reads as
    clockwise from the driver's own eye -- negating omega keeps 'rotate
    cw' swinging the nose toward the driver's own right in both views."""
    for alliance in ("blue", "red"):
        view = run_reefscape.MatchView()
        view.canvas.set_driver_view(alliance)
        out = view._orient_drive_command(DriveCommand(vx=0.0, vy=0.0, omega=0.42))
        assert out.omega == pytest.approx(-0.42)


def test_rotate_cw_key_swings_the_nose_toward_the_drivers_own_right(app):
    """End-to-end check of the handedness-flip fix: starting nose-away
    (heading = the alliance's own forward direction), a small dt of
    'rotate cw' input should swing the heading toward the driver's own
    right on screen, for both alliances, matching a chase-view convention."""
    import math

    from common_sim.geometry import Pose2d
    from gui_utils.field_camera import ALLIANCE_FORWARD

    for alliance in ("blue", "red"):
        view = run_reefscape.MatchView()
        view.canvas.set_driver_view(alliance)
        fx, fy = ALLIANCE_FORWARD[alliance]
        heading = math.atan2(fy, fx)
        view.robot.chassis.body.position = (view.robot.pose.x, view.robot.pose.y)
        view.robot.chassis.body.angle = heading

        # rotate_cw is bound to a negative omega reading (see KEY_BINDINGS).
        out = view._orient_drive_command(DriveCommand(vx=0.0, vy=0.0, omega=-1.0))
        # Advancing heading by the sign of out.omega should move the nose
        # toward +right in the (forward, right) alliance basis.
        from gui_utils.field_camera import ALLIANCE_RIGHT

        rx, ry = ALLIANCE_RIGHT[alliance]
        new_heading = heading + out.omega * 0.05
        nose_before = (math.cos(heading), math.sin(heading))
        nose_after = (math.cos(new_heading), math.sin(new_heading))
        delta = (nose_after[0] - nose_before[0], nose_after[1] - nose_before[1])
        assert delta[0] * rx + delta[1] * ry > 0, f"{alliance}: rotate_cw should swing the nose toward driver-right"

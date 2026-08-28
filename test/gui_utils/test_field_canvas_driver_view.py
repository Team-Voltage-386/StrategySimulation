"""FieldCanvas's driver-station view. Deliberately a rendering smoke test,
not a pixel comparison: the value here is catching exceptions in the
corner-projection/depth-sort path (a perspective camera is exactly the
kind of thing that raises ZeroDivisionError or draws NaN off in a corner
instead of failing loudly), plus the couple of numeric invariants that
are cheap to check without a golden image.
"""
from __future__ import annotations

import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

from apps.reefscape_widgets import build_demo_characteristics, build_demo_match  # noqa: E402
from common_sim.control.human import HumanController  # noqa: E402
from common_sim.geometry import Pose2d  # noqa: E402
from game_specific.reefscape import sweep_trial  # noqa: E402
from gui_utils.field_camera import ELEVATED  # noqa: E402
from gui_utils.field_canvas import FieldCanvas  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _match_with_robots():
    match = build_demo_match()
    for alliance, index in (("blue", -1), ("blue", 0), ("red", -1)):
        pose = sweep_trial.start_pose(alliance, index)
        robot = match.add_robot(build_demo_characteristics(), pose, alliance=alliance)
        robot.set_intake_active(True)
    return match


def _render(canvas: FieldCanvas) -> None:
    pixmap = QtWidgets.QApplication.instance() and canvas.grab()
    assert pixmap is not None
    assert not pixmap.isNull()


def test_top_down_still_renders(app):
    canvas = FieldCanvas(_match_with_robots())
    canvas.resize(1000, 500)
    _render(canvas)


@pytest.mark.parametrize("alliance", ["blue", "red"])
def test_driver_view_renders_without_exceptions(app, alliance):
    canvas = FieldCanvas(_match_with_robots())
    canvas.resize(1000, 500)
    canvas.set_driver_view(alliance)
    _render(canvas)


def test_elevated_preset_renders(app):
    canvas = FieldCanvas(_match_with_robots())
    canvas.resize(1000, 500)
    canvas.set_driver_view("blue", ELEVATED)
    _render(canvas)


def test_switching_back_to_top_down_restores_orthographic_projection(app):
    match = _match_with_robots()
    canvas = FieldCanvas(match)
    canvas.resize(1000, 500)
    canvas.set_driver_view("blue")
    _render(canvas)
    canvas.set_driver_view(None)
    _render(canvas)
    assert canvas._camera is None
    scale = canvas._field_scale()
    wx, wy = canvas._to_widget(0.0, 0.0, scale)
    assert math.isclose(wx, canvas.MARGIN)
    assert math.isclose(wy, canvas.height() - canvas.MARGIN)


def test_far_robot_draws_smaller_than_near_robot_under_perspective(app):
    match = _match_with_robots()
    canvas = FieldCanvas(match)
    canvas.resize(1000, 500)
    canvas.set_driver_view("blue")
    scale = canvas._field_scale()
    canvas._camera = canvas._build_camera()

    near_scale = canvas._scale_at(50.0, match.field.height / 2.0, scale)
    far_scale = canvas._scale_at(match.field.width - 50.0, match.field.height / 2.0, scale)
    assert far_scale < near_scale


def test_robot_corner_projection_matches_top_down_center(app):
    """With no camera, the new corner-projection path for the robot body
    must still center on the robot's pose -- a regression check that the
    translate()+rotate() -> explicit-corner rewrite didn't shift anything."""
    match = build_demo_match()
    pose = Pose2d(x=200.0, y=100.0, heading=0.3)
    robot = match.add_robot(build_demo_characteristics(), pose, alliance="blue")
    canvas = FieldCanvas(match)
    canvas.resize(1000, 500)
    scale = canvas._field_scale()

    corners = canvas._rect_corners(robot)
    avg_x = sum(c[0] for c in corners) / len(corners)
    avg_y = sum(c[1] for c in corners) / len(corners)
    assert math.isclose(avg_x, pose.x, abs_tol=1e-6)
    assert math.isclose(avg_y, pose.y, abs_tol=1e-6)


def test_human_controlled_robot_renders_with_a_glow(app):
    """Rendering smoke test for the human-pilot glow -- catches
    exceptions in _draw_human_glow's QRadialGradient path, same spirit
    as the rest of this file."""
    match = build_demo_match()
    pose = Pose2d(x=200.0, y=100.0, heading=0.0)
    robot = match.add_robot(build_demo_characteristics(), pose, alliance="blue")
    robot.controller = HumanController(command_provider=lambda: (None, None))
    canvas = FieldCanvas(match)
    canvas.resize(1000, 500)
    _render(canvas)


def test_human_glow_paints_alliance_color_that_fades_to_transparent(app):
    """_draw_human_glow itself: a halo center should carry visible
    alliance-colored alpha, fading to fully transparent by its edge --
    the numeric invariant a pixel comparison would otherwise stand in
    for."""
    from pyqtgraph.Qt import QtCore, QtGui

    match = build_demo_match()
    robot = match.add_robot(build_demo_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    canvas = FieldCanvas(match)

    image = QtGui.QImage(200, 200, QtGui.QImage.Format_ARGB32)
    image.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(image)
    canvas._draw_human_glow(painter, robot, cx=100.0, cy=100.0, half_extent=20.0)
    painter.end()

    center = image.pixelColor(100, 100)
    edge = image.pixelColor(100 + int(20.0 * 1.8) - 1, 100)
    outside = image.pixelColor(199, 199)
    assert center.alpha() > 0
    assert edge.alpha() < center.alpha()
    assert outside.alpha() == 0


def test_only_human_controlled_robot_gets_manipulator_direction_arrows(app):
    from pyqtgraph.Qt import QtCore, QtGui

    match = build_demo_match()
    robot = match.add_robot(build_demo_characteristics(), Pose2d(200, 100, 0), alliance="blue")
    canvas = FieldCanvas(match)
    canvas.resize(1000, 500)
    image = QtGui.QImage(1000, 500, QtGui.QImage.Format_ARGB32)
    image.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(image)
    arrows = []
    canvas._draw_screen_arrow = lambda *args: arrows.append(args)

    canvas._draw_side_manipulators(painter, robot, canvas._field_scale())
    assert not arrows

    robot.controller = HumanController(command_provider=lambda: (None, None))
    canvas._draw_side_manipulators(painter, robot, canvas._field_scale())
    painter.end()
    assert arrows

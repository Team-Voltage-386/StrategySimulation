"""
QPainter side-view (X-Z plane) of the mechanism-orchestration sandbox's
elevator+arm superstructure, driven live over NT4 from the Java robot sim
(see MechanismOrchestrationSandbox, a sibling repo, and
common_sim/telemetry/nt4_client.py). Deliberately its own small window
rather than plugged into FieldCanvas/Match -- this sandbox is for
characterizing mechanism cycle times in isolation, not for driving the
top-down strategy sim; see apps/run_mechanism_view.py.

Also draws the CubeShelfScenario demo's field geometry (the shelf wall +
its two slots) and the cube piece, so a scoring attempt's approach can be
watched for whether the gripper actually threads the slot or clips the
wall -- see frc.robot.game.Shelf/Cube on the Java side, mirrored here by
hand the same way the mechanism dimensions already are.
"""
from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from common_sim.telemetry.nt4_client import MechanismSnapshot
from gui_utils import theme

Qt = QtCore.Qt

# Must match the Java project's physical constants (Elevator.java /
# Arm.java) -- there's no shared source of truth across the language
# boundary, so these are duplicated by hand.
ARM_LENGTH_M = 0.70
ELEVATOR_MAX_HEIGHT_M = 1.35
CHASSIS_WIDTH_M = 0.80
CHASSIS_HEIGHT_M = 0.20
# Must match Arm.THICKNESS_M -- the arm is drawn at its true cross-section
# because that is what the robot-side collision check uses.
ARM_THICKNESS_M = 0.06

# Must match frc.robot.game.Shelf.
WALL_NEAR_X = -1.42
WALL_FAR_X = -1.58
WALL_TOP_Z = 1.55
LOW_SLOT_Z = (0.40, 0.70)
HIGH_SLOT_Z = (1.00, 1.30)

# Fixed world span shown at once -- wide enough to keep the cube's start
# position (X=2.5, see CubeShelfScenario.CUBE_START_X) and the shelf wall
# both on screen together, with margin either side.
VIEW_WORLD_X_MIN = -2.0
VIEW_WORLD_X_MAX = 3.0

PIECE_SIZE_M = 0.18


class MechanismCanvas(QtWidgets.QWidget):
    MARGIN = 40
    # Vertical span the view draws, in meters above the ground -- tall
    # enough for the elevator's full travel plus the arm swung vertical,
    # and for the shelf wall's own top.
    VIEW_HEIGHT_M = max(ELEVATOR_MAX_HEIGHT_M + ARM_LENGTH_M + 0.3, WALL_TOP_Z + 0.3)
    VIEW_WIDTH_M = VIEW_WORLD_X_MAX - VIEW_WORLD_X_MIN

    def __init__(self, parent=None):
        super().__init__(parent)
        self.snapshot = MechanismSnapshot()
        self.setMinimumSize(600, 400)
        self.setStyleSheet(f"background-color: {theme.BG_DEEP};")

    def _scale(self) -> float:
        w = max(self.width() - 2 * self.MARGIN, 1)
        h = max(self.height() - 2 * self.MARGIN, 1)
        return min(w / self.VIEW_WIDTH_M, h / self.VIEW_HEIGHT_M)

    def _to_px(self, x_m: float, z_m: float, scale: float) -> QtCore.QPointF:
        """World (x=horizontal, z=height above ground, meters) -> pixels.
        A fixed world frame, not centered on the chassis -- once the field
        has fixed landmarks (the cube's start spot, the shelf) those need
        to stay put rather than scrolling with the robot."""
        px = self.width() / 2 + (x_m - (VIEW_WORLD_X_MIN + VIEW_WORLD_X_MAX) / 2) * scale
        py = self.height() - self.MARGIN - z_m * scale
        return QtCore.QPointF(px, py)

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        scale = self._scale()
        snap = self.snapshot

        ground_y = self._to_px(0.0, 0.0, scale).y()
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 2))
        painter.drawLine(QtCore.QPointF(0, ground_y), QtCore.QPointF(self.width(), ground_y))

        self._draw_shelf(painter, scale, colliding=snap.gripper_colliding)

        chassis_top_m = CHASSIS_HEIGHT_M
        chassis_center = self._to_px(snap.chassis_x_m, chassis_top_m / 2, scale)
        chassis_w_px = CHASSIS_WIDTH_M * scale
        chassis_h_px = CHASSIS_HEIGHT_M * scale
        chassis_rect = QtCore.QRectF(
            chassis_center.x() - chassis_w_px / 2, chassis_center.y() - chassis_h_px / 2,
            chassis_w_px, chassis_h_px,
        )
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 2))
        painter.setBrush(QtGui.QColor(theme.ACCENT_CYAN_DIM))
        painter.drawRect(chassis_rect)

        elevator_pen = QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 6)
        elevator_pen.setCapStyle(Qt.FlatCap)
        painter.setPen(elevator_pen)
        elevator_base = self._to_px(snap.chassis_x_m, chassis_top_m, scale)
        elevator_top = self._to_px(snap.chassis_x_m, chassis_top_m + snap.elevator_height_m, scale)
        painter.drawLine(elevator_base, elevator_top)

        arm_color = theme.ACCENT_RED if snap.gripper_colliding else theme.ACCENT_AMBER
        arm_pen = QtGui.QPen(QtGui.QColor(arm_color), ARM_THICKNESS_M * scale)
        # Flat, not round: the collision check models the link as a rectangle of
        # this thickness, so rounded ends would draw material the robot does not
        # check for -- and the point of this view is that what you see is what is
        # tested.
        arm_pen.setCapStyle(Qt.FlatCap)
        painter.setPen(arm_pen)
        arm_origin_m = chassis_top_m + snap.elevator_height_m
        arm_tip_x = snap.chassis_x_m + ARM_LENGTH_M * math.cos(snap.arm_angle_rad)
        arm_tip_z = arm_origin_m + ARM_LENGTH_M * math.sin(snap.arm_angle_rad)
        arm_origin_px = self._to_px(snap.chassis_x_m, arm_origin_m, scale)
        arm_tip_px = self._to_px(arm_tip_x, arm_tip_z, scale)
        painter.drawLine(arm_origin_px, arm_tip_px)

        self._draw_piece(painter, scale, snap)
        self._draw_hud(painter, snap)

    def _draw_shelf(self, painter: QtGui.QPainter, scale: float, *, colliding: bool) -> None:
        """The wall as three solid segments, with the LOW/HIGH slot bands
        left open between them -- drawn as gaps rather than as a filled
        wall with holes cut out, since QPainter has no boolean-subtract
        primitive and three rects reads just as clearly."""
        solid_bands = [
            (0.0, LOW_SLOT_Z[0]),
            (LOW_SLOT_Z[1], HIGH_SLOT_Z[0]),
            (HIGH_SLOT_Z[1], WALL_TOP_Z),
        ]
        wall_color = QtGui.QColor(theme.ACCENT_RED if colliding else theme.TEXT_DIM)
        painter.setPen(Qt.NoPen)
        painter.setBrush(wall_color)
        for z_min, z_max in solid_bands:
            top_left = self._to_px(WALL_FAR_X, z_max, scale)
            bottom_right = self._to_px(WALL_NEAR_X, z_min, scale)
            painter.drawRect(QtCore.QRectF(top_left, bottom_right))

        # A faint outline bracket around each slot opening so it reads as
        # "the target", not just empty space between two wall segments.
        outline_pen = QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN_DIM), 1, Qt.DashLine)
        painter.setPen(outline_pen)
        painter.setBrush(Qt.NoBrush)
        for z_min, z_max in (LOW_SLOT_Z, HIGH_SLOT_Z):
            top_left = self._to_px(WALL_FAR_X, z_max, scale)
            bottom_right = self._to_px(WALL_NEAR_X, z_min, scale)
            painter.drawRect(QtCore.QRectF(top_left, bottom_right))

    def _draw_piece(self, painter: QtGui.QPainter, scale: float, snap: MechanismSnapshot) -> None:
        if snap.piece_scored:
            color = "#3ddc84"  # success green -- distinct from the palette's other accents
        elif snap.holding_piece:
            color = theme.ACCENT_AMBER
        else:
            color = theme.TEXT_PRIMARY
        center = self._to_px(snap.piece_x_m, snap.piece_z_m, scale)
        size_px = PIECE_SIZE_M * scale
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        painter.setBrush(QtGui.QColor(color).darker(160))
        # Drawn at its carry angle, because that is the footprint the robot's own
        # collision check uses -- a level box here would show clearance through a
        # slot that a tilted piece does not actually have. Negated because screen
        # Y grows downward while world Z grows up, so a world-CCW rotation is a
        # screen-CW one.
        painter.save()
        painter.translate(center)
        painter.rotate(-math.degrees(snap.piece_angle_rad))
        painter.drawRect(QtCore.QRectF(-size_px / 2, -size_px / 2, size_px, size_px))
        painter.restore()

    def _draw_hud(self, painter: QtGui.QPainter, snap: MechanismSnapshot) -> None:
        painter.setFont(theme.technical_font(11))
        status = "LIVE" if snap.connected else "NOT CONNECTED"
        painter.setPen(QtGui.QColor(theme.ACCENT_CYAN if snap.connected else theme.ACCENT_RED))
        painter.drawText(10, 20, status)

        if snap.gripper_colliding:
            painter.setPen(QtGui.QColor(theme.ACCENT_RED))
            painter.drawText(10, 38, "COLLISION")

        painter.setPen(QtGui.QColor(theme.TEXT_PRIMARY))
        if snap.piece_scored:
            piece_status = "scored"
        elif snap.holding_piece:
            piece_status = "held"
        else:
            piece_status = "on field"
        lines = [
            f"elevator: {snap.elevator_height_m * 1000:.0f} mm",
            f"arm: {math.degrees(snap.arm_angle_rad):.1f} deg",
            f"chassis x: {snap.chassis_x_m:.2f} m",
            f"piece: {piece_status}",
        ]
        for i, line in enumerate(lines):
            painter.drawText(10, 58 + i * 16, line)

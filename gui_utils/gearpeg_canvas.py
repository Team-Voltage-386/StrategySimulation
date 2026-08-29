"""
QPainter side-view (X-Z plane) of the mechanism-orchestration sandbox's
gear-peg demo -- a fixed-pivot arm plus wrist picking a gear off the floor
and threading it, standing vertical, onto one of two peg posts, driven live
over NT4 from the Java robot sim (see MechanismOrchestrationSandbox, a
sibling repo, frc.robot.gearpeg.*, and
common_sim/telemetry/nt4_client.py).

Structurally this is MechanismCanvas's twin -- same fixed-world-frame
_scale()/_to_px() machinery, same HUD approach -- copied rather than shared
because the two demos' drawables genuinely differ: no elevator segment here
(the arm originates from a fixed pivot height instead), no wall (two peg
posts instead of a slotted wall), and a non-square piece (long side/short
side, not the cube's PIECE_SIZE_M square) whose *shape* is part of what
makes "standing up on the peg" visually legible.
"""
from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from common_sim.telemetry.nt4_client import MechanismSnapshot
from gui_utils import theme

Qt = QtCore.Qt

# Must match the Java project's physical constants
# (frc.robot.gearpeg.config.ArmWristConfig.DEFAULT) -- there's no shared
# source of truth across the language boundary, so these are duplicated by
# hand, the same way mechanism_canvas.py already does for cube_shelf's own
# constants.
ARM_LENGTH_M = 0.80
PIVOT_HEIGHT_M = 0.60
ARM_THICKNESS_M = 0.06
CHASSIS_WIDTH_M = 0.80
CHASSIS_HEIGHT_M = 0.20

# Must match frc.robot.gearpeg.game.Peg.
PEG_X = -1.30
LOW_PEG_Z = 0.45
HIGH_PEG_Z = 1.05
# Height of the drawn peg post stub above its base -- a drawing constant
# only, not a Java-side dimension (Peg has no collision geometry to mirror).
PEG_STUB_HEIGHT_M = 0.12

# Must match frc.robot.gearpeg.game.GearPegScenario. The layout is one
# monotonic drive from right to left -- PEG_X < GEAR_START_X <
# ROBOT_START_X -- because the arm always points left (reflected
# asin/cos solution, cos(angle) < 0) for both the pickup and the peg
# reach, so the chassis always sits to the right of whatever the arm is
# reaching for. See GearPegScenario's class doc for the full picture.
GEAR_START_X = 0.60
GEAR_START_Z = 0.30
ROBOT_START_X = 2.40

# Must match frc.robot.gearpeg.config.ArmWristConfig.DEFAULT.
GEAR_LONG_SIDE_M = 0.24
GEAR_SHORT_SIDE_M = 0.10

# Must match frc.robot.gearpeg.config.ArmWristConfig.DEFAULT.wristLengthM --
# unlike the earlier drawing-only indicator length this replaced, the wrist
# now has a real kinematic reach (see GearPegScenario.goalsFor and
# GearGripper.tipX/tipZ), and the gear's own held/scored position already
# reflects it -- so the indicator has to be drawn at the same length or it
# would visually end somewhere other than where the piece actually sits.
WRIST_INDICATOR_LENGTH_M = 0.12
WRIST_INDICATOR_CROSSBAR_M = 0.06

# Fixed world span shown at once -- wide enough to keep the robot's start
# pose, the gear's pickup spot, and the peg post all on screen together,
# with margin on both sides (mirrors MechanismCanvas's own
# VIEW_WORLD_X_MIN/MAX choice). ROBOT_START_X = 2.40 is the rightmost
# landmark now (previously GEAR_START_X = 2.2 was), so the old upper
# bound of 3.0 no longer has as much margin -- widened to 3.2.
VIEW_WORLD_X_MIN = -2.0
VIEW_WORLD_X_MAX = 3.2


class GearPegCanvas(QtWidgets.QWidget):
    MARGIN = 40
    # Vertical span the view draws, in meters above the ground -- tall
    # enough for the pivot height plus the arm swung fully vertical, and
    # for the high peg's stub.
    VIEW_HEIGHT_M = max(PIVOT_HEIGHT_M + ARM_LENGTH_M + 0.3, HIGH_PEG_Z + PEG_STUB_HEIGHT_M + 0.3)
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
        Same fixed-world-frame approach as MechanismCanvas._to_px -- the
        gear's start spot and the peg post are fixed landmarks that need to
        stay put rather than scrolling with the robot."""
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

        self._draw_pegs(painter, scale, colliding=snap.gripper_colliding)

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

        # No elevator segment: the arm originates straight from the fixed
        # pivot height above the chassis, unlike MechanismCanvas's
        # chassis-top-plus-elevator-height origin.
        arm_color = theme.ACCENT_RED if snap.gripper_colliding else theme.ACCENT_AMBER
        arm_pen = QtGui.QPen(QtGui.QColor(arm_color), ARM_THICKNESS_M * scale)
        arm_pen.setCapStyle(Qt.FlatCap)
        painter.setPen(arm_pen)
        arm_tip_x = snap.chassis_x_m + ARM_LENGTH_M * math.cos(snap.arm_angle_rad)
        arm_tip_z = PIVOT_HEIGHT_M + ARM_LENGTH_M * math.sin(snap.arm_angle_rad)
        arm_origin_px = self._to_px(snap.chassis_x_m, PIVOT_HEIGHT_M, scale)
        arm_tip_px = self._to_px(arm_tip_x, arm_tip_z, scale)
        painter.drawLine(arm_origin_px, arm_tip_px)

        self._draw_wrist(painter, scale, snap, arm_tip_x, arm_tip_z)
        self._draw_piece(painter, scale, snap)
        self._draw_hud(painter, snap)

    def _draw_wrist(
        self,
        painter: QtGui.QPainter,
        scale: float,
        snap: MechanismSnapshot,
        arm_tip_x: float,
        arm_tip_z: float,
    ) -> None:
        """A short indicator continuing past the arm tip at the *absolute*
        carry angle (arm + wrist, matching PieceAngleRad/piece_angle_rad's
        own convention), with a small crossbar at its end so the rotation
        reads at a glance rather than needing the HUD's numeric readout.
        Drawn every frame regardless of whether a piece is held -- unlike
        the piece itself, which only exists once grabbed, the wrist's own
        rotation is live from the moment the sim starts, and watching it
        settle onto the scoring angle *before* the drive reaches the peg
        (see ScoreOnPeg's two-phase approach) is exactly the kind of timing
        this view exists to make visible."""
        absolute_angle = snap.arm_angle_rad + snap.wrist_angle_rad
        tip_px = self._to_px(arm_tip_x, arm_tip_z, scale)
        end_x = arm_tip_x + WRIST_INDICATOR_LENGTH_M * math.cos(absolute_angle)
        end_z = arm_tip_z + WRIST_INDICATOR_LENGTH_M * math.sin(absolute_angle)
        end_px = self._to_px(end_x, end_z, scale)

        pen = QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 3)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(tip_px, end_px)

        # Crossbar perpendicular to the indicator, at its far end -- reads as
        # the gripper's own face rather than just a second, thinner arm link.
        painter.save()
        painter.translate(end_px)
        painter.rotate(-math.degrees(absolute_angle))
        half_px = WRIST_INDICATOR_CROSSBAR_M / 2 * scale
        painter.drawLine(QtCore.QPointF(0, -half_px), QtCore.QPointF(0, half_px))
        painter.restore()

    def _draw_pegs(self, painter: QtGui.QPainter, scale: float, *, colliding: bool) -> None:
        """Each peg as a short vertical stub sticking up from a base, plus a
        dashed bracket outline the way _draw_shelf's slot brackets read as
        "the target" -- there's no wall here to draw solid material for, so
        the post itself is the only landmark."""
        post_color = QtGui.QColor(theme.ACCENT_RED if colliding else theme.TEXT_DIM)
        pen = QtGui.QPen(post_color, 4)
        pen.setCapStyle(Qt.FlatCap)
        painter.setPen(pen)
        for peg_z, label in ((LOW_PEG_Z, "LOW"), (HIGH_PEG_Z, "HIGH")):
            base_px = self._to_px(PEG_X, peg_z - PEG_STUB_HEIGHT_M / 2, scale)
            top_px = self._to_px(PEG_X, peg_z + PEG_STUB_HEIGHT_M / 2, scale)
            painter.drawLine(base_px, top_px)

            outline_pen = QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN_DIM), 1, Qt.DashLine)
            painter.setPen(outline_pen)
            painter.setBrush(Qt.NoBrush)
            half = PEG_STUB_HEIGHT_M
            top_left = self._to_px(PEG_X - half, peg_z + half, scale)
            bottom_right = self._to_px(PEG_X + half, peg_z - half, scale)
            painter.drawRect(QtCore.QRectF(top_left, bottom_right))
            painter.setPen(QtGui.QColor(theme.TEXT_DIM))
            painter.setFont(theme.technical_font(9))
            painter.drawText(top_left + QtCore.QPointF(2, -4), label)
            painter.setPen(pen)

    def _draw_piece(self, painter: QtGui.QPainter, scale: float, snap: MechanismSnapshot) -> None:
        if snap.piece_scored:
            color = "#3ddc84"  # success green -- matches MechanismCanvas's own choice
        elif snap.holding_piece:
            color = theme.ACCENT_AMBER
        else:
            color = theme.TEXT_PRIMARY
        center = self._to_px(snap.piece_x_m, snap.piece_z_m, scale)
        long_px = GEAR_LONG_SIDE_M * scale
        short_px = GEAR_SHORT_SIDE_M * scale
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        painter.setBrush(QtGui.QColor(color).darker(160))
        # Drawn at its carry angle, same reasoning and same screen-vs-world
        # rotation sign flip as MechanismCanvas._draw_piece. Long side along
        # the piece's own X so angleRad=0 (resting flat on the floor) draws
        # it lying down, matching Gear's actual resting orientation.
        painter.save()
        painter.translate(center)
        painter.rotate(-math.degrees(snap.piece_angle_rad))
        painter.drawRect(QtCore.QRectF(-long_px / 2, -short_px / 2, long_px, short_px))
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
            f"arm: {math.degrees(snap.arm_angle_rad):.1f} deg",
            f"wrist: {math.degrees(snap.wrist_angle_rad):.1f} deg",
            f"chassis x: {snap.chassis_x_m:.2f} m",
            f"piece: {piece_status}",
        ]
        for i, line in enumerate(lines):
            painter.drawText(10, 58 + i * 16, line)

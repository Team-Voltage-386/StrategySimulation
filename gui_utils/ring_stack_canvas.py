"""
QPainter side-view (X-Z plane) of the mechanism-orchestration sandbox's
ring-stack demo -- a fixed-angle telescoping arm plus wrist picking a ring
off its stand and threading it, lying flat, onto the next open slot of a
single vertical pole, driven live over NT4 from the Java robot sim (see
MechanismOrchestrationSandbox, a sibling repo, frc.robot.ringstack.*, and
common_sim/telemetry/mechanism_snapshot.py).

Structurally this is GearPegCanvas's cousin -- same fixed-world-frame
_scale()/_to_px() machinery, same HUD approach -- copied rather than shared
because the two demos' drawables genuinely differ: the arm here extends
along a *fixed* angle rather than sweeping through a variable one, there is
one pole with a growing stack instead of two independent peg posts, and the
piece must arrive *flat* rather than vertical to score.
"""
from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from common_sim.telemetry.mechanism_snapshot import MechanismSnapshot
from gui_utils import theme

Qt = QtCore.Qt

# Must match the Java project's physical constants
# (frc.robot.ringstack.config.RingStackConfig.DEFAULT) -- there's no shared
# source of truth across the language boundary, so these are duplicated by
# hand, the same way gearpeg_canvas.py already does for its own constants.
MOUNT_ANGLE_RAD = math.radians(120.0)
PIVOT_HEIGHT_M = 0.30
# The boom's pivot is not mounted at the chassis's own reference point (chassis_x_m) -- it's
# offset further toward -X, the same direction the boom already points, so the chassis body can
# sit further from the pole than the pivot itself does. See RingStackConfig.DEFAULT's "Why the
# pivot is also offset" doc.
ARM_MOUNT_OFFSET_X_M = -0.15
ARM_MIN_LENGTH_M = 0.25
ARM_MAX_LENGTH_M = 1.05
ARM_THICKNESS_M = 0.06
CHASSIS_WIDTH_M = 0.80
CHASSIS_HEIGHT_M = 0.20

# Must match frc.robot.ringstack.game.Pole. POLE_TOP_Z is the pole's real physical top now (a
# ring must clear it before it's safe to bring toward the pole's X -- see Pole's own class doc),
# not just a cosmetic drawing headroom the way gearpeg_canvas.py's PEG_STUB_HEIGHT_M is.
POLE_X = -1.30
BASE_SLOT_Z = 0.60
SLOT_SPACING_M = 0.07
MAX_RINGS = 5
TOP_CLEARANCE_M = 0.12
POLE_TOP_Z = BASE_SLOT_Z + (MAX_RINGS - 1) * SLOT_SPACING_M + TOP_CLEARANCE_M

# Must match frc.robot.ringstack.game.RingStackScenario.
RING_START_X = 0.60
RING_START_Z = 0.75
ROBOT_START_X = 1.80
HOVER_CLEARANCE_M = 0.12
HOVER_Z = POLE_TOP_Z + HOVER_CLEARANCE_M

# Must match frc.robot.ringstack.config.RingStackConfig.DEFAULT.
WRIST_LENGTH_M = 0.12
RING_OUTER_RADIUS_M = 0.18
RING_TUBE_RADIUS_M = 0.035
WRIST_INDICATOR_CROSSBAR_M = 0.06

# Fixed world span shown at once -- wide enough to keep the robot's start
# pose, the ring's pickup stand, and the pole all on screen together, with
# margin on both sides.
VIEW_WORLD_X_MIN = -2.0
VIEW_WORLD_X_MAX = 2.6


class RingStackCanvas(QtWidgets.QWidget):
    MARGIN = 40
    # Vertical span the view draws, in meters above the ground -- tall
    # enough for the pivot height plus the arm fully extended, and for the
    # pole's own top.
    VIEW_HEIGHT_M = max(
        PIVOT_HEIGHT_M + ARM_MAX_LENGTH_M * math.sin(MOUNT_ANGLE_RAD) + WRIST_LENGTH_M + 0.3,
        POLE_TOP_Z + 0.3,
    )
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
        Same fixed-world-frame approach as MechanismCanvas/GearPegCanvas --
        the ring's stand and the pole are fixed landmarks that need to stay
        put rather than scrolling with the robot."""
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

        self._draw_pole(painter, scale, snap, colliding=snap.gripper_colliding)

        chassis_center = self._to_px(snap.chassis_x_m, CHASSIS_HEIGHT_M / 2, scale)
        chassis_w_px = CHASSIS_WIDTH_M * scale
        chassis_h_px = CHASSIS_HEIGHT_M * scale
        chassis_rect = QtCore.QRectF(
            chassis_center.x() - chassis_w_px / 2, chassis_center.y() - chassis_h_px / 2,
            chassis_w_px, chassis_h_px,
        )
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 2))
        painter.setBrush(QtGui.QColor(theme.ACCENT_CYAN_DIM))
        painter.drawRect(chassis_rect)

        # No swinging pivot -- the boom always leaves the mount at the same fixed angle, only its
        # length (snap.arm_length_m) changes, unlike MechanismCanvas/GearPegCanvas's variable-angle arms.
        # The pivot itself is drawn offset from the chassis center by ARM_MOUNT_OFFSET_X_M, not
        # dead center on it -- see that constant's own doc.
        arm_color = theme.ACCENT_RED if snap.gripper_colliding else theme.ACCENT_AMBER
        arm_pen = QtGui.QPen(QtGui.QColor(arm_color), ARM_THICKNESS_M * scale)
        arm_pen.setCapStyle(Qt.FlatCap)
        painter.setPen(arm_pen)
        pivot_x = snap.chassis_x_m + ARM_MOUNT_OFFSET_X_M
        arm_tip_x = pivot_x + snap.arm_length_m * math.cos(MOUNT_ANGLE_RAD)
        arm_tip_z = PIVOT_HEIGHT_M + snap.arm_length_m * math.sin(MOUNT_ANGLE_RAD)
        arm_origin_px = self._to_px(pivot_x, PIVOT_HEIGHT_M, scale)
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
        """Same indicator-plus-crossbar treatment as GearPegCanvas._draw_wrist -- a short
        indicator continuing past the arm tip at the *absolute* carry angle (mount angle plus
        wrist, matching PieceAngleRad's own convention once a ring is held), with a crossbar so
        the rotation reads at a glance."""
        absolute_angle = MOUNT_ANGLE_RAD + snap.wrist_angle_rad
        tip_px = self._to_px(arm_tip_x, arm_tip_z, scale)
        end_x = arm_tip_x + WRIST_LENGTH_M * math.cos(absolute_angle)
        end_z = arm_tip_z + WRIST_LENGTH_M * math.sin(absolute_angle)
        end_px = self._to_px(end_x, end_z, scale)

        pen = QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 3)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(tip_px, end_px)

        painter.save()
        painter.translate(end_px)
        painter.rotate(-math.degrees(absolute_angle))
        half_px = WRIST_INDICATOR_CROSSBAR_M / 2 * scale
        painter.drawLine(QtCore.QPointF(0, -half_px), QtCore.QPointF(0, half_px))
        painter.restore()

    def _draw_pole(
        self, painter: QtGui.QPainter, scale: float, snap: MechanismSnapshot, *, colliding: bool
    ) -> None:
        """The pole itself as a vertical post, plus one dashed bracket per slot -- filled and
        labeled with its stack index once a ring has landed there (drawn purely from
        snap.rings_on_pole, since the Java side only tracks a count, not each historical ring's
        own position -- see Pole's class doc), dashed-only while still open. The next open slot
        (the one an in-flight ScoreRing is actually driving to) is called out separately so a
        viewer can see where the *next* ring is headed, not just how many have landed."""
        post_color = QtGui.QColor(theme.ACCENT_RED if colliding else theme.TEXT_DIM)
        pen = QtGui.QPen(post_color, 4)
        pen.setCapStyle(Qt.FlatCap)
        painter.setPen(pen)
        base_px = self._to_px(POLE_X, 0.0, scale)
        top_px = self._to_px(POLE_X, POLE_TOP_Z, scale)
        painter.drawLine(base_px, top_px)

        for i in range(MAX_RINGS):
            slot_z = BASE_SLOT_Z + i * SLOT_SPACING_M
            filled = i < snap.rings_on_pole
            is_next = i == snap.rings_on_pole
            if filled:
                self._draw_ring_rect(painter, scale, POLE_X, slot_z, 0.0, "#3ddc84")
            outline_color = theme.ACCENT_AMBER if is_next else theme.ACCENT_CYAN_DIM
            outline_pen = QtGui.QPen(QtGui.QColor(outline_color), 1, Qt.DashLine)
            painter.setPen(outline_pen)
            painter.setBrush(Qt.NoBrush)
            half = RING_OUTER_RADIUS_M
            top_left = self._to_px(POLE_X - half, slot_z + RING_TUBE_RADIUS_M, scale)
            bottom_right = self._to_px(POLE_X + half, slot_z - RING_TUBE_RADIUS_M, scale)
            painter.drawRect(QtCore.QRectF(top_left, bottom_right))

        painter.setPen(QtGui.QColor(theme.TEXT_DIM))
        painter.setFont(theme.technical_font(9))
        label_px = self._to_px(POLE_X, POLE_TOP_Z, scale)
        painter.drawText(label_px + QtCore.QPointF(6, 0), f"{snap.rings_on_pole}/{MAX_RINGS}")

    def _draw_ring_rect(
        self, painter: QtGui.QPainter, scale: float, x_m: float, z_m: float, angle_rad: float, color: str
    ) -> None:
        """A ring drawn side-on as a flat oriented rectangle -- long side the outer diameter,
        short side the tube diameter -- the same rotated-rectangle technique GearPegCanvas uses
        for its own (differently-shaped) piece. angleRad=0 draws it lying flat (long side
        horizontal); +-90 degrees draws it standing on edge."""
        center = self._to_px(x_m, z_m, scale)
        long_px = 2 * RING_OUTER_RADIUS_M * scale
        short_px = 2 * RING_TUBE_RADIUS_M * scale
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        painter.setBrush(QtGui.QColor(color).darker(160))
        painter.save()
        painter.translate(center)
        painter.rotate(-math.degrees(angle_rad))
        painter.drawRect(QtCore.QRectF(-long_px / 2, -short_px / 2, long_px, short_px))
        painter.restore()

    def _draw_piece(self, painter: QtGui.QPainter, scale: float, snap: MechanismSnapshot) -> None:
        if snap.piece_scored:
            color = "#3ddc84"  # success green -- matches the other two canvases' choice
        elif snap.holding_piece:
            color = theme.ACCENT_AMBER
        else:
            color = theme.TEXT_PRIMARY
        self._draw_ring_rect(painter, scale, snap.piece_x_m, snap.piece_z_m, snap.piece_angle_rad, color)

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
            f"arm length: {snap.arm_length_m:.2f} m",
            f"wrist: {math.degrees(snap.wrist_angle_rad):.1f} deg",
            f"chassis x: {snap.chassis_x_m:.2f} m",
            f"piece: {piece_status}",
            f"rings on pole: {snap.rings_on_pole}/{MAX_RINGS}",
        ]
        for i, line in enumerate(lines):
            painter.drawText(10, 58 + i * 16, line)

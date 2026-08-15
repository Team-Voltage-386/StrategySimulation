"""
QPainter-based live render of a Match: field boundary, obstacles,
scoring regions, game pieces, and robots. Game-agnostic -- it only
reads common_sim's generic FieldConfig/Robot/GamePiece state, so it
works unmodified for any game_specific package. Renders in the sci-fi
theme's palette so it composes with theme.py/overlay_panel.py. See
apps/run_match.py for how it's wired into a window with keyboard input.
"""
from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from common_sim.field.field_config import point_in_polygon, polygon_centroid
from common_sim.match.match import Match
from gui_utils import theme

Qt = QtCore.Qt

# Colors for the "will this score" region highlight -- green when a robot
# sitting in the zone right now, with its selected action and held piece
# type, would score; red when a robot's in the zone but its selection/piece
# wouldn't score there; the region's normal amber otherwise.
VALID_FILL = QtGui.QColor(60, 220, 120, 70)
VALID_OUTLINE = QtGui.QColor(60, 220, 120)
INVALID_FILL = QtGui.QColor(255, 77, 77, 40)

ALLIANCE_COLORS = {"red": QtGui.QColor(220, 30, 60), "blue": QtGui.QColor(theme.ACCENT_CYAN)}

# Intake locations (feeder/loading stations) get their own outline color,
# distinct from scoring regions' amber, so the two kinds of zone read as
# different things at a glance; they light up in the occupying robot's
# alliance color the same way scoring regions do.
INTAKE_OUTLINE = QtGui.QColor(190, 110, 240)
INTAKE_FILL = QtGui.QColor(190, 110, 240, 30)

# Reef-face scoring regions offer several stacked levels (l1..l4) at one
# physical zone -- l2-l4 are drawn as a small grid of per-branch squares
# rather than a single number, matching how each level has a fixed number
# of physical branches a piece can occupy. This couples the canvas to
# REEFSCAPE's action-naming convention; harmless while it's the only
# game_specific package plugged in, but would need generalizing (e.g. a
# display-capacity hint on ScoringRegion) for a second game.
REEF_GRID_LEVELS = ("l4", "l3", "l2")
REEF_GRID_SLOTS_PER_LEVEL = 2

# Outward normal of each robot-relative side *in the QPainter-local space*
# already established by _draw_one_robot's translate()+rotate(-heading) --
# not the same sign convention as common_sim's world-frame _SIDE_OUTWARD,
# because the widget's y axis points down while the field's world y axis
# points up (see field_canvas's _to_widget). Derived so that "front" lands
# on the same edge as the existing heading line (drawn to (+half_l, 0)).
SIDE_NORMAL_LOCAL = {"front": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, -1.0), "right": (0.0, 1.0)}


def _types_label(piece_types: frozenset[str]) -> str:
    return ",".join(t[:1].upper() for t in sorted(piece_types))


class FieldCanvas(QtWidgets.QWidget):
    MARGIN = 20

    def __init__(self, match: Match, parent=None):
        super().__init__(parent)
        self.match = match
        self.setMinimumSize(400, 300)
        self.setStyleSheet(f"background-color: {theme.BG_DEEP};")
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)

    def _field_scale(self) -> float:
        w = max(self.width() - 2 * self.MARGIN, 1)
        h = max(self.height() - 2 * self.MARGIN, 1)
        return min(w / self.match.field.width, h / self.match.field.height)

    def _to_widget(self, x: float, y: float, scale: float) -> tuple[float, float]:
        # Field origin is bottom-left with +y up; widget origin is
        # top-left with +y down.
        return self.MARGIN + x * scale, self.height() - self.MARGIN - y * scale

    def _to_field(self, wx: float, wy: float, scale: float) -> tuple[float, float]:
        """Inverse of _to_widget -- widget-local pixel coords back to field
        coords, for hit-testing under the mouse cursor."""
        return (wx - self.MARGIN) / scale, (self.height() - self.MARGIN - wy) / scale

    def mouseMoveEvent(self, event) -> None:
        scale = self._field_scale()
        pos = event.position() if hasattr(event, "position") else event.pos()
        global_pos = event.globalPosition().toPoint() if hasattr(event, "globalPosition") else event.globalPos()
        field_point = self._to_field(pos.x(), pos.y(), scale)
        for region in self.match.field.scoring_regions:
            if point_in_polygon(field_point, region.vertices):
                QtWidgets.QToolTip.showText(global_pos, region.name, self)
                return
        QtWidgets.QToolTip.hideText()

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        # Explicit fill rather than relying on the QSS background-color
        # applying -- a bare QWidget subclass with its own paintEvent does
        # not auto-paint its stylesheet background unless WA_StyledBackground
        # is set, so this keeps rendering correct even if a host app never
        # calls theme.apply_app_theme().
        painter.fillRect(self.rect(), QtGui.QColor(theme.BG_DEEP))
        scale = self._field_scale()

        painter.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 2))
        x0, y0 = self._to_widget(0, self.match.field.height, scale)
        x1, y1 = self._to_widget(self.match.field.width, 0, scale)
        painter.drawRect(QtCore.QRectF(x0, y0, x1 - x0, y1 - y0))

        self._draw_scoring_regions(painter, scale)
        self._draw_intake_locations(painter, scale)
        self._draw_obstacles(painter, scale)
        self._draw_region_piece_counts(painter, scale)
        self._draw_pieces(painter, scale)
        self._draw_robots(painter, scale)
        self._draw_hud(painter)
        painter.end()

    def _polygon(self, vertices, scale: float) -> QtGui.QPolygonF:
        poly = QtGui.QPolygonF()
        for vx, vy in vertices:
            wx, wy = self._to_widget(vx, vy, scale)
            poly.append(QtCore.QPointF(wx, wy))
        return poly

    def _draw_scoring_regions(self, painter, scale: float) -> None:
        for region in self.match.field.scoring_regions:
            status = self._region_highlight(region)
            if status == "valid":
                painter.setPen(QtGui.QPen(VALID_OUTLINE, 3))
                painter.setBrush(VALID_FILL)
            elif status == "invalid":
                painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_RED), 3))
                painter.setBrush(INVALID_FILL)
            else:
                painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_AMBER), 2))
                painter.setBrush(QtGui.QColor(255, 176, 32, 30))
            painter.drawPolygon(self._polygon(region.vertices, scale))

    def _region_highlight(self, region) -> str | None:
        """'valid' if some robot sitting in `region` right now, with its
        selected deposit action and held piece type, would score there;
        'invalid' if a robot's in the zone holding a piece but its
        selection/piece type wouldn't score; None otherwise."""
        status = None
        for robot in self.match.robots:
            robot_status = self._scoring_status_for(robot, region)
            if robot_status == "valid":
                return "valid"
            if robot_status == "invalid":
                status = "invalid"
        return status

    def _scoring_status_for(self, robot, region) -> str | None:
        """'valid'/'invalid'/None for this one robot against this one
        region -- the single-robot check `_region_highlight` (drawing the
        zone itself) and `_robot_scoring_status` (drawing the marker
        pinned to a specific robot, since the robot's own footprint can
        cover the zone it's standing in) both build on.

        Deliberately asks the robot's *scoring side* whether it reaches
        this region, and defers the valid/invalid call to
        Match.deposit_region_for -- the same check that gates the deposit
        timer. Testing the chassis center here instead (which is what
        this used to do) lit the indicator up regardless of which way the
        robot was facing, and disagreed with whether scoring would
        actually start.

        The piece tested against is whichever held piece `region` itself
        accepts, not just the FIFO-first one -- a robot holding both a
        coral and an algae needs the coral's side tested against a REEF
        face, not whichever type happened to be picked up first."""
        piece = next(
            (p for p in robot.held_pieces if not region.piece_types or p.piece_type in region.piece_types), None,
        )
        if piece is None:
            return None
        side = robot.scoring_side(piece.piece_type)
        if side is None:
            return None
        if not robot.side_engages_polygon(side, region.vertices):
            return None
        return "valid" if self.match.deposit_region_for(robot, piece) is region else "invalid"

    def _robot_scoring_status(self, robot) -> str | None:
        """Whichever scoring region `robot` currently sits in says about
        it, or None if it's not in any -- for the on-robot validity
        marker, since the zone highlight itself is often hidden under
        the robot's own sprite once it's actually in position to score."""
        for region in self.match.field.scoring_regions:
            status = self._scoring_status_for(robot, region)
            if status is not None:
                return status
        return None

    def _draw_intake_locations(self, painter, scale: float) -> None:
        for location in self.match.field.intake_locations:
            alliance = self._intake_occupant_alliance(location)
            if alliance is not None:
                painter.setPen(QtGui.QPen(ALLIANCE_COLORS.get(alliance, INTAKE_OUTLINE), 3))
                painter.setBrush(ALLIANCE_COLORS.get(alliance, INTAKE_OUTLINE).lighter(250))
            else:
                painter.setPen(QtGui.QPen(INTAKE_OUTLINE, 2))
                painter.setBrush(INTAKE_FILL)
            painter.drawPolygon(self._polygon(location.vertices, scale))

            cx, cy = polygon_centroid(location.vertices)
            wx, wy = self._to_widget(cx, cy, scale)
            remaining = self.match.station_supply.get(location)
            label = "∞" if remaining is None else str(remaining)
            self._draw_count_badge(painter, wx, wy, label)

    def _intake_occupant_alliance(self, location) -> str | None:
        """Alliance of a robot currently sitting in `location`'s zone, or
        None -- mirrors how a scoring region lights up while a robot's in
        it, so a station reads as "in use" the same way."""
        for robot in self.match.robots:
            pose = robot.pose
            if point_in_polygon((pose.x, pose.y), location.vertices):
                return robot.alliance
        return None

    def _draw_region_piece_counts(self, painter, scale: float) -> None:
        for region in self.match.field.scoring_regions:
            counts = self.match.region_scores.get(region.name, {})
            cx, cy = polygon_centroid(region.vertices)
            wx, wy = self._to_widget(cx, cy, scale)
            if set(REEF_GRID_LEVELS) & region.actions:
                alliance = "red" if region.name.startswith("red_") else "blue"
                self._draw_reef_grid(painter, wx, wy, counts, alliance)
            elif region.actions:
                self._draw_count_badge(painter, wx, wy, str(sum(counts.values())))

    def _draw_reef_grid(self, painter, cx: float, cy: float, counts: dict, alliance: str) -> None:
        cell, gap = 7, 2
        cols = REEF_GRID_SLOTS_PER_LEVEL
        rows = len(REEF_GRID_LEVELS)
        total_w = cols * cell + (cols - 1) * gap
        total_h = rows * cell + (rows - 1) * gap
        start_x = cx - total_w / 2.0
        start_y = cy - total_h / 2.0
        filled = ALLIANCE_COLORS.get(alliance, QtGui.QColor(theme.ACCENT_CYAN))
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 1))
        for r, level in enumerate(REEF_GRID_LEVELS):
            filled_count = min(counts.get(level, 0), cols)
            for c in range(cols):
                x = start_x + c * (cell + gap)
                y = start_y + r * (cell + gap)
                painter.setBrush(filled if c < filled_count else QtGui.QColor(0, 0, 0, 0))
                painter.drawRect(QtCore.QRectF(x, y, cell, cell))

    def _draw_count_badge(self, painter, cx: float, cy: float, label: str) -> None:
        r = 10
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 1))
        painter.setBrush(QtGui.QColor(theme.BG_RAISED))
        painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)
        painter.setPen(QtGui.QColor(theme.ACCENT_AMBER))
        painter.setFont(theme.technical_font(9, bold=True))
        painter.drawText(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt.AlignCenter, label)

    def _draw_obstacles(self, painter, scale: float) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 2))
        painter.setBrush(QtGui.QColor(theme.BG_RAISED))
        for obstacle in self.match.field.obstacles:
            painter.drawPolygon(self._polygon(obstacle.vertices, scale))

    def _draw_pieces(self, painter, scale: float) -> None:
        painter.setPen(Qt.NoPen)
        for piece in self.match.active_pieces:
            painter.setBrush(QtGui.QColor(piece.color) if piece.color else QtGui.QColor(255, 110, 0))
            wx, wy = self._to_widget(piece.position.x, piece.position.y, scale)
            r = max(piece.radius * scale, 2.0)
            painter.drawEllipse(QtCore.QPointF(wx, wy), r, r)

    def _draw_robots(self, painter, scale: float) -> None:
        for robot in self.match.robots:
            self._draw_one_robot(painter, robot, scale)

    def _draw_one_robot(self, painter, robot, scale: float) -> None:
        pose = robot.pose
        cx, cy = self._to_widget(pose.x, pose.y, scale)
        half_l = robot.characteristics.length / 2.0 * scale
        half_w = robot.characteristics.width / 2.0 * scale

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-math.degrees(pose.heading))

        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(theme.BG_RAISED))
        painter.drawRect(QtCore.QRectF(-half_l, -half_w, 2 * half_l, 2 * half_w))

        bumper_color = QtGui.QColor(220, 30, 60) if robot.alliance == "red" else QtGui.QColor(theme.ACCENT_CYAN)
        painter.setPen(QtGui.QPen(bumper_color, 4))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(QtCore.QRectF(-half_l, -half_w, 2 * half_l, 2 * half_w))

        painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_PRIMARY), 2))
        painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(half_l, 0))

        if robot.held_pieces:
            # Mirrors the lateral fan-out Robot.sync_held_piece_positions
            # applies to the actual held pieces, so e.g. a held coral and
            # algae show as two distinct dots rather than one on top of the
            # other. Painter y is flipped relative to the robot's local
            # (pymunk) y axis -- see SIDE_NORMAL_LOCAL above -- hence the
            # negation.
            painter.setPen(Qt.NoPen)
            count = len(robot.held_pieces)
            for i, held_piece in enumerate(robot.held_pieces):
                local_y = (i - (count - 1) / 2.0) * robot.HELD_PIECE_SPACING
                held_color = held_piece.color
                painter.setBrush(QtGui.QColor(held_color) if held_color else QtGui.QColor(255, 110, 0))
                painter.drawEllipse(QtCore.QPointF(half_l * 0.4, -local_y * scale), 5, 5)

        self._draw_side_manipulators(painter, robot, half_l, half_w)

        painter.restore()

        self._draw_action_progress(painter, robot, cx, cy, half_l, half_w)

    def _side_manipulator_tags(self, robot) -> list[tuple[str, str, str]]:
        """(side, mode, label) for every intake/score capability this
        robot has -- mode is "in"/"out", label is the piece type(s)'
        initials, or "*" for the legacy no-side-config default of
        "any type" (RobotCharacteristics.accepted_piece_types empty)."""
        chars = robot.characteristics
        if chars.side_manipulators:
            tags = []
            for side, cfg in chars.side_manipulators.items():
                if cfg.intake_piece_types:
                    tags.append((side, "in", _types_label(cfg.intake_piece_types)))
                if cfg.score_piece_types:
                    tags.append((side, "out", _types_label(cfg.score_piece_types)))
            return tags
        label = _types_label(chars.accepted_piece_types) if chars.accepted_piece_types else "*"
        return [("front", "in", label), ("front", "out", label)]

    def _draw_side_manipulators(self, painter, robot, half_l: float, half_w: float) -> None:
        """Small circular badges on each edge the robot has a manipulator
        on -- cyan for intake, amber for scoring, labeled with the piece
        type(s) that side handles. Called inside the same translate()+
        rotate(-heading) block _draw_one_robot uses for the body/bumper,
        so coordinates here are robot-local (see SIDE_NORMAL_LOCAL)."""
        for side, mode, label in self._side_manipulator_tags(robot):
            nx, ny = SIDE_NORMAL_LOCAL.get(side, (1.0, 0.0))
            tx, ty = -ny, nx  # tangent along the edge, to separate IN/OUT badges
            shift = -9 if mode == "in" else 9
            cx = nx * half_l + tx * shift
            cy = ny * half_w + ty * shift
            color = QtGui.QColor(theme.ACCENT_CYAN) if mode == "in" else QtGui.QColor(theme.ACCENT_AMBER)
            r = 6.5
            painter.setPen(QtGui.QPen(color, 1.5))
            painter.setBrush(color.darker(220))
            painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)
            painter.setPen(color)
            painter.setFont(theme.technical_font(6, bold=True))
            painter.drawText(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt.AlignCenter, label)

    def _draw_action_progress(self, painter, robot, cx: float, cy: float, half_l: float, half_w: float) -> None:
        bar_w = max(2 * half_l, 2 * half_w)
        bar_h = 5
        bar_x = cx - bar_w / 2.0
        bar_y = cy - half_w - bar_h - 6

        # Selected deposit action (e.g. which CORAL level a REEF-toggle key
        # cycles through) has no other on-canvas indication otherwise --
        # the target's only visible effect used to be a highlight color
        # that happens to be identical across l1..l4 (a REEF face accepts
        # all 4), so toggling it looked like it did nothing.
        if robot.deposit_action is not None:
            painter.setPen(QtGui.QColor(theme.TEXT_PRIMARY))
            painter.setFont(theme.technical_font(8, bold=True))
            painter.drawText(
                QtCore.QRectF(bar_x, bar_y - 13, bar_w, 12), Qt.AlignCenter, robot.deposit_action.upper(),
            )

        deposit_frac = robot.deposit_progress_fraction()
        intake_frac = robot.intake_progress_fraction()
        if deposit_frac is not None or intake_frac is not None:
            frac, color = (deposit_frac, QtGui.QColor(theme.ACCENT_AMBER)) if deposit_frac is not None \
                else (intake_frac, QtGui.QColor(theme.ACCENT_CYAN))
            painter.setPen(Qt.NoPen)
            painter.setBrush(QtGui.QColor(theme.BG_RAISED))
            painter.drawRect(QtCore.QRectF(bar_x, bar_y, bar_w, bar_h))
            painter.setBrush(color)
            painter.drawRect(QtCore.QRectF(bar_x, bar_y, bar_w * frac, bar_h))
            return

        # Not actively intaking/depositing -- if the robot is sitting in a
        # scoring zone right now, show the same bar shape empty (outline
        # only) in green/red rather than nothing, since the robot's own
        # sprite usually covers the zone highlight it would otherwise show.
        status = self._robot_scoring_status(robot)
        if status is None:
            return
        outline = VALID_OUTLINE if status == "valid" else QtGui.QColor(theme.ACCENT_RED)
        painter.setPen(QtGui.QPen(outline, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(QtCore.QRectF(bar_x, bar_y, bar_w, bar_h))

    def _draw_hud(self, painter) -> None:
        painter.setFont(theme.technical_font(11, bold=True))
        fm = painter.fontMetrics()
        x, y = 10, 20

        painter.setPen(QtGui.QColor(theme.TEXT_PRIMARY))
        header = f"t={self.match.elapsed:5.1f}s  phase={self.match.phase.value.upper()}   "
        painter.drawText(x, y, header)
        x += fm.horizontalAdvance(header)

        for alliance in ("red", "blue"):
            score = self.match.scores.get(alliance, 0.0)
            text = f"{alliance.upper()}: {score:.0f}   "
            painter.setPen(ALLIANCE_COLORS.get(alliance, QtGui.QColor(theme.TEXT_PRIMARY)))
            painter.drawText(x, y, text)
            x += fm.horizontalAdvance(text)

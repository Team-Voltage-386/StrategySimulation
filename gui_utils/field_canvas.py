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

from common_sim.control.human import HumanController
from common_sim.field.field_config import point_in_polygon, polygon_centroid
from common_sim.match.match import Match
from common_sim.robot.characteristics import SIDE_OUTWARD
from gui_utils import theme
from gui_utils.field_camera import DRIVER, FieldCamera, ViewPreset

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

# Emitter regions (spawn pieces onto the field over time) get a third,
# distinct outline color -- amber (scoring) and purple (intake) are already
# taken.
EMITTER_OUTLINE = QtGui.QColor(255, 200, 60)
EMITTER_FILL = QtGui.QColor(255, 200, 60, 30)
# Protected zones are drawn unfilled: they overlap scoring regions and
# obstacles by design, and a fill would hide what's underneath.
PROTECTED_OUTLINE = QtGui.QColor(200, 200, 210, 110)

# Reef-face scoring regions offer several stacked levels (l1..l4) at one
# physical zone -- l2-l4 are drawn as a small grid of per-branch squares
# rather than a single number, matching how each level has a fixed number
# of physical branches a piece can occupy. This couples the canvas to
# REEFSCAPE's action-naming convention; harmless while it's the only
# game_specific package plugged in, but would need generalizing (e.g. a
# display-capacity hint on ScoringRegion) for a second game.
REEF_GRID_LEVELS = ("l4", "l3", "l2")
REEF_GRID_SLOTS_PER_LEVEL = 2

# Visual-only box heights for a robot drawn under a driver-station camera --
# this sim tracks no physical robot height (RobotCharacteristics has no z
# dimension), so these are fixed stand-ins just tall enough to read as a
# solid chassis on top of a bumper, not a measurement of anything real.
ROBOT_BUMPER_HEIGHT = 5.0
ROBOT_BODY_HEIGHT = 12.0

# Height of the collect/score progress bar drawn above each robot by
# _draw_action_progress -- shared with _draw_one_robot_intent so the intent
# label can be positioned above the bar stack without overlapping it.
_BAR_H = 5


def _types_label(piece_types: frozenset[str]) -> str:
    return ",".join(t[:1].upper() for t in sorted(piece_types))


class FieldCanvas(QtWidgets.QWidget):
    MARGIN = 20

    def __init__(self, match: Match, parent=None):
        super().__init__(parent)
        self.match = match
        # Set by a host app (e.g. apps/run_reefscape.py) while scrubbed
        # into playback, to a list of telemetry PieceSnapshots (or any
        # object with position_x/position_y/radius/color) for the scrubbed
        # time -- None means "draw live from match.active_pieces" (the
        # normal case). A scored piece is permanently removed from
        # match.active_pieces (see Match.step), so it can only be redrawn
        # from its own recorded telemetry, not from a live GamePiece.
        self.playback_pieces: list | None = None
        # Set alongside playback_pieces while scrubbed into playback, to a
        # dict of robot_name -> RobotSnapshot.target_name (see
        # common_sim/match/telemetry.py) -- None means "draw live from
        # robot.intent" (the normal case). robot.intent isn't rewound on
        # scrub (it's ephemeral controller state, not physics), so without
        # this the dashed pairing line would stay frozen at whatever it was
        # when the match was paused instead of tracking the scrubbed time.
        self.playback_targets: dict[str, str | None] | None = None
        # Paired with playback_targets: robot_name -> RobotSnapshot.tactic_name,
        # for the tactic-name label drawn above each robot's pairing line.
        self.playback_tactics: dict[str, str | None] | None = None
        self.show_intent = True
        # None = today's top-down orthographic view; "blue"/"red" = that
        # alliance's driver-station perspective (see set_driver_view).
        self.driver_alliance: str | None = None
        self.view_preset: ViewPreset = DRIVER
        self._camera: FieldCamera | None = None
        self.setMinimumSize(400, 300)
        self.setStyleSheet(f"background-color: {theme.BG_DEEP};")
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)

    def set_driver_view(self, alliance: str | None, preset: ViewPreset | None = None) -> None:
        """None switches back to top-down; "blue"/"red" switches to that
        alliance's driver-station perspective. `preset` swaps the eye
        placement (see field_camera.DRIVER/ELEVATED); omitted keeps
        whichever preset was already active."""
        self.driver_alliance = alliance
        if preset is not None:
            self.view_preset = preset

    def _field_scale(self) -> float:
        w = max(self.width() - 2 * self.MARGIN, 1)
        h = max(self.height() - 2 * self.MARGIN, 1)
        return min(w / self.match.field.width, h / self.match.field.height)

    def _build_camera(self) -> FieldCamera | None:
        if self.driver_alliance is None:
            return None
        return FieldCamera(
            self.match.field.width, self.match.field.height, self.driver_alliance,
            self.width(), self.height(), self.view_preset,
        )

    def _to_widget(self, x: float, y: float, scale: float) -> tuple[float, float]:
        if self._camera is not None:
            px, py, _ = self._camera.project(x, y, 0.0)
            return px, py
        # Field origin is bottom-left with +y up; widget origin is
        # top-left with +y down.
        return self.MARGIN + x * scale, self.height() - self.MARGIN - y * scale

    def _to_field(self, wx: float, wy: float, scale: float) -> tuple[float, float]:
        """Inverse of _to_widget -- widget-local pixel coords back to field
        coords, for hit-testing under the mouse cursor. Top-down only; a
        perspective projection isn't invertible from a single screen point
        without knowing which z the ray was meant to hit (see
        mouseMoveEvent, which skips hit-testing while a camera is active)."""
        return (wx - self.MARGIN) / scale, (self.height() - self.MARGIN - wy) / scale

    def _scale_at(self, x: float, y: float, scale: float) -> float:
        """Pixels-per-inch at this field point -- `scale` unchanged in
        top-down mode, distance-dependent under a camera so a near robot
        draws larger than a far one."""
        if self._camera is not None:
            return self._camera.scale_at(x, y, 0.0)
        return scale

    def mouseMoveEvent(self, event) -> None:
        if self.driver_alliance is not None:
            # A driver-station view isn't a pointing surface -- inverting
            # a single screen point through a perspective projection needs
            # a z to resolve against, which a mouse position doesn't carry.
            QtWidgets.QToolTip.hideText()
            return
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
        self._camera = self._build_camera()
        scale = self._field_scale()

        painter.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 2))
        painter.setBrush(Qt.NoBrush)
        boundary = (
            (0, 0), (self.match.field.width, 0),
            (self.match.field.width, self.match.field.height), (0, self.match.field.height),
        )
        painter.drawPolygon(self._polygon(boundary, scale))

        self._draw_scoring_regions(painter, scale)
        self._draw_intake_locations(painter, scale)
        self._draw_emitter_regions(painter, scale)
        self._draw_protected_zones(painter, scale)
        self._draw_obstacles(painter, scale)
        self._draw_region_piece_counts(painter, scale)
        self._draw_pieces(painter, scale)
        self._draw_robots(painter, scale)
        self._draw_intent_overlay(painter, scale)
        self._draw_hud(painter)
        painter.end()

    def _polygon(self, vertices, scale: float) -> QtGui.QPolygonF:
        poly = QtGui.QPolygonF()
        for vx, vy in vertices:
            wx, wy = self._to_widget(vx, vy, scale)
            poly.append(QtCore.QPointF(wx, wy))
        return poly

    def _prism_faces(
        self, verts_2d, z0: float, z1: float, top_brush, side_brush,
        shade_sides: bool = False, include_top: bool = True,
    ) -> list:
        """Side quads + one top face of a right prism standing on
        `verts_2d` (a CCW-wound field-space polygon) from z0 to z1,
        projected through the active camera and returned back-to-front
        (painter's algorithm) as (depth, QPolygonF, brush) tuples --
        adequate for one convex/non-self-intersecting prism, since
        nothing on it can interpenetrate itself. Only meaningful with a
        camera active; callers gate on that. A side face whose outward
        normal points away from the eye is skipped outright, both saving
        a draw call and sidestepping ever having to resolve a hidden
        back face's draw order against the front faces covering it.

        `shade_sides` tints each surviving side face by how square-on it
        is to the eye (the same cosine the backface cull already computes,
        just not thrown away) -- a face dead ahead of the camera reads
        brighter than one closer to the grazing angle that would have
        gotten it culled, a cheap stand-in for directional lighting that
        makes a box read as one lit object instead of flat same-toned
        panels. Off by default so the REEF (still just two tones) doesn't
        change look by association."""
        cam = self._camera
        n = len(verts_2d)
        faces = []
        for i in range(n):
            x1, y1 = verts_2d[i]
            x2, y2 = verts_2d[(i + 1) % n]
            # Outward normal of a CCW-wound polygon: rotate the edge
            # vector -90 deg (clockwise), (ex, ey) -> (ey, -ex).
            nx, ny = y2 - y1, -(x2 - x1)
            mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            ex, ey = cam.eye[0] - mx, cam.eye[1] - my
            dot = nx * ex + ny * ey
            if dot <= 0:
                continue
            brush = side_brush
            if shade_sides:
                n_len, e_len = math.hypot(nx, ny), math.hypot(ex, ey)
                cos_angle = dot / (n_len * e_len) if n_len > 0 and e_len > 0 else 1.0
                brush = QtGui.QColor(side_brush).lighter(int(65 + 55 * cos_angle))  # ~65 (grazing) - 120 (square-on)
            p1, p2, p3, p4 = cam.project(x1, y1, z0), cam.project(x2, y2, z0), cam.project(x2, y2, z1), cam.project(x1, y1, z1)
            poly = QtGui.QPolygonF([QtCore.QPointF(p[0], p[1]) for p in (p1, p2, p3, p4)])
            faces.append(((p1[2] + p2[2] + p3[2] + p4[2]) / 4.0, poly, brush))

        if include_top:
            # False for a lower tier stacked under another prism (see
            # _draw_one_robot's bumper/frame split) -- that z1 is an
            # internal seam, not a surface anything should ever see, and
            # a real robot has no visible cap floating between its bumper
            # and its frame.
            top_pts = [cam.project(x, y, z1) for x, y in verts_2d]
            top_poly = QtGui.QPolygonF([QtCore.QPointF(p[0], p[1]) for p in top_pts])
            faces.append((sum(p[2] for p in top_pts) / n, top_poly, top_brush))

        faces.sort(key=lambda f: f[0], reverse=True)
        return faces

    def _draw_scoring_regions(self, painter, scale: float) -> None:
        for region in self.match.field.scoring_regions:
            status = self._region_highlight(region)
            if status == "valid":
                painter.setPen(QtGui.QPen(VALID_OUTLINE, 3))
                painter.setBrush(VALID_FILL)
            elif status == "invalid":
                painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_RED), 3))
                painter.setBrush(INVALID_FILL)
            elif region.alliance is not None:
                outline = ALLIANCE_COLORS.get(region.alliance, QtGui.QColor(theme.ACCENT_AMBER))
                fill = QtGui.QColor(outline)
                fill.setAlpha(30)
                painter.setPen(QtGui.QPen(outline, 2))
                painter.setBrush(fill)
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

    def _robot_station_ready(self, robot) -> bool:
        """Whether `robot` is currently positioned to trigger an intake
        location's station-dispense timer -- the collect-side mirror of
        _robot_scoring_status. Checks the same three things Match.step
        gates dispensing on (a side touching it and configured for its
        piece type, capacity left for that type, supply left at the
        station) without needing intake to actually be commanded, since
        this is a "you're in position" readiness marker, not an
        in-progress one."""
        location = robot.nearby_station()
        if location is None:
            return False
        if not robot.has_capacity_for(location.piece_type):
            return False
        remaining = self.match.station_supply.get(location)
        return remaining is None or remaining > 0

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

    def _draw_emitter_regions(self, painter, scale: float) -> None:
        for emitter in self.match.field.emitter_regions:
            outline = ALLIANCE_COLORS.get(emitter.alliance, EMITTER_OUTLINE) if emitter.alliance else EMITTER_OUTLINE
            painter.setPen(QtGui.QPen(outline, 2, Qt.DashLine))
            painter.setBrush(EMITTER_FILL)
            painter.drawPolygon(self._polygon(emitter.vertices, scale))

            cx, cy = polygon_centroid(emitter.vertices)
            wx, wy = self._to_widget(cx, cy, scale)
            remaining = self.match.emitter_capacity_remaining(emitter)
            label = "∞" if remaining is None else str(remaining)
            self._draw_count_badge(painter, wx, wy, label)

    def _draw_protected_zones(self, painter, scale: float) -> None:
        for zone in getattr(self.match.field, "protected_zones", ()):
            color = ALLIANCE_COLORS.get(zone.alliance, PROTECTED_OUTLINE) if zone.alliance else PROTECTED_OUTLINE
            pen = QtGui.QPen(color, 1, Qt.DotLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(self._polygon(zone.vertices, scale))

    def _draw_region_piece_counts(self, painter, scale: float) -> None:
        for region in self.match.field.scoring_regions:
            counts = self.match.region_scores.get(region.name, {})
            cx, cy = polygon_centroid(region.vertices)
            wx, wy = self._to_widget(cx, cy, scale)
            if set(REEF_GRID_LEVELS) & region.actions:
                alliance = region.alliance or "blue"
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
        for obstacle in self.match.field.obstacles:
            if self._camera is not None and obstacle.height > 0:
                # Semi-transparent rather than solid: a real REEF is open
                # lattice, not a wall, and the near face's fill is the
                # only thing standing between the driver and whatever
                # sits behind it on the far side (scoring zones, staged
                # ALGAE) -- the far faces themselves are already skipped
                # by _prism_faces' backface cull, so this is the one knob
                # that controls whether anything behind the structure is
                # visible at all.
                top = QtGui.QColor(theme.BG_RAISED).lighter(125)
                top.setAlpha(70)
                side = QtGui.QColor(theme.BG_RAISED).darker(140)
                side.setAlpha(90)
                for _depth, poly, brush in self._prism_faces(obstacle.vertices, 0.0, obstacle.height, top, side):
                    painter.setBrush(brush)
                    painter.drawPolygon(poly)
            else:
                painter.setBrush(QtGui.QColor(theme.BG_RAISED))
                painter.drawPolygon(self._polygon(obstacle.vertices, scale))

    def _draw_pieces(self, painter, scale: float) -> None:
        painter.setPen(Qt.NoPen)
        if self.playback_pieces is not None:
            pieces = self.playback_pieces
            if self._camera is not None:
                pieces = sorted(pieces, key=lambda p: self._camera.project(p.position_x, p.position_y, 0.0)[2], reverse=True)
            for snapshot in pieces:
                painter.setBrush(QtGui.QColor(snapshot.color) if snapshot.color else QtGui.QColor(255, 110, 0))
                wx, wy = self._to_widget(snapshot.position_x, snapshot.position_y, scale)
                r = max(snapshot.radius * self._scale_at(snapshot.position_x, snapshot.position_y, scale), 2.0)
                painter.drawEllipse(QtCore.QPointF(wx, wy), r, r)
            return
        pieces = self.match.active_pieces
        if self._camera is not None:
            pieces = sorted(pieces, key=lambda p: self._camera.project(p.position.x, p.position.y, 0.0)[2], reverse=True)
        for piece in pieces:
            painter.setBrush(QtGui.QColor(piece.color) if piece.color else QtGui.QColor(255, 110, 0))
            wx, wy = self._to_widget(piece.position.x, piece.position.y, scale)
            r = max(piece.radius * self._scale_at(piece.position.x, piece.position.y, scale), 2.0)
            painter.drawEllipse(QtCore.QPointF(wx, wy), r, r)

    def _draw_robots(self, painter, scale: float) -> None:
        robots = self.match.robots
        if self._camera is not None:
            robots = sorted(robots, key=lambda r: self._camera.project(r.pose.x, r.pose.y, 0.0)[2], reverse=True)
        for robot in robots:
            self._draw_one_robot(painter, robot, scale)

    def _robot_to_world(self, robot, local_x: float, local_y: float) -> tuple[float, float]:
        """A robot-local offset (local_x forward along heading, local_y
        left of heading -- the same front=+x/left=+y convention as
        common_sim.robot.characteristics.SIDE_OUTWARD) to a world-frame
        point."""
        pose = robot.pose
        c, s = math.cos(pose.heading), math.sin(pose.heading)
        return pose.x + local_x * c - local_y * s, pose.y + local_x * s + local_y * c

    def _rect_corners(self, robot) -> list[tuple[float, float]]:
        """World-space footprint corners, walking the rectangle boundary
        CCW (front-right, front-left, back-left, back-right) so QPolygonF
        draws the actual edges rather than a diagonal criss-cross --
        CCW specifically (not just "a consistent order") because
        _prism_faces' outward-normal formula assumes it, matching
        game_specific/reefscape/field.py's _hex_vertices. Getting this
        backwards doesn't crash: it silently swaps which faces
        backface-culling keeps, so the far side of the box draws solid
        and the near side goes missing -- wrong, but not obviously so."""
        half_l = robot.characteristics.length / 2.0
        half_w = robot.characteristics.width / 2.0
        return [
            self._robot_to_world(robot, half_l, -half_w),
            self._robot_to_world(robot, half_l, half_w),
            self._robot_to_world(robot, -half_l, half_w),
            self._robot_to_world(robot, -half_l, -half_w),
        ]

    def _draw_human_glow(self, painter, robot, cx: float, cy: float, half_extent: float) -> None:
        """Soft alliance-colored halo behind a human-piloted robot, so it
        reads at a glance which robot(s) on the field a person is
        actually driving right now versus AI. Drawn before the body/prism
        so the opaque robot paints over the glow's center."""
        radius = half_extent * 1.8
        color = QtGui.QColor(ALLIANCE_COLORS.get(robot.alliance, QtGui.QColor(theme.ACCENT_CYAN)))
        gradient = QtGui.QRadialGradient(QtCore.QPointF(cx, cy), radius)
        inner = QtGui.QColor(color)
        inner.setAlpha(160)
        outer = QtGui.QColor(color)
        outer.setAlpha(0)
        gradient.setColorAt(0.0, inner)
        gradient.setColorAt(1.0, outer)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QBrush(gradient))
        painter.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)

    def _draw_one_robot(self, painter, robot, scale: float) -> None:
        pose = robot.pose
        cx, cy = self._to_widget(pose.x, pose.y, scale)
        local_scale = self._scale_at(pose.x, pose.y, scale)
        half_l = robot.characteristics.length / 2.0
        half_w = robot.characteristics.width / 2.0
        bumper_color = QtGui.QColor(220, 30, 60) if robot.alliance == "red" else QtGui.QColor(theme.ACCENT_CYAN)

        if isinstance(robot.controller, HumanController):
            self._draw_human_glow(painter, robot, cx, cy, max(half_l, half_w) * local_scale)

        # Corners are projected explicitly (rather than the old
        # painter.translate()+rotate()) because that's affine and does not
        # survive a perspective projection -- a driver-station camera needs
        # each corner mapped through _to_widget (or, extruded, through
        # _prism_faces) on its own.
        corners = self._rect_corners(robot)
        if self._camera is not None:
            # Two stacked tiers rather than one flat-colored box: a
            # shaded alliance-colored bumper band under a neutral shaded
            # frame reads as an actual lit object with real proportions,
            # not a single flat-toned slab -- shade_sides tints each
            # surviving side face by how square-on it sits to the eye.
            frame_color = QtGui.QColor(theme.BG_RAISED).lighter(210)
            top_color = QtGui.QColor(theme.BG_RAISED).lighter(240)
            faces = self._prism_faces(
                corners, 0.0, ROBOT_BUMPER_HEIGHT, top_color, bumper_color, shade_sides=True, include_top=False,
            )
            faces += self._prism_faces(
                corners, ROBOT_BUMPER_HEIGHT, ROBOT_BODY_HEIGHT, top_color, frame_color, shade_sides=True,
            )
            faces.sort(key=lambda f: f[0], reverse=True)
            painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 1))
            for _depth, poly, brush in faces:
                painter.setBrush(brush)
                painter.drawPolygon(poly)

            # A colored rim traces the top plate's own edge on top of
            # everything else, so alliance ID reads at a glance from any
            # angle -- the flat/top-down view's equivalent is its heavier
            # bumper-colored outline.
            top_pts = [self._camera.project(x, y, ROBOT_BODY_HEIGHT) for x, y in corners]
            top_poly = QtGui.QPolygonF([QtCore.QPointF(p[0], p[1]) for p in top_pts])
            painter.setPen(QtGui.QPen(bumper_color, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(top_poly)
        else:
            body_poly = QtGui.QPolygonF([QtCore.QPointF(*self._to_widget(wx, wy, scale)) for wx, wy in corners])
            painter.setPen(Qt.NoPen)
            painter.setBrush(QtGui.QColor(theme.BG_RAISED))
            painter.drawPolygon(body_poly)
            painter.setPen(QtGui.QPen(bumper_color, 4))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(body_poly)

        fx, fy = self._robot_to_world(robot, half_l, 0.0)
        fwx, fwy = self._to_widget(fx, fy, scale)
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_PRIMARY), 2))
        painter.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(fwx, fwy))

        if robot.held_pieces:
            # Mirrors the lateral fan-out Robot.sync_held_piece_positions
            # applies to the actual held pieces, so e.g. a held coral and
            # algae show as two distinct dots rather than one on top of the
            # other.
            painter.setPen(Qt.NoPen)
            count = len(robot.held_pieces)
            for i, held_piece in enumerate(robot.held_pieces):
                local_y = (i - (count - 1) / 2.0) * robot.HELD_PIECE_SPACING
                px, py = self._robot_to_world(robot, half_l * 0.4, local_y)
                wx, wy = self._to_widget(px, py, scale)
                held_color = held_piece.color
                painter.setBrush(QtGui.QColor(held_color) if held_color else QtGui.QColor(255, 110, 0))
                painter.drawEllipse(QtCore.QPointF(wx, wy), 5, 5)

        self._draw_side_manipulators(painter, robot, scale)

        self._draw_action_progress(painter, robot, cx, cy, half_l * local_scale, half_w * local_scale)

    def _draw_intent_overlay(self, painter, scale: float) -> None:
        """Optional debug overlay for AI-driven robots: highlights the
        controller's current target region (a ScoringRegion or
        IntakeLocation, looked up by name -- Intent.target_region is a
        plain string so this stays game-agnostic), draws a dashed line
        to the target piece/region, and labels the robot with its
        active tactic. Toggle via `show_intent`."""
        if not self.show_intent:
            return
        if self.playback_targets is not None:
            for robot in self.match.robots:
                target_name = self.playback_targets.get(robot.characteristics.name)
                tactic_name = self.playback_tactics.get(robot.characteristics.name) if self.playback_tactics else None
                self._draw_one_robot_intent_playback(painter, robot, target_name, tactic_name, scale)
            return
        for robot in self.match.robots:
            intent = robot.intent
            if intent is None:
                continue
            self._draw_one_robot_intent(painter, robot, intent, scale)

    def _draw_one_robot_intent(self, painter, robot, intent, scale: float) -> None:
        target_point, zone = None, None
        piece = intent.target_piece
        if piece is not None and piece.held_by is None and not piece.scored:
            target_point = self._to_widget(piece.position.x, piece.position.y, scale)
        elif intent.target_region is not None:
            zone = self._zone_by_name(intent.target_region)
            if zone is not None:
                cx, cy = polygon_centroid(zone.vertices)
                target_point = self._to_widget(cx, cy, scale)

        self._draw_intent_pairing(painter, robot, zone, target_point, intent.tactic_name, scale)

    def _draw_one_robot_intent_playback(self, painter, robot, target_name, tactic_name, scale: float) -> None:
        """Same visual as _draw_one_robot_intent, but resolves the target
        from telemetry's recorded RobotSnapshot.target_name string instead
        of the live Intent object -- see TelemetryRecorder._target_name_for.
        A region name is looked up live (zones don't move/despawn); a
        targeted piece is recorded as a synthetic 'piece:type@x,y' label and
        drawn at that literal recorded point, since the live GamePiece may
        since have scored/despawned and no longer exist to read a position
        from."""
        if target_name is None:
            if tactic_name is None:
                return
            self._draw_intent_pairing(painter, robot, None, None, tactic_name, scale)
            return

        target_point, zone = None, None
        if target_name.startswith("piece:"):
            try:
                coords = target_name.rsplit("@", 1)[1]
                px, py = (float(v) for v in coords.split(","))
                target_point = self._to_widget(px, py, scale)
            except (IndexError, ValueError):
                target_point = None
        else:
            zone = self._zone_by_name(target_name)
            if zone is not None:
                cx, cy = polygon_centroid(zone.vertices)
                target_point = self._to_widget(cx, cy, scale)

        self._draw_intent_pairing(painter, robot, zone, target_point, tactic_name, scale)

    def _draw_intent_pairing(self, painter, robot, zone, target_point, tactic_name, scale: float) -> None:
        color = ALLIANCE_COLORS.get(robot.alliance, QtGui.QColor(theme.ACCENT_CYAN))
        rx, ry = self._to_widget(robot.pose.x, robot.pose.y, scale)

        if zone is not None:
            painter.setPen(QtGui.QPen(color, 3, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(self._polygon(zone.vertices, scale))

        if target_point is not None:
            painter.setPen(QtGui.QPen(color, 1.5, Qt.DashLine))
            painter.drawLine(QtCore.QPointF(rx, ry), QtCore.QPointF(*target_point))

        # Position above the action-progress bar stack (bar + optional
        # deposit-action label drawn by _draw_action_progress) so the two
        # overlays never overlap, regardless of robot size.
        half_w = robot.characteristics.width / 2.0 * self._scale_at(robot.pose.x, robot.pose.y, scale)
        bar_y = ry - half_w - _BAR_H - 6
        text_bottom = bar_y - 13 - 4
        painter.setPen(color)
        painter.setFont(theme.technical_font(8, bold=True))
        painter.drawText(QtCore.QRectF(rx - 55, text_bottom - 14, 110, 14), Qt.AlignCenter, tactic_name.upper())

    def _zone_by_name(self, name: str):
        for region in self.match.field.scoring_regions:
            if region.name == name:
                return region
        for location in self.match.field.intake_locations:
            if location.name == name:
                return location
        return None

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

    def _draw_side_manipulators(self, painter, robot, scale: float) -> None:
        """Small circular badges on each edge the robot has a manipulator
        on -- cyan for intake, amber for scoring, labeled with the piece
        type(s) that side handles. Positioned from a robot-local offset
        (SIDE_OUTWARD, the same front=+x/left=+y convention _rect_corners
        uses) projected to world then screen, so this works unmodified
        under a driver-station camera; badge size itself stays a fixed
        screen radius regardless of view, same as before this was
        projected per-point."""
        half_l = robot.characteristics.length / 2.0
        half_w = robot.characteristics.width / 2.0
        for side, mode, label in self._side_manipulator_tags(robot):
            nx, ny = SIDE_OUTWARD.get(side, (1.0, 0.0))
            tx, ty = -ny, nx  # tangent along the edge, to separate IN/OUT badges
            shift = -6.0 if mode == "in" else 6.0  # inches
            local_x = nx * half_l + tx * shift
            local_y = ny * half_w + ty * shift
            wx, wy = self._to_widget(*self._robot_to_world(robot, local_x, local_y), scale)
            color = QtGui.QColor(theme.ACCENT_CYAN) if mode == "in" else QtGui.QColor(theme.ACCENT_AMBER)
            r = 6.5
            painter.setPen(QtGui.QPen(color, 1.5))
            painter.setBrush(color.darker(220))
            painter.drawEllipse(QtCore.QPointF(wx, wy), r, r)
            painter.setPen(color)
            painter.setFont(theme.technical_font(6, bold=True))
            painter.drawText(QtCore.QRectF(wx - r, wy - r, 2 * r, 2 * r), Qt.AlignCenter, label)

    def _draw_action_progress(self, painter, robot, cx: float, cy: float, half_l: float, half_w: float) -> None:
        bar_w = max(2 * half_l, 2 * half_w)
        bar_h = _BAR_H
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
        if status is not None:
            outline = VALID_OUTLINE if status == "valid" else QtGui.QColor(theme.ACCENT_RED)
            painter.setPen(QtGui.QPen(outline, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(QtCore.QRectF(bar_x, bar_y, bar_w, bar_h))
            return

        # Same idea for a station-style intake location (e.g. a 1-capacity
        # staged REEF ALGAE zone): once the robot is actually positioned
        # over it, its own sprite covers the station highlight entirely,
        # so an empty cyan-outline bar is the only way to see "ready to
        # collect" before RunIntake starts filling it in.
        if self._robot_station_ready(robot):
            painter.setPen(QtGui.QPen(QtGui.QColor(theme.ACCENT_CYAN), 2))
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

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
from common_sim.field.game_piece import piece_spec
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
# Drawn just past the scoring region's outer edge (see _reef_grid_anchor)
# rather than at its centroid -- the centroid sits inside the region's own
# translucent valid/invalid fill and right on top of the staged ALGAE,
# which narrows to a strip through the middle of that same rectangle
# (REEF_ALGAE_ZONE_WIDTH in game_specific/reefscape/field.py), so the grid
# used to render on top of both. World units (inches), not pixels.
REEF_GRID_OUTWARD_MARGIN = 6.0

# Visual-only box heights for a robot drawn under a driver-station camera --
# this sim tracks no physical robot height (RobotCharacteristics has no z
# dimension), so these are fixed stand-ins just tall enough to read as a
# solid chassis on top of a bumper, not a measurement of anything real.
ROBOT_BUMPER_HEIGHT = 5.0
ROBOT_BODY_HEIGHT = 12.0

# Visual-only field perimeter. Only the long side guardrails are drawn: the
# alliance walls sit directly in the driver's sightline and add more clutter
# than useful depth information in this perspective.
SIDE_WALL_HEIGHT = 24.0
WALL_POST_SPACING = 72.0

# Height of the collect/score progress bar drawn above each robot by
# _draw_action_progress -- shared with _draw_one_robot_intent so the intent
# label can be positioned above the bar stack without overlapping it.
_BAR_H = 5

# Big top-corner score readout drawn only under a driver-station camera
# (see _draw_driver_scoreboard) -- a perspective projection keeps the
# field below the horizon line, so both top corners are otherwise dead
# space no field content ever reaches.
DRIVER_SCORE_MARGIN = 12
# Clears _draw_hud's small "t=...  phase=..." line, which keeps drawing
# top-left regardless of view mode.
DRIVER_SCORE_TOP = 30
DRIVER_SCORE_BOX_W = 150
DRIVER_SCORE_LABEL_SIZE = 11
DRIVER_SCORE_FONT_SIZE = 40


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

        boundary = (
            (0, 0), (self.match.field.width, 0),
            (self.match.field.width, self.match.field.height), (0, self.match.field.height),
        )
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 2))
        painter.setBrush(QtGui.QColor(19, 42, 39) if self._camera is not None else Qt.NoBrush)
        painter.drawPolygon(self._polygon(boundary, scale))
        if self._camera is not None:
            self._draw_driver_carpet_markings(painter, scale)
            self._draw_driver_side_walls(painter)

        self._draw_scoring_regions(painter, scale)
        self._draw_intake_locations(painter, scale)
        self._draw_emitter_regions(painter, scale)
        self._draw_protected_zones(painter, scale)
        self._draw_field_visuals(painter, scale)
        self._draw_obstacles(painter, scale)
        self._draw_region_piece_counts(painter, scale)
        self._draw_pieces(painter, scale)
        self._draw_robots(painter, scale)
        self._draw_intent_overlay(painter, scale)
        self._draw_hud(painter)
        if self._camera is not None:
            self._draw_driver_scoreboard(painter)
        painter.end()

    def _polygon(self, vertices, scale: float) -> QtGui.QPolygonF:
        poly = QtGui.QPolygonF()
        for vx, vy in vertices:
            wx, wy = self._to_widget(vx, vy, scale)
            poly.append(QtCore.QPointF(wx, wy))
        return poly

    def _project(self, x: float, y: float, z: float, scale: float) -> QtCore.QPointF:
        if self._camera is not None:
            px, py, _ = self._camera.project(x, y, z)
        else:
            px, py = self._to_widget(x, y, scale)
        return QtCore.QPointF(px, py)

    def _draw_driver_carpet_markings(self, painter, scale: float) -> None:
        """Sparse field markings that make distance and field center legible."""
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(220, 225, 220, 75), 1, Qt.DashLine))
        mid_x = self.match.field.width / 2.0
        painter.drawLine(self._project(mid_x, 0.0, 0.1, scale), self._project(mid_x, self.match.field.height, 0.1, scale))

        # Subtle transverse ticks give the otherwise featureless carpet a
        # perspective ruler without pretending to be game-rule markings.
        painter.setPen(QtGui.QPen(QtGui.QColor(180, 195, 190, 35), 1))
        x = WALL_POST_SPACING
        while x < self.match.field.width:
            painter.drawLine(self._project(x, 0.0, 0.1, scale), self._project(x, self.match.field.height, 0.1, scale))
            x += WALL_POST_SPACING

    def _draw_wall_segment(
        self, painter, start: tuple[float, float], end: tuple[float, float],
        height: float, color: QtGui.QColor, *, post_spacing: float,
    ) -> None:
        x1, y1 = start
        x2, y2 = end
        bottom1 = self._project(x1, y1, 0.0, 1.0)
        bottom2 = self._project(x2, y2, 0.0, 1.0)
        top2 = self._project(x2, y2, height, 1.0)
        top1 = self._project(x1, y1, height, 1.0)
        panel = QtGui.QColor(color)
        panel.setAlpha(24)
        painter.setPen(Qt.NoPen)
        painter.setBrush(panel)
        painter.drawPolygon(QtGui.QPolygonF([bottom1, bottom2, top2, top1]))

        rail = QtGui.QColor(color)
        rail.setAlpha(185)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QtGui.QPen(rail, 2.5))
        painter.drawLine(top1, top2)

        length = math.hypot(x2 - x1, y2 - y1)
        posts = max(1, int(math.ceil(length / post_spacing)))
        painter.setPen(QtGui.QPen(QtGui.QColor(rail), 1.2))
        for i in range(posts + 1):
            t = i / posts
            x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            painter.drawLine(self._project(x, y, 0.0, 1.0), self._project(x, y, height, 1.0))

    def _draw_driver_side_walls(self, painter) -> None:
        """Polycarbonate guardrails along the field's two long sides."""
        width, height = self.match.field.width, self.match.field.height
        neutral = QtGui.QColor(175, 205, 215)
        self._draw_wall_segment(
            painter, (0.0, 0.0), (width, 0.0), SIDE_WALL_HEIGHT,
            neutral, post_spacing=WALL_POST_SPACING,
        )
        self._draw_wall_segment(
            painter, (width, height), (0.0, height), SIDE_WALL_HEIGHT,
            neutral, post_spacing=WALL_POST_SPACING,
        )

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
        face, not whichever type happened to be picked up first.

        Full and blocked regions read "invalid" here even though
        `deposit_region_for` still returns them. That is deliberate: this
        indicator answers "will this deposit score", and the answer for a
        REEF branch that already holds a CORAL, or one whose ALGAE is
        still in the way, is no -- `Match._try_score` drops both on the
        floor. The deposit *timer* is intentionally not gated on either
        (a robot must stay free to release a piece anywhere, and gating
        the timer on a condition that can change underneath it is how
        this sim has historically grown frozen-robot bugs), so the honest
        indicator is stricter than the gate rather than equal to it."""
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
        if self.match.deposit_region_for(robot, piece) is not region:
            return "invalid"
        action = robot.deposit_action
        if action is not None and (
            self.match.region_full(region, action) or self.match.region_blocked(region, action)
        ):
            return "invalid"
        return "valid"

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
            if set(REEF_GRID_LEVELS) & region.actions:
                alliance = region.alliance or "blue"
                cx, cy = self._reef_grid_anchor(region.vertices)
                wx, wy = self._to_widget(cx, cy, scale)
                self._draw_reef_grid(painter, wx, wy, counts, alliance)
            elif region.actions:
                cx, cy = polygon_centroid(region.vertices)
                wx, wy = self._to_widget(cx, cy, scale)
                self._draw_count_badge(painter, wx, wy, str(sum(counts.values())))

    def _reef_grid_anchor(self, vertices) -> tuple[float, float]:
        """World point just outside a REEF face's scoring rectangle, along
        its outward normal -- clear of the region's own fill and the
        ALGAE strip through its middle (see REEF_GRID_OUTWARD_MARGIN).
        `vertices` is the face-aligned quad _face_rect builds: corners
        0/1 on the inner (hex-facing) edge, 2/3 on the outer edge -- see
        game_specific/reefscape/field.py."""
        (ix0, iy0), (ix1, iy1), (ox0, oy0), (ox1, oy1) = vertices[0], vertices[1], vertices[2], vertices[3]
        inner_mid = ((ix0 + ix1) / 2.0, (iy0 + iy1) / 2.0)
        outer_mid = ((ox0 + ox1) / 2.0, (oy0 + oy1) / 2.0)
        dx, dy = outer_mid[0] - inner_mid[0], outer_mid[1] - inner_mid[1]
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        return (outer_mid[0] + ux * REEF_GRID_OUTWARD_MARGIN, outer_mid[1] + uy * REEF_GRID_OUTWARD_MARGIN)

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

    def _visual_color(self, visual) -> QtGui.QColor:
        if visual.color:
            return QtGui.QColor(visual.color)
        return ALLIANCE_COLORS.get(visual.alliance, QtGui.QColor(175, 185, 195))

    def _draw_field_visuals(self, painter, scale: float) -> None:
        """Draw non-physical hardware declared by the game package."""
        if self._camera is None:
            return
        for visual in getattr(self.match.field, "visuals", ()):
            color = self._visual_color(visual)
            if visual.kind == "prism" and len(visual.points) >= 3:
                top = QtGui.QColor(color)
                top.setAlpha(55)
                side = QtGui.QColor(color)
                side.setAlpha(38)
                painter.setPen(QtGui.QPen(color, 1.5))
                for _depth, poly, brush in self._prism_faces(
                    visual.points, visual.base_height, visual.height, top, side,
                ):
                    painter.setBrush(brush)
                    painter.drawPolygon(poly)
            elif visual.kind == "frame" and len(visual.points) >= 3:
                bottom = [self._project(x, y, visual.base_height, scale) for x, y in visual.points]
                top = [self._project(x, y, visual.height, scale) for x, y in visual.points]
                fill = QtGui.QColor(color)
                fill.setAlpha(12)
                painter.setPen(QtGui.QPen(color, 2))
                painter.setBrush(fill)
                painter.drawPolygon(QtGui.QPolygonF(top))
                painter.setBrush(Qt.NoBrush)
                painter.drawPolygon(QtGui.QPolygonF(bottom))
                for low, high in zip(bottom, top):
                    painter.drawLine(low, high)
            elif visual.kind == "net" and len(visual.points) >= 2:
                self._draw_net_visual(painter, visual, color, scale)

    def _draw_net_visual(self, painter, visual, color: QtGui.QColor, scale: float) -> None:
        """Vertical mesh between the first and last points of a net visual."""
        (x1, y1), (x2, y2) = visual.points[0], visual.points[-1]
        low1 = self._project(x1, y1, visual.base_height, scale)
        low2 = self._project(x2, y2, visual.base_height, scale)
        high2 = self._project(x2, y2, visual.height, scale)
        high1 = self._project(x1, y1, visual.height, scale)
        mesh = QtGui.QColor(color)
        mesh.setAlpha(80)
        fill = QtGui.QColor(color)
        fill.setAlpha(10)
        painter.setPen(QtGui.QPen(mesh, 1))
        painter.setBrush(fill)
        painter.drawPolygon(QtGui.QPolygonF([low1, low2, high2, high1]))

        divisions = 8
        for i in range(divisions + 1):
            t = i / divisions
            x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            painter.drawLine(
                self._project(x, y, visual.base_height, scale),
                self._project(x, y, visual.height, scale),
            )
        levels = 4
        for i in range(levels + 1):
            z = visual.base_height + (visual.height - visual.base_height) * i / levels
            painter.drawLine(self._project(x1, y1, z, scale), self._project(x2, y2, z, scale))

        painter.setPen(QtGui.QPen(color, 3))
        painter.drawLine(high1, high2)

    def _draw_obstacles(self, painter, scale: float) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 2))
        for obstacle in self.match.field.obstacles:
            if self._camera is not None and obstacle.height > 0:
                if getattr(obstacle, "render_style", "solid") == "lattice":
                    self._draw_lattice_obstacle(painter, obstacle, scale)
                    continue
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

    def _draw_lattice_obstacle(self, painter, obstacle, scale: float) -> None:
        """Open-frame extrusion for truss-like field structures such as a REEF."""
        color = QtGui.QColor(185, 195, 200)
        faint = QtGui.QColor(theme.BG_RAISED)
        faint.setAlpha(25)
        bottom = [self._project(x, y, 0.0, scale) for x, y in obstacle.vertices]
        top = [self._project(x, y, obstacle.height, scale) for x, y in obstacle.vertices]

        painter.setPen(QtGui.QPen(color, 1.5))
        painter.setBrush(faint)
        painter.drawPolygon(QtGui.QPolygonF(top))
        painter.setBrush(Qt.NoBrush)
        painter.drawPolygon(QtGui.QPolygonF(bottom))
        for i, (low1, high1) in enumerate(zip(bottom, top)):
            j = (i + 1) % len(bottom)
            painter.drawLine(low1, high1)
            # Cross-bracing is the cue that this is an open structure, not
            # a tinted solid wall. Low alpha keeps far-side braces quiet.
            brace = QtGui.QColor(color)
            brace.setAlpha(115)
            painter.setPen(QtGui.QPen(brace, 1))
            painter.drawLine(low1, top[j])
            painter.drawLine(bottom[j], high1)
            painter.setPen(QtGui.QPen(color, 1.5))
        painter.drawPolygon(QtGui.QPolygonF(top))

    def _draw_pieces(self, painter, scale: float) -> None:
        if self.playback_pieces is not None:
            pieces = self.playback_pieces
            if self._camera is not None:
                pieces = sorted(pieces, key=lambda p: self._camera.project(p.position_x, p.position_y, 0.0)[2], reverse=True)
            for snapshot in pieces:
                wx, wy = self._to_widget(snapshot.position_x, snapshot.position_y, scale)
                r = max(snapshot.radius * self._scale_at(snapshot.position_x, snapshot.position_y, scale), 2.0)
                self._draw_piece_marker(
                    painter, snapshot.piece_type, wx, wy, r,
                    QtGui.QColor(snapshot.color) if snapshot.color else QtGui.QColor(255, 110, 0),
                )
            return
        pieces = self.match.active_pieces
        if self._camera is not None:
            pieces = sorted(pieces, key=lambda p: self._camera.project(p.position.x, p.position.y, 0.0)[2], reverse=True)
        for piece in pieces:
            wx, wy = self._to_widget(piece.position.x, piece.position.y, scale)
            r = max(piece.radius * self._scale_at(piece.position.x, piece.position.y, scale), 2.0)
            self._draw_piece_marker(
                painter, piece.piece_type, wx, wy, r,
                QtGui.QColor(piece.color) if piece.color else QtGui.QColor(255, 110, 0),
            )

    def _draw_piece_marker(
        self, painter, piece_type: str, cx: float, cy: float, radius: float,
        color: QtGui.QColor, *, shadow: bool = True,
    ) -> None:
        """Distinct, screen-readable silhouettes over circular 2D physics."""
        shape = piece_spec(piece_type).display_shape
        r = max(radius, 2.5)
        if shadow and self._camera is not None:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QtGui.QColor(0, 0, 0, 90))
            painter.drawEllipse(QtCore.QPointF(cx + 1.5, cy + max(2.0, r * 0.65)), r * 1.05, max(1.5, r * 0.42))

        if shape == "sphere":
            gradient = QtGui.QRadialGradient(QtCore.QPointF(cx - r * 0.35, cy - r * 0.4), r * 1.35)
            gradient.setColorAt(0.0, color.lighter(180))
            gradient.setColorAt(0.45, color)
            gradient.setColorAt(1.0, color.darker(210))
            painter.setPen(QtGui.QPen(color.lighter(135), 1))
            painter.setBrush(QtGui.QBrush(gradient))
            painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)
            return

        painter.save()
        painter.translate(cx, cy)
        if shape == "capsule":
            painter.rotate(-18.0)
            width, height = max(9.0, r * 3.4), max(4.5, r * 1.45)
            painter.setPen(QtGui.QPen(QtGui.QColor(185, 195, 200), 1.2))
            painter.setBrush(color)
            painter.drawRoundedRect(QtCore.QRectF(-width / 2, -height / 2, width, height), height / 2, height / 2)
            # A dark inner bore makes the white CORAL read as tubing rather
            # than a generic pill, even when perspective shrinks it.
            painter.setPen(Qt.NoPen)
            painter.setBrush(QtGui.QColor(55, 65, 70))
            painter.drawEllipse(QtCore.QPointF(width / 2 - height * 0.42, 0.0), height * 0.22, height * 0.30)
        elif shape == "box":
            painter.rotate(8.0)
            painter.setPen(QtGui.QPen(color.lighter(150), 1))
            painter.setBrush(color.darker(115))
            painter.drawRect(QtCore.QRectF(-r, -r, 2 * r, 2 * r))
            painter.drawLine(QtCore.QPointF(-r, -r), QtCore.QPointF(r, r))
        elif shape == "shard":
            painter.setPen(QtGui.QPen(color.lighter(140), 1))
            painter.setBrush(color)
            painter.drawPolygon(QtGui.QPolygonF([
                QtCore.QPointF(-r, r * 0.45), QtCore.QPointF(-r * 0.25, -r),
                QtCore.QPointF(r, -r * 0.2), QtCore.QPointF(r * 0.35, r),
            ]))
        else:
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(QtCore.QPointF(0.0, 0.0), r, r)
        painter.restore()

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

    def _project_robot_local(
        self, robot, local_x: float, local_y: float, z: float, scale: float,
    ) -> QtCore.QPointF:
        x, y = self._robot_to_world(robot, local_x, local_y)
        return self._project(x, y, z if self._camera is not None else 0.0, scale)

    def _draw_robot_shadow(self, painter, corners, scale: float) -> None:
        if self._camera is None:
            return
        points = []
        for x, y in corners:
            p = self._project(x, y, 0.0, scale)
            points.append(QtCore.QPointF(p.x() + 2.0, p.y() + 4.0))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(0, 0, 0, 105))
        painter.drawPolygon(QtGui.QPolygonF(points))

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

        # Corners are projected explicitly (rather than the old
        # painter.translate()+rotate()) because that's affine and does not
        # survive a perspective projection -- a driver-station camera needs
        # each corner mapped through _to_widget (or, extruded, through
        # _prism_faces) on its own.
        corners = self._rect_corners(robot)
        self._draw_robot_shadow(painter, corners, scale)
        if isinstance(robot.controller, HumanController):
            self._draw_human_glow(painter, robot, cx, cy, max(half_l, half_w) * local_scale)
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

        self._draw_robot_front_marker(painter, robot, scale)

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
                held_color = QtGui.QColor(held_piece.color) if held_piece.color else QtGui.QColor(255, 110, 0)
                self._draw_piece_marker(
                    painter, held_piece.piece_type, wx, wy, 5.0, held_color, shadow=False,
                )

        self._draw_side_manipulators(painter, robot, scale)

        self._draw_action_progress(painter, robot, cx, cy, half_l * local_scale, half_w * local_scale)

    def _draw_robot_front_marker(self, painter, robot, scale: float) -> None:
        """Large top-plate chevron plus a contrasting front-bumper stripe."""
        half_l = robot.characteristics.length / 2.0
        half_w = robot.characteristics.width / 2.0
        z = ROBOT_BODY_HEIGHT + 0.15
        rear_l = self._project_robot_local(robot, half_l * 0.05, half_w * 0.48, z, scale)
        tip = self._project_robot_local(robot, half_l * 0.80, 0.0, z, scale)
        rear_r = self._project_robot_local(robot, half_l * 0.05, -half_w * 0.48, z, scale)
        white = QtGui.QColor(245, 248, 250)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QtGui.QPen(white, 3.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPolyline(QtGui.QPolygonF([rear_l, tip, rear_r]))

        front_r = self._project_robot_local(robot, half_l + 0.1, -half_w * 0.72, ROBOT_BUMPER_HEIGHT, scale)
        front_l = self._project_robot_local(robot, half_l + 0.1, half_w * 0.72, ROBOT_BUMPER_HEIGHT, scale)
        painter.setPen(QtGui.QPen(white, 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(front_r, front_l)

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
        """Edge bands and directional arrows for every robot mechanism.

        On a human-controlled robot, cyan arrows point into the chassis
        for intake and amber arrows point out for scoring. AI robots keep
        the edge bands and piece initials but omit the arrows to reduce
        field clutter around robots the driver is not controlling.
        """
        half_l = robot.characteristics.length / 2.0
        half_w = robot.characteristics.width / 2.0
        tags = self._side_manipulator_tags(robot)
        show_direction_arrows = isinstance(robot.controller, HumanController)
        for side in SIDE_OUTWARD:
            side_tags = [(mode, label) for tag_side, mode, label in tags if tag_side == side]
            if not side_tags:
                continue
            nx, ny = SIDE_OUTWARD.get(side, (1.0, 0.0))
            tx, ty = -ny, nx
            normal_extent = half_l if nx else half_w
            tangent_extent = half_w if nx else half_l
            count = len(side_tags)
            for index, (mode, label) in enumerate(side_tags):
                lane_center = ((index + 0.5) / count - 0.5) * 1.35 * tangent_extent
                lane_half = tangent_extent * 0.55 / count
                edge = normal_extent + 0.6
                z = ROBOT_BODY_HEIGHT + 0.3
                p1 = self._project_robot_local(
                    robot, nx * edge + tx * (lane_center - lane_half),
                    ny * edge + ty * (lane_center - lane_half), z, scale,
                )
                p2 = self._project_robot_local(
                    robot, nx * edge + tx * (lane_center + lane_half),
                    ny * edge + ty * (lane_center + lane_half), z, scale,
                )
                color = QtGui.QColor(theme.ACCENT_CYAN) if mode == "in" else QtGui.QColor(theme.ACCENT_AMBER)
                painter.setPen(QtGui.QPen(color, 5, Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(p1, p2)

                if show_direction_arrows:
                    outer_extent, inner_extent = normal_extent + 6.0, normal_extent - 3.0
                    start_extent, tip_extent = (outer_extent, inner_extent) if mode == "in" else (inner_extent, outer_extent)
                    start = self._project_robot_local(
                        robot, nx * start_extent + tx * lane_center,
                        ny * start_extent + ty * lane_center, z, scale,
                    )
                    tip = self._project_robot_local(
                        robot, nx * tip_extent + tx * lane_center,
                        ny * tip_extent + ty * lane_center, z, scale,
                    )
                    self._draw_screen_arrow(painter, start, tip, color)

                mid_x, mid_y = (p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0
                text = label
                box_w = max(14.0, 7.0 * len(text))
                box = QtCore.QRectF(mid_x - box_w / 2.0, mid_y - 16.0, box_w, 12.0)
                background = QtGui.QColor(10, 18, 22, 205)
                painter.setPen(Qt.NoPen)
                painter.setBrush(background)
                painter.drawRoundedRect(box, 2.0, 2.0)
                painter.setPen(color)
                painter.setFont(theme.technical_font(7, bold=True))
                painter.drawText(box, Qt.AlignCenter, text)

    @staticmethod
    def _draw_screen_arrow(painter, start: QtCore.QPointF, tip: QtCore.QPointF, color: QtGui.QColor) -> None:
        dx, dy = tip.x() - start.x(), tip.y() - start.y()
        length = math.hypot(dx, dy)
        if length < 1e-3:
            return
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        head = min(6.0, max(3.5, length * 0.45))
        painter.setPen(QtGui.QPen(color, 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(start, tip)
        painter.setBrush(color)
        painter.drawPolygon(QtGui.QPolygonF([
            tip,
            QtCore.QPointF(tip.x() - ux * head + px * head * 0.55, tip.y() - uy * head + py * head * 0.55),
            QtCore.QPointF(tip.x() - ux * head - px * head * 0.55, tip.y() - uy * head - py * head * 0.55),
        ]))

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

        if self._camera is not None:
            # Driver view shows scores as big corner numbers instead (see
            # _draw_driver_scoreboard) -- repeating them small here would
            # just stack clutter on top of that.
            return

        for alliance in ("red", "blue"):
            score = self.match.scores.get(alliance, 0.0)
            text = f"{alliance.upper()}: {score:.0f}   "
            painter.setPen(ALLIANCE_COLORS.get(alliance, QtGui.QColor(theme.TEXT_PRIMARY)))
            painter.drawText(x, y, text)
            x += fm.horizontalAdvance(text)

    def _draw_driver_scoreboard(self, painter) -> None:
        """Big RED/BLUE score readout in the top-left/top-right corners --
        the dead space a driver-station perspective always leaves above
        the field's near edge (see the module docstring on
        DRIVER_SCORE_MARGIN). Sized to read at a glance, unlike the small
        running _draw_hud line the top-down view keeps."""
        y = DRIVER_SCORE_TOP
        for alliance, align in (("red", Qt.AlignLeft), ("blue", Qt.AlignRight)):
            x = DRIVER_SCORE_MARGIN if align == Qt.AlignLeft \
                else self.width() - DRIVER_SCORE_MARGIN - DRIVER_SCORE_BOX_W
            color = ALLIANCE_COLORS.get(alliance, QtGui.QColor(theme.TEXT_PRIMARY))

            painter.setPen(QtGui.QColor(theme.TEXT_DIM))
            painter.setFont(theme.technical_font(DRIVER_SCORE_LABEL_SIZE, bold=True))
            painter.drawText(
                QtCore.QRectF(x, y, DRIVER_SCORE_BOX_W, DRIVER_SCORE_LABEL_SIZE + 4),
                align | Qt.AlignTop, alliance.upper(),
            )

            score = self.match.scores.get(alliance, 0.0)
            painter.setPen(color)
            painter.setFont(theme.technical_font(DRIVER_SCORE_FONT_SIZE, bold=True))
            painter.drawText(
                QtCore.QRectF(x, y + DRIVER_SCORE_LABEL_SIZE + 4, DRIVER_SCORE_BOX_W, DRIVER_SCORE_FONT_SIZE + 8),
                align | Qt.AlignTop | Qt.TextDontClip, f"{score:.0f}",
            )

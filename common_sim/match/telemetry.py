"""
Telemetry recording and playback for match replay analysis.

Records robot and match-level data at 20Hz, and active game-piece positions
at a coarser 5Hz (a field can carry up to a few hundred pieces at once, so
recording them at the same rate as robots would multiply row counts by the
piece count), enabling timeline scrubbing to review match progression and
debug strategy/tactics behavior.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import pandas as pd

if TYPE_CHECKING:
    from common_sim.match.match import Match
    from common_sim.robot.robot import Robot


@dataclass
class RobotSnapshot:
    """Per-robot state at a recording tick."""
    time: float
    robot_name: str
    position_x: float
    position_y: float
    orientation_deg: float
    velocity_x: float
    velocity_y: float
    tactic_name: str | None
    target_name: str | None


@dataclass
class MatchSnapshot:
    """Match-level state at a recording tick."""
    time: float
    phase: str
    alliance_scores: dict[str, float]
    region_scores: dict[str, dict[str, int]]
    active_piece_count: int
    # IntakeLocation.name -> pieces remaining, mirroring Match.station_supply's
    # own convention: a location with unlimited supply (starting_pieces=None)
    # never appears here, so a .get(name) miss always means "unlimited".
    station_supply: dict[str, int]


@dataclass
class PieceSnapshot:
    """State of one active (unscored) game piece at a recording tick. A
    piece stops appearing once it scores, mirroring the convention behind
    MatchSnapshot.active_piece_count. Carries radius/color (not just
    position) so a scrubbed-to piece can be drawn from telemetry alone --
    a scored piece is permanently removed from Match.active_pieces/pymunk
    (see Match.step), so replaying it has no live GamePiece object to read
    those from."""
    time: float
    piece_id: int
    piece_type: str
    position_x: float
    position_y: float
    radius: float
    color: str | None
    held_by: str | None  # robot_name, or None if free on the field


class TelemetryRecorder:
    """Records robot/match telemetry at 20Hz (every 3rd tick from 60Hz) and
    game-piece telemetry at a coarser 5Hz (every 12th tick) -- see module
    docstring for why pieces are decimated separately."""

    RECORD_HZ = 20
    PIECE_RECORD_HZ = 5
    TICK_HZ = 60

    def __init__(self, match: Match):
        self.match = match
        self.robot_frames: list[RobotSnapshot] = []
        self.match_frames: list[MatchSnapshot] = []
        self.piece_frames: list[PieceSnapshot] = []
        self._tick_counter = 0

    def tick(self) -> None:
        """Call once per simulation tick (60Hz). Records robot/match state
        at 20Hz and piece state at 5Hz internally."""
        self._tick_counter += 1
        elapsed = self.match.elapsed

        if self._tick_counter % (self.TICK_HZ // self.RECORD_HZ) == 0:
            for robot in self.match.robots:
                self._record_robot(robot, elapsed)
            self._record_match(elapsed)

        if self._tick_counter % (self.TICK_HZ // self.PIECE_RECORD_HZ) == 0:
            self._record_pieces(elapsed)

    def _record_robot(self, robot: Robot, elapsed: float) -> None:
        """Record a single robot's state."""
        pose = robot.pose
        velocity = robot.chassis.body.velocity

        tactic_name = None
        target_name = None
        intent = robot.intent
        if intent is not None:
            tactic_name = intent.tactic_name
            target_name = self._target_name_for(intent)

        snapshot = RobotSnapshot(
            time=elapsed,
            robot_name=robot.characteristics.name,
            position_x=pose.x,
            position_y=pose.y,
            orientation_deg=math.degrees(pose.heading),
            velocity_x=velocity.x,
            velocity_y=velocity.y,
            tactic_name=tactic_name,
            target_name=target_name,
        )
        self.robot_frames.append(snapshot)

    @staticmethod
    def _target_name_for(intent) -> str | None:
        """A single display name for whatever `intent` is targeting --
        a scoring/intake region name if set, else a synthetic label for
        a targeted GamePiece (which has no id/name of its own), matching
        the two cases FieldCanvas._draw_one_robot_intent draws a pairing
        line for (see gui_utils/field_canvas.py)."""
        if intent.target_region is not None:
            return intent.target_region
        piece = intent.target_piece
        if piece is not None:
            pos = piece.position
            return f"piece:{piece.piece_type}@{pos.x:.0f},{pos.y:.0f}"
        return None

    def _record_match(self, elapsed: float) -> None:
        """Record match-level state."""
        snapshot = MatchSnapshot(
            time=elapsed,
            phase=self.match.phase.value,
            alliance_scores=dict(self.match.scores),
            region_scores={
                region: dict(actions) for region, actions in self.match.region_scores.items()
            },
            active_piece_count=len(self.match.active_pieces),
            station_supply={
                location.name: remaining for location, remaining in self.match.station_supply.items()
            },
        )
        self.match_frames.append(snapshot)

    def _record_pieces(self, elapsed: float) -> None:
        """Record every active (unscored) game piece. `id(piece)` is used
        as a stable identifier for the piece's lifetime in
        Match.active_pieces (it's never persisted or reused across pieces
        that are simultaneously alive)."""
        for piece in self.match.active_pieces:
            pos = piece.position
            held_by = piece.held_by.characteristics.name if piece.held_by is not None else None
            snapshot = PieceSnapshot(
                time=elapsed,
                piece_id=id(piece),
                piece_type=piece.piece_type,
                position_x=pos.x,
                position_y=pos.y,
                radius=piece.radius,
                color=piece.color,
                held_by=held_by,
            )
            self.piece_frames.append(snapshot)

    def to_robot_dataframe(self) -> pd.DataFrame:
        """Convert robot telemetry to DataFrame."""
        if not self.robot_frames:
            return pd.DataFrame()
        return pd.DataFrame([
            {
                "time": s.time,
                "robot_name": s.robot_name,
                "position_x": s.position_x,
                "position_y": s.position_y,
                "orientation_deg": s.orientation_deg,
                "velocity_x": s.velocity_x,
                "velocity_y": s.velocity_y,
                "tactic_name": s.tactic_name,
                "target_name": s.target_name,
            }
            for s in self.robot_frames
        ])

    def to_match_dataframe(self) -> pd.DataFrame:
        """Convert match telemetry to DataFrame."""
        if not self.match_frames:
            return pd.DataFrame()
        return pd.DataFrame([
            {
                "time": s.time,
                "phase": s.phase,
                "alliance_scores": s.alliance_scores,
                "region_scores": s.region_scores,
                "active_piece_count": s.active_piece_count,
                "station_supply": s.station_supply,
            }
            for s in self.match_frames
        ])

    def to_piece_dataframe(self) -> pd.DataFrame:
        """Convert piece telemetry to DataFrame."""
        if not self.piece_frames:
            return pd.DataFrame()
        return pd.DataFrame([
            {
                "time": s.time,
                "piece_id": s.piece_id,
                "piece_type": s.piece_type,
                "position_x": s.position_x,
                "position_y": s.position_y,
                "radius": s.radius,
                "color": s.color,
                "held_by": s.held_by,
            }
            for s in self.piece_frames
        ])

    def get_piece_states_at_time(self, target_time: float) -> list[PieceSnapshot]:
        """All piece snapshots from the latest recorded tick at or before
        target_time -- pieces are all recorded together each tick, so this
        is one batch rather than a per-piece nearest lookup."""
        times = [s.time for s in self.piece_frames if s.time <= target_time]
        if not times:
            return []
        latest = max(times)
        return [s for s in self.piece_frames if s.time == latest]

    def get_robot_state_at_time(self, target_time: float, robot_name: str) -> RobotSnapshot | None:
        """Get the closest robot snapshot at or before target_time."""
        matching = [s for s in self.robot_frames if s.robot_name == robot_name and s.time <= target_time]
        return matching[-1] if matching else None

    def get_match_state_at_time(self, target_time: float) -> MatchSnapshot | None:
        """Get the closest match snapshot at or before target_time."""
        matching = [s for s in self.match_frames if s.time <= target_time]
        return matching[-1] if matching else None

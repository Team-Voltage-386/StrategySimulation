"""
Generic physical game piece: a circular pymunk body/shape tagged with a
piece_type string (e.g. "coral", "algae", "note"). Game-specific piece
subclasses may override radius/mass/visuals but should not need to
touch physics wiring.
"""
from __future__ import annotations

from dataclasses import dataclass

import pymunk


@dataclass(frozen=True)
class GamePieceSpec:
    """Physical/visual characteristics for one piece_type -- radius, mass,
    and display color. A game package registers one of these per piece
    type it defines (see register_piece_spec), so every place that spawns
    a piece of that type (Match.spawn_piece, an EmitterRegion, a station
    dispense, a robot's preload) gets the same characteristics automatically
    by piece_type alone, rather than each caller re-specifying them and
    risking a mismatch (e.g. a piece spawned via one path ending up a
    different size/color than the same piece_type spawned via another)."""
    radius: float = 8.0
    mass: float = 0.5
    color: str | None = None


# piece_type -> GamePieceSpec, populated by game-specific packages at
# import time (see game_specific.reefscape.game_pieces). Looked up by
# piece_spec() below; a piece_type that was never registered falls back to
# GamePieceSpec()'s defaults, matching GamePiece's own pre-registry
# defaults so unregistered types behave exactly as before this existed.
_PIECE_SPECS: dict[str, GamePieceSpec] = {}


def register_piece_spec(piece_type: str, spec: GamePieceSpec) -> None:
    _PIECE_SPECS[piece_type] = spec


def piece_spec(piece_type: str) -> GamePieceSpec:
    return _PIECE_SPECS.get(piece_type, GamePieceSpec())


class GamePiece:
    def __init__(
        self,
        space: pymunk.Space,
        piece_type: str,
        position: tuple[float, float],
        *,
        radius: float = 8.0,
        mass: float = 0.5,
        collision_type: int = 0,
        source: str = "field",
        color: str | None = None,
    ):
        self.piece_type = piece_type
        self.radius = radius
        self.source = source  # "field" or "station" -- selects intake timing, see robot/robot.py
        self.color = color  # optional display color (e.g. Qt color name/hex); None = caller picks a default
        self.held_by = None   # Robot instance, or None if free
        self.scored = False
        self.last_holder_alliance: str | None = None  # set by Robot on capture; used to attribute score
        self.target_action: str | None = None  # scoring action a robot targeted on release; set by Robot

        moment = pymunk.moment_for_circle(mass, 0, radius)
        self.body = pymunk.Body(mass, moment)
        self.body.position = position
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = 0.4
        self.shape.friction = 0.3
        self.shape.collision_type = collision_type
        self.shape.game_piece = self  # backref for collision-handler dispatch

        self._space = space
        space.add(self.body, self.shape)

    @property
    def position(self) -> pymunk.Vec2d:
        return self.body.position

    def remove_from_space(self) -> None:
        if self.shape in self._space.shapes:
            self._space.remove(self.shape)
        if self.body in self._space.bodies:
            self._space.remove(self.body)

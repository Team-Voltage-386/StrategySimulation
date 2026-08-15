"""
Generic physical game piece: a circular pymunk body/shape tagged with a
piece_type string (e.g. "coral", "algae", "note"). Game-specific piece
subclasses may override radius/mass/visuals but should not need to
touch physics wiring.
"""
from __future__ import annotations

import pymunk


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

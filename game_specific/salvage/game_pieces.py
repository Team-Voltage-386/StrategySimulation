"""
SALVAGE 2027 game pieces. SALVAGE is an *invented* game -- see
game_specific/salvage/field.py's module docstring for why it exists.

Three piece types, against REEFSCAPE's two, and each one is sourced a
deliberately different way:

  CRATE  a light box, handed out by each alliance's own CARGO BAY. Both
         a station handoff and a loose field piece (crates get knocked
         loose), so it is the one type that behaves like REEFSCAPE CORAL.
  CELL   a fuel cell, available *only* from the neutral SALVAGE DEPOT at
         the centre of the field, which holds a finite number of them
         and is shared by both alliances. There is no such thing as a
         CELL lying on the floor: a robot that wants one has to go and
         contest the depot. This is the type that makes scarcity a real
         decision rather than a term stuck at zero.
  SCRAP  debris. Never dispensed by anything a robot drives up to -- it
         is pre-staged on the floor and lobbed in by human players over
         the course of the match (see field.build_field's emitters), so
         its supply arrives on the clock rather than on request.
"""
from __future__ import annotations

from common_sim.field.game_piece import GamePiece, GamePieceSpec, register_piece_spec
from common_sim.match.match import Match

CRATE_TYPE = "crate"
CELL_TYPE = "cell"
SCRAP_TYPE = "scrap"

CRATE_RADIUS = 5.0
CRATE_MASS = 0.4
CELL_RADIUS = 7.0
CELL_MASS = 0.5
SCRAP_RADIUS = 3.5
SCRAP_MASS = 0.25

CRATE_COLOR = "#c8a05a"
CELL_COLOR = "#4fc3f7"
SCRAP_COLOR = "#9e9e9e"

register_piece_spec(CRATE_TYPE, GamePieceSpec(radius=CRATE_RADIUS, mass=CRATE_MASS, color=CRATE_COLOR))
register_piece_spec(CELL_TYPE, GamePieceSpec(radius=CELL_RADIUS, mass=CELL_MASS, color=CELL_COLOR))
register_piece_spec(SCRAP_TYPE, GamePieceSpec(radius=SCRAP_RADIUS, mass=SCRAP_MASS, color=SCRAP_COLOR))


def spawn_crate(match: Match, position: tuple[float, float], source: str = "field") -> GamePiece:
    return match.spawn_piece(CRATE_TYPE, position, source=source)


def spawn_cell(match: Match, position: tuple[float, float], source: str = "field") -> GamePiece:
    return match.spawn_piece(CELL_TYPE, position, source=source)


def spawn_scrap(match: Match, position: tuple[float, float], source: str = "field") -> GamePiece:
    return match.spawn_piece(SCRAP_TYPE, position, source=source)

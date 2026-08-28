"""
REEFSCAPE game pieces: CORAL (scored on REEF branches L1-L4) and ALGAE
(scored in a PROCESSOR or a NET). Radii approximate each piece's real
top-down footprint -- CORAL is a ~4.5in-diameter PVC-like tube, ALGAE a
~16in-diameter ball.

GamePiece itself is generic (any piece_type string); this module registers
the radius/mass/color GamePieceSpec for REEFSCAPE's two piece types (see
common_sim.field.game_piece) so every spawn path -- Match.spawn_piece
directly, an EmitterRegion, a station dispense, a robot's preload -- picks
up the same characteristics by piece_type alone, with no need to redeclare
them at each call site. spawn_coral/spawn_algae remain as the common-case
helpers, passed through Match.spawn_piece rather than constructing
GamePiece directly -- Match owns collision-type assignment and
active_pieces bookkeeping, so a piece must be created via spawn_piece to
actually participate in intake/scoring collisions.
"""
from __future__ import annotations

from common_sim.field.game_piece import GamePiece, GamePieceSpec, register_piece_spec
from common_sim.match.match import Match

CORAL_TYPE = "coral"
ALGAE_TYPE = "algae"

CORAL_RADIUS = 2.25
CORAL_MASS = 0.3
ALGAE_RADIUS = 8.0
ALGAE_MASS = 0.6

register_piece_spec(CORAL_TYPE, GamePieceSpec(
    radius=CORAL_RADIUS, mass=CORAL_MASS, color="white", display_shape="capsule",
))
register_piece_spec(ALGAE_TYPE, GamePieceSpec(
    radius=ALGAE_RADIUS, mass=ALGAE_MASS, color="green", display_shape="sphere",
))


def spawn_coral(match: Match, position: tuple[float, float], source: str = "field") -> GamePiece:
    return match.spawn_piece(CORAL_TYPE, position, source=source)


def spawn_algae(match: Match, position: tuple[float, float], source: str = "field") -> GamePiece:
    return match.spawn_piece(ALGAE_TYPE, position, source=source)

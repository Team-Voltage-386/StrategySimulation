"""
Generic timed mechanisms shared by every robot. These classes know
nothing about pymunk, field layout, or scoring -- they're pure
timers/state-machines that Robot drives with pre-filtered inputs
(nearby pieces, a duration in seconds) each tick.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Optional

from common_sim.field.game_piece import GamePiece

if TYPE_CHECKING:
    from common_sim.field.field_config import IntakeLocation


class Intake:
    """Captures the first eligible nearby piece after `duration_for(piece)`
    seconds of continuous, commanded-active contact. Restarts its timer
    if the target piece changes or intake is commanded off."""

    def __init__(self):
        self._timer = 0.0
        self._target: Optional[GamePiece] = None

    def update(
        self,
        dt: float,
        commanded_active: bool,
        nearby_pieces: list[GamePiece],
        capacity_available: bool,
        duration_for: Callable[[GamePiece], float],
    ) -> Optional[GamePiece]:
        if not commanded_active or not capacity_available or not nearby_pieces:
            self._reset()
            return None

        piece = nearby_pieces[0]
        if piece is not self._target:
            self._target = piece
            self._timer = 0.0

        self._timer += dt
        if self._timer >= duration_for(piece):
            captured = self._target
            self._reset()
            return captured
        return None

    def _reset(self) -> None:
        self._timer = 0.0
        self._target = None

    @property
    def progress(self) -> float:
        return self._timer

    @property
    def target(self) -> Optional[GamePiece]:
        return self._target


class StationIntake:
    """Dispenses a new piece after `duration` seconds of continuous,
    commanded-active operation while a robot sits in an IntakeLocation's
    zone -- `duration` is the collecting robot's own
    RobotCharacteristics.station_intake_time, passed in by the caller
    each tick so this class stays free of any Robot/RobotCharacteristics
    dependency. Mirrors Intake's timer/state-machine shape, but there's
    no physical piece to track -- the location itself is the "target",
    and the caller (Robot/Match) is responsible for actually
    materializing a piece once dispensed."""

    def __init__(self):
        self._timer = 0.0
        self._location: Optional["IntakeLocation"] = None

    def update(
        self,
        dt: float,
        commanded_active: bool,
        location: Optional["IntakeLocation"],
        capacity_available: bool,
        duration: float,
    ) -> Optional["IntakeLocation"]:
        if not commanded_active or not capacity_available or location is None:
            self._reset()
            return None

        if location is not self._location:
            self._location = location
            self._timer = 0.0

        self._timer += dt
        if self._timer >= duration:
            dispensed = self._location
            self._reset()
            return dispensed
        return None

    def _reset(self) -> None:
        self._timer = 0.0
        self._location = None

    @property
    def progress(self) -> float:
        return self._timer

    @property
    def target(self) -> Optional["IntakeLocation"]:
        return self._location


class Manipulator:
    """Completes a deposit/launch action after `duration` seconds of
    continuous, commanded-active operation while holding at least one
    piece. Signals completion once; the caller (Robot) is responsible
    for actually releasing/launching the held piece.

    Edge-triggered: once a deposit completes, a further completion is
    withheld until `commanded_active` drops back to False and is
    re-raised. Without this, a robot holding more than one piece at once
    (e.g. coral + algae) that keeps the deposit button held down past the
    first release would immediately start timing -- and eventually eject
    -- its *next* held piece too, since `has_piece` is still True. One
    press should deposit one piece."""

    def __init__(self):
        self._timer = 0.0
        self._awaiting_release = False

    def update(self, dt: float, commanded_active: bool, has_piece: bool, duration: float) -> bool:
        if not commanded_active:
            self._timer = 0.0
            self._awaiting_release = False
            return False

        if not has_piece or self._awaiting_release:
            self._timer = 0.0
            return False

        self._timer += dt
        if self._timer >= duration:
            self._timer = 0.0
            self._awaiting_release = True
            return True
        return False

    @property
    def progress(self) -> float:
        return self._timer

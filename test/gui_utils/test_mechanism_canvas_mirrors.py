"""Guards the hand-mirrored geometry constants in gui_utils/mechanism_canvas.py
against the Java source they mirror (MechanismOrchestrationSandbox, a sibling
repo).

These mirror the sandbox's *default* mechanism specifically. The Java side can
now be handed a different MechanismConfig -- a longer arm, a raised elevator
floor -- and the canvas cannot follow that, since it reads only the joint
positions off NetworkTables and not the shape of the robot producing them. So
what this guards is narrower than it looks: that the picture matches the robot
you get when nobody has passed a config. Drawing a swept mechanism correctly
would mean publishing the dimensions over NT alongside the positions, which is
worth doing before anyone views a non-default robot in this canvas.

There is no shared source of truth across the language boundary -- the Java
side owns the physical constants and the canvas restates them by hand -- and
that was tolerable while the canvas was only drawing an approximate picture.
It stopped being tolerable once the robot's collision check started using the
arm's real cross-section and the piece's carry angle: the viewer is what a
person watches to decide whether an approach is safe, so a mirrored constant
that has drifted doesn't just look slightly wrong, it shows clearance the
robot does not actually have.

Skips (rather than fails) when the sibling repo isn't checked out beside this
one, since it's a separate repository and this suite has to pass without it.
"""
from __future__ import annotations

import os
import re
from pathlib import Path

import pytest

from gui_utils import mechanism_canvas as canvas

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SANDBOX_ENV = "MECHANISM_SANDBOX_PATH"


def _sandbox_java_root() -> Path | None:
    override = os.environ.get(_SANDBOX_ENV)
    candidate = Path(override) if override else _REPO_ROOT.parent / "MechanismOrchestrationSandbox"
    java_root = candidate / "src" / "main" / "java" / "frc" / "robot"
    return java_root if java_root.is_dir() else None


@pytest.fixture(scope="module")
def java_constants() -> dict[str, float]:
    java_root = _sandbox_java_root()
    if java_root is None:
        pytest.skip(
            f"MechanismOrchestrationSandbox not found beside this repo; set {_SANDBOX_ENV} to check mirrors"
        )
    # Only literal `= <number>;` declarations -- a constant that became a derived
    # expression on the Java side can't be mirrored by a literal here anyway, and
    # silently skipping it would be worse than the KeyError the tests below raise.
    pattern = re.compile(r"\b(\w+)\s*=\s*(-?\d+\.?\d*)\s*;")
    found: dict[str, float] = {}
    # The mechanism's dimensions now live in one place on the Java side (config/MechanismConfig,
    # as the DEFAULT_* literals its default instance is assembled from), rather than scattered
    # across the subsystems that happened to use each one. The field geometry stays in game/Shelf,
    # which is right: the wall is not part of the robot and is not swept.
    for name in ("config/MechanismConfig", "game/Shelf"):
        source = (java_root / f"{name}.java").read_text(encoding="utf-8")
        for const, value in pattern.findall(source):
            found.setdefault(f"{Path(name).name}.{const}", float(value))
    return found


@pytest.mark.parametrize(
    ("java_name", "python_value"),
    [
        ("MechanismConfig.DEFAULT_ARM_LENGTH_M", canvas.ARM_LENGTH_M),
        ("MechanismConfig.DEFAULT_ARM_THICKNESS_M", canvas.ARM_THICKNESS_M),
        ("MechanismConfig.DEFAULT_ELEVATOR_MAX_HEIGHT_M", canvas.ELEVATOR_MAX_HEIGHT_M),
        ("MechanismConfig.DEFAULT_CHASSIS_HEIGHT_M", canvas.CHASSIS_HEIGHT_M),
        ("MechanismConfig.DEFAULT_PIECE_SIZE_M", canvas.PIECE_SIZE_M),
        ("Shelf.WALL_NEAR_X", canvas.WALL_NEAR_X),
        ("Shelf.WALL_FAR_X", canvas.WALL_FAR_X),
        ("Shelf.WALL_TOP_Z", canvas.WALL_TOP_Z),
        ("Shelf.LOW_SLOT_MIN_Z", canvas.LOW_SLOT_Z[0]),
        ("Shelf.LOW_SLOT_MAX_Z", canvas.LOW_SLOT_Z[1]),
        ("Shelf.HIGH_SLOT_MIN_Z", canvas.HIGH_SLOT_Z[0]),
        ("Shelf.HIGH_SLOT_MAX_Z", canvas.HIGH_SLOT_Z[1]),
    ],
)
def test_canvas_mirrors_java_constant(java_constants, java_name, python_value):
    assert java_constants[java_name] == pytest.approx(python_value), (
        f"{java_name} has drifted from the canvas mirror: "
        f"Java says {java_constants[java_name]}, canvas draws {python_value}"
    )


def test_chassis_width_mirrors_the_half_width_java_uses(java_constants):
    # Java stores the half-width (it is a clearance limit there); the canvas stores
    # the full width (it is a rectangle to draw). Same number, different convention,
    # so this one can't go through the table above.
    assert java_constants["MechanismConfig.DEFAULT_CHASSIS_HALF_WIDTH_M"] == pytest.approx(
        canvas.CHASSIS_WIDTH_M / 2
    )

"""Keeps the app windows' tests off this machine's actual USB ports.

`MatchView` and `SalvageWindow` both construct a real `GamepadInput` at
init and sum it into every `_tick` via `CombinedInput`, so on any
machine with a controller attached the physical sticks and buttons are
a live input to every test that ticks one of these windows. A stick
drifting past `GamepadInput.DEADBAND` (0.12, ordinary for a worn
controller) is enough to drive the robot out of the zone a test just
asserted it was standing in; the failure then surfaces somewhere else
entirely -- an empty intake, a sound that didn't play -- and reads like
a bug in the sim.

Patching the name in each app module rather than the class itself is
deliberate: it neutralizes only the windows' own auto-detected device,
and leaves tests that build a `GamepadInput(joystick=FakeJoystick())`
on purpose working exactly as written.
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class _NoGamepad:
    """An input source that is definitively not there. `CombinedInput`
    short-circuits on `available`, so nothing downstream polls it."""

    available = False

    def poll(self):
        from common_sim.control.input_sources import DriveCommand, OperatorCommand

        return DriveCommand(), OperatorCommand()


@pytest.fixture(autouse=True)
def _no_real_gamepad(monkeypatch):
    for module_name in ("apps.run_reefscape", "apps.run_salvage"):
        try:
            module = __import__(module_name, fromlist=["GamepadInput"])
        except Exception:  # pragma: no cover - app deps missing is its own failure
            continue
        monkeypatch.setattr(module, "GamepadInput", lambda *a, **k: _NoGamepad(), raising=False)
    yield

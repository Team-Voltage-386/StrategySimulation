"""
Short WAV cues for match-lifecycle moments (start of AUTO, start of
TELEOP, start of endgame, pause, match end) -- purely presentational,
fired by apps/run_reefscape.py's MatchView as it ticks; nothing in
common_sim/game_specific depends on this or is aware of it.

Uses the stdlib `winsound` module (Windows-only, matching this repo's
existing Windows-only conventions -- see run.bat) rather than
PyQt5.QtMultimedia's QSoundEffect: QSoundEffect was tried first, but its
Windows Media Foundation-backed loading gets stuck permanently in the
"Loading" status (never reaches Ready, and never errors either) once a
real QtWidgets.QApplication is running -- reproduced with a
QSoundEffect as the only object in an otherwise-empty QApplication, so
it's not something this app's own code can work around. Confirmed the
identical QSoundEffect code loads fine under a bare QCoreApplication,
which is exactly why it looked fine in isolation while staying silent
in the actual GUI app. `winsound.PlaySound` has no such async
loading/decoding pipeline to get stuck in -- it plays a WAV directly via
the Win32 waveOut API.
"""
from __future__ import annotations

from pathlib import Path

try:
    import winsound
    _WINSOUND_AVAILABLE = True
except ImportError:
    _WINSOUND_AVAILABLE = False

# Cue name -> filename under the assets directory passed to MatchSoundboard.
SOUND_FILES = {
    "start_auto": "Start Auto_normalized.wav",
    "start_teleop": "Start Teleop_normalized.wav",
    "start_endgame": "Start of End Game_normalized.wav",
    "pause": "Match Pause_normalized.wav",
    "match_end": "Match End_normalized.wav",
}


class MatchSoundboard:
    """`play(cue)` for any of SOUND_FILES' keys. Missing winsound (a
    non-Windows machine), a missing assets directory, or a missing
    individual file all degrade to a silent no-op for that cue rather
    than raising -- a match should never fail to run because a sound
    clip didn't load."""

    def __init__(self, assets_dir: Path):
        self._paths: dict[str, str] = {}
        if not _WINSOUND_AVAILABLE:
            return
        for cue, filename in SOUND_FILES.items():
            path = assets_dir / filename
            if path.is_file():
                self._paths[cue] = str(path)

    def play(self, cue: str) -> None:
        path = self._paths.get(cue)
        if path is None:
            return
        # SND_ASYNC returns immediately rather than blocking the GUI
        # thread; SND_NODEFAULT means a lookup miss stays silent instead
        # of falling back to the Windows default "ding". A new call
        # while a previous cue is still playing cuts it off and starts
        # the new one -- winsound only drives one stream at a time --
        # which is fine here since these cues fire at genuinely distinct
        # match moments and are not expected to overlap.
        try:
            winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
        except RuntimeError:
            # No audio device (e.g. a CI runner) -- same "never break the
            # match over a sound cue" contract as every other failure
            # mode here.
            pass

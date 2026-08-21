"""MatchSoundboard: loads the 5 match-cue WAVs from an assets directory
and plays them by name, degrading to a silent no-op for anything
missing (or a machine with no audio device) rather than raising -- a
match should never fail to run because a sound clip didn't load.
"""
from __future__ import annotations

from pathlib import Path

from gui_utils.match_sounds import SOUND_FILES, MatchSoundboard

REPO_ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"


def test_all_cue_files_exist_in_the_real_assets_dir():
    """Guards SOUND_FILES' filenames against a rename/typo -- every cue
    this class knows about must resolve to a real file in assets/."""
    for filename in SOUND_FILES.values():
        assert (REPO_ASSETS_DIR / filename).is_file()


def test_loads_and_plays_every_cue_without_raising():
    board = MatchSoundboard(REPO_ASSETS_DIR)
    assert set(board._paths) == set(SOUND_FILES)
    for cue in SOUND_FILES:
        board.play(cue)


def test_unknown_cue_is_a_silent_no_op():
    board = MatchSoundboard(REPO_ASSETS_DIR)
    board.play("not_a_real_cue")


def test_missing_assets_dir_is_a_silent_no_op(tmp_path):
    board = MatchSoundboard(tmp_path / "does_not_exist")
    assert board._paths == {}
    for cue in SOUND_FILES:
        board.play(cue)

"""Match-cue sound wiring in MatchView: which cue fires at which
lifecycle moment. Swaps in a recording fake for MatchView.sounds so
these don't depend on real audio playback, only on MatchSoundboard's
play(cue) contract (see gui_utils/match_sounds.py).
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

import apps.run_reefscape as run_reefscape  # noqa: E402
from common_sim.match.match import Phase  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _RecordingSoundboard:
    def __init__(self):
        self.played: list[str] = []

    def play(self, cue: str) -> None:
        self.played.append(cue)


def test_start_auto_plays_exactly_once_on_first_play(app):
    view = run_reefscape.MatchView()
    view.sounds = _RecordingSoundboard()

    view._toggle_paused()  # PLAY on a fresh match: elapsed == 0.0
    assert view.sounds.played == ["start_auto"]

    view._toggle_paused()  # pause again: must not replay start_auto
    view.match.elapsed = 3.0
    view._toggle_paused()  # resume mid-match: must not replay start_auto
    assert view.sounds.played.count("start_auto") == 1


def test_pause_plays_when_pausing_a_match_already_in_progress(app):
    view = run_reefscape.MatchView()
    view.sounds = _RecordingSoundboard()

    view._toggle_paused()  # PLAY: fires start_auto, not pause
    view.match.elapsed = 5.0
    view._toggle_paused()  # PAUSE mid-match: fires pause
    assert view.sounds.played == ["start_auto", "pause"]


def test_reset_does_not_play_a_pause_cue(app):
    view = run_reefscape.MatchView()
    view.sounds = _RecordingSoundboard()
    view._reset_match()
    assert view.sounds.played == []


def test_match_end_plays_when_tick_detects_the_match_ended(app):
    view = run_reefscape.MatchView()
    view.sounds = _RecordingSoundboard()
    view.paused = False
    view.match.ended = True

    view._tick()

    assert "match_end" in view.sounds.played
    assert view.paused is True


def test_start_teleop_plays_on_the_auto_to_teleop_transition(app):
    view = run_reefscape.MatchView()
    # Fast-forward makes _tick's single physics step deterministic
    # (bypasses the wall-clock accumulator _advance_realtime uses).
    view.roster_panel.ai_primary_check.setChecked(True)
    view.roster_panel.fast_forward_check.setChecked(True)
    view._reset_match()
    view.sounds = _RecordingSoundboard()
    view.paused = False
    view.match.elapsed = view.match.config.auto_duration - 0.001
    view.match.phase = Phase.AUTO

    view._tick()

    assert "start_teleop" in view.sounds.played


def test_start_endgame_plays_once_teleop_crosses_the_threshold(app):
    view = run_reefscape.MatchView()
    view.roster_panel.ai_primary_check.setChecked(True)
    view.roster_panel.fast_forward_check.setChecked(True)
    view._reset_match()
    view.sounds = _RecordingSoundboard()
    view.paused = False
    view.match.phase = Phase.TELEOP
    view.match.elapsed = view.match.config.total_duration - run_reefscape.ENDGAME_SECONDS + 0.001

    view._tick()

    assert view.sounds.played.count("start_endgame") == 1

    view._tick()  # a second tick past the threshold must not replay it
    assert view.sounds.played.count("start_endgame") == 1

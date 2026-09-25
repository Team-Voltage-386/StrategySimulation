"""
Build the user guides for all four REEFSCAPE tabs -- MATCH, STRATEGY,
SWEEP, SEARCH: real screenshots, numbered callouts drawn from the
widgets' own documentation tags, and a reference table generated from
the same tags.

    python -m apps.build_guide                # all four
    python -m apps.build_guide --guide search  # just one
    python -m apps.build_guide --open

Why generate it rather than write it once. A hand-made guide is a
photograph of the GUI on the afternoon someone took it, and the half-life
of that photograph is one refactor. Here the pictures are grabbed from
the live widgets and the numbers beside them come from
`gui_utils/doc_tags.py`, so:

* a control that moves takes its callout with it;
* a control that is renamed or removed cannot leave a stale paragraph
  behind, because the paragraph is the tag;
* adding a control to a tab adds it to that tab's guide, and forgetting
  to document one is visible as a gap rather than invisible.

Prose lives in `docs/*_guide_template.html`, not here -- one template per
tab, all PROSE ONLY. Editing an explanation should never mean editing
Python. Each template fills in with:

    {{SHOT:name}}     an annotated screenshot, inlined as a data URI
    {{TAGS:name}}     the numbered reference list for that screenshot
    {{STYLE}}         docs/guide_style.css, shared by every guide
    {{NAV}}           the pill row linking to the other three guides
    {{GENERATED}}     when it was built, and against what

Screenshots are inlined rather than linked so each finished guide is one
file a student can email, drop on a shared drive, or publish -- with no
image folder to lose on the way.

The SEARCH guide's PROGRESS and RESULT shots are populated with the
numbers from this tool's *first real run* (a 6-generation search over 4
seeds that reported +9.5 points and kept +0.2 on held-out matches) rather
than with invented ones -- the single most useful thing a new student can
be shown about parameter search, so the guide shows it happening.
"""
from __future__ import annotations

import argparse
import base64
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import date
from pathlib import Path

# Must precede the Qt import.
#
# Not on Windows, though, and this is worth knowing before you "fix" it:
# Qt's `offscreen` plugin registers *zero* font families there, so every
# glyph in the screenshots renders as an empty box. The failure is
# entirely silent -- the build succeeds, the images are the right size,
# and the guide is unreadable. On Windows the native plugin is used
# instead and nothing is ever shown; `QWidget.grab()` renders a widget
# that has been laid out but never made visible, so no window appears.
# Elsewhere (CI, SSH, a headless build box) offscreen has fontconfig
# behind it and is both correct and necessary.
if sys.platform != "win32":
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets   # noqa: E402

from common_sim.analysis.hall_of_fame import Payoff                                   # noqa: E402
from common_sim.analysis.metrics import MatchMetrics                                 # noqa: E402
from common_sim.analysis.param_search import Confirmation, Generation, SearchResult  # noqa: E402
from common_sim.analysis.sweep_spec import TrialOutcome                              # noqa: E402
from common_sim.control import strategy_io, strategy_params                          # noqa: E402
from gui_utils import theme                                                          # noqa: E402
from gui_utils.doc_tags import Callout, collect_callouts                             # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DOCS = REPO_ROOT / "docs"
STYLE_SHEET = DOCS / "guide_style.css"
IMAGE_DIR = DOCS / "guide_images"

TAB_SIZE = QtCore.QSize(1500, 860)


# -- guide staging: SEARCH ----------------------------------------------
#
# The measured first run, reproduced so the guide's screenshots show real
# behaviour. Source: benchmarks/README.md and the run recorded there.
DEMO_BASELINE = 212.0
DEMO_GENERATIONS = (
    #  index, best,  mean,  best-so-far, sigma, seconds
    (1, 212.0, 196.0, 212.0, 0.198, 12.5),
    (2, 212.0, 209.0, 212.0, 0.174, 12.3),
    (3, 212.0, 207.0, 212.0, 0.146, 12.5),
    (4, 221.5, 205.9, 221.5, 0.127, 11.9),
    (5, 220.5, 210.0, 221.5, 0.115, 12.5),
    (6, 215.0, 209.8, 221.5, 0.100, 12.7),
)
DEMO_TUNED_VECTOR = (0.000, 1.592, 0.000, 2.254, 6.124)
# The run log recorded this confirmation twice over, to one decimal each
# way: the pair as "209.5 -> 209.8" and the difference as "+0.2". Those
# two facts do not both hold for 209.5 and 209.8 exactly (that is +0.3),
# so the underlying floats were somewhere inside the rounding. This is
# the reconstruction consistent with both.
DEMO_CONFIRMATION = Confirmation(baseline=209.54, tuned=209.76, seeds=12)
DEMO_MATCHES = 148
DEMO_STRATEGY = "cycle_coral"

# A second real run, this time with the hall of fame switched on -- same
# machine, same strategy, an empty archive (so the field was just the six
# hand-written strategies). Recorded rather than invented for the same
# reason as the numbers above: `python -m apps.run_param_search
# --hall-of-fame hall_of_fame.json --hof-sample 1 --generations 2
# --population 2 --seeds 4 --confirm-seeds 8`. It happens to have found no
# real gain either, which is worth keeping in the guide rather than
# swapping for a flattering invented number -- see section 3.
DEMO_HOF_BASELINE = 219.2
DEMO_HOF_GENERATIONS = (
    #  index, best,  mean,  best-so-far, sigma, seconds
    (1, 217.5, 203.2, 219.2, 0.212, 99.3),
    (2, 218.6, 199.4, 219.2, 0.181, 98.9),
)
DEMO_HOF_VECTOR = (0.000, 0.000, 0.000, 0.000, 24.000)
DEMO_HOF_MATCHES = 120
DEMO_HOF_CONFIRMATION = Confirmation(baseline=223.8, tuned=223.8, seeds=8)
# The holdout payoff matrix -- the confirmed numbers, not the search's own
# -- for the same reason VerdictPanel shows the confirmed score in large
# type: this is the honest one.
DEMO_HOF_PAYOFFS = (
    Payoff("full_defense", candidate_score=211.5, opponent_score=0.0),
    Payoff("endgame_defense", candidate_score=230.5, opponent_score=140.0),
    Payoff("auto_then_cycle", candidate_score=224.2, opponent_score=159.2),
    Payoff("cycle_coral", candidate_score=223.8, opponent_score=160.2),
    Payoff("algae_processor", candidate_score=229.8, opponent_score=167.2),
    Payoff("cycle_coral_evasive", candidate_score=222.8, opponent_score=164.5),
)
DEMO_HOF_ARCHIVE_NOTE = "Archived as 'cycle_coral_tuned' (fitness 223.8); 1 strategies now in hall_of_fame.json"


def _demo_search_result(tab):
    """A `SearchResult` for the strategy the tab is showing, carrying the
    first real run's numbers."""
    payload = tab.current_payload()
    refs = strategy_params.continuous_params(payload)
    # Pad or trim against the live strategy file: the recorded run was
    # over five parameters, and if someone edits cycle_coral.json the
    # guide should still build rather than raise from a length mismatch.
    vector = tuple(
        DEMO_TUNED_VECTOR[i] if i < len(DEMO_TUNED_VECTOR) else ref.value
        for i, ref in enumerate(refs))
    return SearchResult(
        payload=strategy_params.with_vector(payload, refs, vector),
        fitness=DEMO_GENERATIONS[-1][3],
        baseline_payload=payload,
        baseline_fitness=DEMO_BASELINE,
        refs=refs,
        vector=vector,
        generations=tuple(
            Generation(index=i, best=b, mean=m, best_so_far=bsf, sigma=s, failures=0, seconds=sec)
            for i, b, m, bsf, s, sec in DEMO_GENERATIONS),
        matches=DEMO_MATCHES,
    )


def _demo_hof_search_result(tab):
    """Same padding logic as `_demo_search_result`, over the hall-of-fame
    run's recorded vector instead."""
    payload = tab.current_payload()
    refs = strategy_params.continuous_params(payload)
    vector = tuple(
        DEMO_HOF_VECTOR[i] if i < len(DEMO_HOF_VECTOR) else ref.value
        for i, ref in enumerate(refs))
    return SearchResult(
        payload=strategy_params.with_vector(payload, refs, vector),
        fitness=DEMO_HOF_GENERATIONS[-1][3],
        baseline_payload=payload,
        baseline_fitness=DEMO_HOF_BASELINE,
        refs=refs,
        vector=vector,
        generations=tuple(
            Generation(index=i, best=b, mean=m, best_so_far=bsf, sigma=s, failures=0, seconds=sec)
            for i, b, m, bsf, s, sec in DEMO_HOF_GENERATIONS),
        matches=DEMO_HOF_MATCHES,
    )


def build_shots_search(app):
    """One annotated screenshot per SEARCH sub-tab, driven through the
    same state a real run leaves behind -- populated plot, populated log,
    populated verdict -- because a screenshot of an empty panel teaches
    nothing about the panel."""
    from apps.search_tab import SearchTab

    tab = SearchTab()
    tab.resize(TAB_SIZE)
    _stage(tab)
    index = tab.setup_panel.strategy_combo.findText(DEMO_STRATEGY)
    if index >= 0:
        tab.setup_panel.strategy_combo.setCurrentIndex(index)
    app.processEvents()

    result = _demo_search_result(tab)
    shots = {}

    tab.right_tabs.setCurrentWidget(tab.parameter_panel)
    app.processEvents()
    shots["setup"] = _grab(tab)

    # From here on the panel shows the settings that actually produced the
    # recorded run -- 6 generations of 6 candidates over 4 matches each --
    # rather than the tab's defaults. Two reasons: the budget readout then
    # agrees with the progress bar, and 4 matches per candidate trips the
    # thin-seeds warning, so the screenshot the guide uses to explain the
    # trap is a screenshot of the tab warning about it.
    tab.setup_panel.seeds_spin.setValue(4)
    tab.setup_panel.generations_spin.setValue(6)
    tab.setup_panel.population_spin.setValue(6)
    tab.setup_panel.confirm_spin.setValue(DEMO_CONFIRMATION.seeds)
    app.processEvents()

    tab.progress_panel.set_baseline(DEMO_BASELINE)
    for record in result.generations:
        tab.progress_panel.append(record)
    tab.setup_panel.set_running(True)
    tab.setup_panel.set_progress(DEMO_MATCHES, 172)
    tab.setup_panel.set_status(
        f"Generation 6: best so far {DEMO_GENERATIONS[-1][3]:.1f}. About 25s left.")
    tab.right_tabs.setCurrentWidget(tab.progress_panel)
    app.processEvents()
    shots["progress"] = _grab(tab)

    tab.setup_panel.set_running(False)
    tab.setup_panel.set_progress(172, 172)
    tab.setup_panel.set_status("Search complete.")
    tab.parameter_panel.set_refs(result.refs, result.vector)
    tab.verdict_panel.set_result(result, DEMO_CONFIRMATION)
    tab.right_tabs.setCurrentWidget(tab.verdict_panel)
    app.processEvents()
    shots["result"] = _grab(tab)

    # One screenshot doing double duty: the left column shows the hall of
    # fame switched on (Opponent greyed out, the archive controls live),
    # and the right panel shows what that field's payoff matrix and
    # exploitability look like once a run finishes.
    tab.setup_panel.hall_of_fame_check.setChecked(True)
    tab.setup_panel.archive_edit.setText("hall_of_fame.json")
    tab.setup_panel.hof_sample_spin.setValue(1)
    # Match the recorded run exactly (2 generations x 2 candidates x 4
    # seeds), so the settings on the left agree with the matches-run
    # figure the verdict panel prints on the right.
    tab.setup_panel.generations_spin.setValue(2)
    tab.setup_panel.population_spin.setValue(2)
    tab.setup_panel.seeds_spin.setValue(4)
    tab.setup_panel.confirm_spin.setValue(DEMO_HOF_CONFIRMATION.seeds)
    app.processEvents()

    hof_result = _demo_hof_search_result(tab)
    tab.parameter_panel.set_refs(hof_result.refs, hof_result.vector)
    tab.verdict_panel.set_result(hof_result, DEMO_HOF_CONFIRMATION)
    tab.verdict_panel.set_payoffs(DEMO_HOF_PAYOFFS, DEMO_HOF_ARCHIVE_NOTE)
    tab.right_tabs.setCurrentWidget(tab.verdict_panel)
    app.processEvents()
    shots["hof"] = _grab(tab)

    return shots


# -- guide staging: MATCH ------------------------------------------------

def build_shots_match(app, highlight_keys=None):
    """MATCH before and after a second robot joins the roster -- the two
    states that between them touch every panel on the tab."""
    from apps.run_reefscape import MatchView

    view = MatchView()
    view.resize(1700, 900)
    _stage(view)
    app.processEvents()

    shots = {"layout": _grab(view, highlight_keys=highlight_keys)}

    # `_add_row` is what the (untagged, closure-only) "+ ADD ROBOT" button
    # itself calls -- there is no other handle to it, so this is the same
    # action a click would trigger, not a shortcut around it.
    view.roster_panel.blue_roster._add_row()
    rows = view.roster_panel.roster_rows("blue")
    if rows:
        row, _config_tab = rows[0]
        index = row.strategy_combo.findText("full_defense")
        if index >= 0:
            row.strategy_combo.setCurrentIndex(index)
    view._reset_match()  # roster edits only take effect on RESET
    app.processEvents()
    shots["roster"] = _grab(view, highlight_keys=highlight_keys)

    return shots


# -- guide staging: STRATEGY ---------------------------------------------

def build_shots_strategy(app, highlight_keys=None):
    """The rule editor and its live graph, side by side as they sit in
    the real STRATEGY tab, with a real strategy loaded so both panels
    have something to show."""
    from apps.reefscape_widgets import STRATEGIES_DIR, build_demo_match
    from gui_utils.strategy_editor import StrategyEditor
    from gui_utils.strategy_graph import StrategyGraphPanel

    editor = StrategyEditor()
    graph = StrategyGraphPanel()
    editor.changed.connect(lambda: graph.set_strategy(editor.current_strategy()))

    match = build_demo_match()
    editor.set_field(match.field)
    editor.set_robots(["PRIMARY"])
    strategy = strategy_io.load_strategy(STRATEGIES_DIR / f"{DEMO_STRATEGY}.json")
    editor._display_strategy("PRIMARY", strategy)   # same call Load.../set_robots use internally
    graph.set_strategy(strategy)
    if strategy.rules:
        editor.select_rule(strategy.rules[0].name)

    splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
    splitter.addWidget(editor)
    splitter.addWidget(graph)
    splitter.setSizes([620, 520])
    splitter.resize(1500, 880)
    _stage(splitter)
    app.processEvents()

    shots = {"editor": _grab(splitter, highlight_keys=highlight_keys)}

    # A live-mode moment: the graph glowing on its active rule with one
    # transition already flashed and logged, so "graph" shows what a
    # student watching a real match sees, not just the static diagram.
    if strategy.rules:
        first = strategy.rules[0].name
        graph.set_active_rule(first)
        graph.record_transition(3.4, None, first, "PiecesHeld")
    app.processEvents()
    shots["graph"] = _grab(splitter, highlight_keys=highlight_keys)

    # `record_transition` leaves a 700ms QTimer running (the flash fading
    # back to a normal pen) that outlives this function's local `graph` --
    # if the widget gets garbage-collected before it fires, the callback
    # touches a deleted C++ object and Qt prints a RuntimeError to stderr.
    # Draining the event loop past 700ms here lets it fire cleanly while
    # the widget is still alive, rather than chasing a "harmless" crash
    # log out of every future guide build.
    loop = QtCore.QEventLoop()
    QtCore.QTimer.singleShot(750, loop.quit)
    loop.exec()

    return shots


# -- guide staging: SWEEP -------------------------------------------------

def _fake_outcome(index: int, seed: int, params: dict, score: float) -> TrialOutcome:
    metrics = MatchMetrics(
        final_scores={"blue": score, "red": score * 0.7},
        pieces_scored=int(score / 6), pieces_intaked=int(score / 5) + 2,
        pieces_deposited=int(score / 5), misses=1,
        cycle_times=[8.0, 7.5], mean_cycle_time=7.75,
        pieces_scored_by_alliance={"blue": int(score / 6), "red": int(score / 9)},
        mean_cycle_time_by_alliance={"blue": 7.75, "red": 9.1},
        protection_fouls_by_alliance={"blue": 0, "red": 0},
        pin_fouls_by_alliance={"blue": 0, "red": 0},
    )
    return TrialOutcome(index=index, seed=seed, params=params, metrics=metrics, duration_s=3.9)


def build_shots_sweep(app, highlight_keys=None):
    """SETUP configured with a two-axis grid (a numeric characteristic
    crossed with strategy), then RESULTS and PLOTS populated with
    synthetic-but-representative rows -- no real match is run, so this
    builds in under a second."""
    from apps.sweep_tab import SweepTab

    tab = SweepTab()
    tab.resize(1650, 920)
    _stage(tab)
    app.processEvents()

    tab.variable_table.add_row()
    speed_row = tab.variable_table._rows[0]
    _select_target(speed_row, "PRIMARY")
    speed_index = speed_row.property_combo.findText("Speed", QtCore.Qt.MatchContains)
    if speed_index >= 0:
        speed_row.property_combo.setCurrentIndex(speed_index)
    speed_row.min_spin.setValue(4.0)
    speed_row.max_spin.setValue(7.0)
    speed_row.points_spin.setValue(4)
    speed_row._on_changed()

    tab.variable_table.add_row()
    strategy_row = tab.variable_table._rows[1]
    _select_target(strategy_row, "PRIMARY")
    strategy_index = strategy_row.property_combo.findText("strategy")
    if strategy_index >= 0:
        strategy_row.property_combo.setCurrentIndex(strategy_index)
        for i in range(strategy_row.choices_list.count()):
            item = strategy_row.choices_list.item(i)
            if item.text() in ("cycle_coral", "full_defense"):
                item.setCheckState(QtCore.Qt.Checked)
    strategy_row._on_changed()
    app.processEvents()

    shots = {"setup": _grab(tab, highlight_keys=highlight_keys)}

    columns = [v.column for v in tab.variable_table.variables()]
    tab.results_model.set_columns(columns)
    tab.plot_panel.set_columns(columns)
    outcomes = []
    index = 0
    for speed in (4.0, 5.0, 6.0, 7.0):
        for strategy_name, base_score in (("cycle_coral", 190.0), ("full_defense", 140.0)):
            for seed in range(3):
                score = base_score + (speed - 4.0) * 9.0 + seed * 3.5
                params = {"PRIMARY.max_speed": speed, "PRIMARY.strategy": strategy_name}
                outcomes.append(_fake_outcome(index, seed, params, score))
                index += 1
    tab.results_model.append_batch(outcomes)
    tab.control_panel.set_total_runs(len(outcomes))
    tab.control_panel.set_status(f"Completed {len(outcomes)}/{len(outcomes)} runs.")
    app.processEvents()
    shots["results"] = _grab(tab, highlight_keys=highlight_keys)

    # `addTab` reparents its widget into the QTabWidget's internal
    # QStackedWidget, so `.parent()` is one level too shallow -- has to
    # walk up to find the QTabWidget itself. SweepTab doesn't keep its
    # own reference (it wires RESULTS/PLOTS and moves on), so this is
    # the only way in from the outside.
    results_tabs = tab.results_table.parent()
    while results_tabs is not None and not isinstance(results_tabs, QtWidgets.QTabWidget):
        results_tabs = results_tabs.parent()
    if results_tabs is not None:
        results_tabs.setCurrentWidget(tab.plot_panel)
    # `request_redraw` only arms a 1-second debounce timer (so a live
    # sweep doesn't repaint on every row); a screenshot can't wait a
    # second for a QTimer, so this calls what that timer would have
    # called.
    tab.plot_panel._do_redraw()
    app.processEvents()
    shots["plots"] = _grab(tab, highlight_keys=highlight_keys)

    return shots


# -- guide staging: STUDENT LABS -----------------------------------------

def build_shots_labs(app):
    """Four annotated, live UI states used by the step-by-step lab guide.

    Keep this as a composition of the tab-guide staging functions rather
    than drawing pretend controls.  A lab instruction should show precisely
    the controls a student will find in the running application, and a UI
    refactor then updates both the reference guide and the lab guide.
    """
    # One callout per image. The tab handbooks deliberately document every
    # control; lab images are procedural and should point only to the next
    # interaction in the written step beside them.
    match = build_shots_match(app, highlight_keys={"ai_primary"})
    strategy = build_shots_strategy(app, highlight_keys={"apply"})
    sweep = build_shots_sweep(app, highlight_keys={"execute"})
    # Lab 4 is command-line on purpose.  Capture a terminal-like panel with
    # the exact successful output rather than showing an unrelated SEARCH
    # tab, which would suggest that clicking SEARCH runs release regressions.
    terminal = QtWidgets.QFrame()
    terminal.setObjectName("guideTerminal")
    terminal.setStyleSheet(
        "QFrame#guideTerminal { background: #05080d; border: 1px solid #173142; }"
        "QLabel { color: #d6faff; font-family: Consolas; font-size: 15px; padding: 22px; }"
    )
    terminal_layout = QtWidgets.QVBoxLayout(terminal)
    terminal_layout.setContentsMargins(0, 0, 0, 0)
    terminal_text = QtWidgets.QLabel(
        "Windows PowerShell\n"
        "PS C:\\StrategySimulation> python -m apps.run_regression_scenarios\n\n"
        "PASS single_coral_cycle: A single AI completes repeated CORAL cycles on the real field.\n"
        "PASS cycler_vs_defense: A cycling robot and an opposing defender share the real field safely.\n\n"
        "PS C:\\StrategySimulation>"
    )
    terminal_text.setWordWrap(True)
    terminal_layout.addWidget(terminal_text)
    terminal.resize(1380, 280)
    _stage(terminal)
    app.processEvents()
    return {
        "match": match["layout"],
        "strategy": strategy["graph"],
        "sweep": sweep["results"],
        "regression": _grab(terminal),
    }


def _select_target(row, label: str) -> None:
    index = row.target_combo.findText(label)
    if index >= 0:
        row.target_combo.setCurrentIndex(index)
        row._refresh_properties()


# -- shared staging / capture -------------------------------------------

def _stage(widget) -> None:
    """WA_DontShowOnScreen + show() is what gets a complete layout
    without a window appearing. Merely calling `layout().activate()` is
    not enough: nested layouts inside scroll areas and group boxes stay
    unresolved, so `mapTo` reports stale geometry and callouts get drawn
    in the wrong places -- which looks like a bug in the annotator
    rather than in the layout."""
    widget.setAttribute(QtCore.Qt.WA_DontShowOnScreen, True)
    widget.show()


def _grab(widget, *, highlight_keys=None):
    callouts = collect_callouts(widget)
    if highlight_keys is not None:
        callouts = [callout for callout in callouts if callout.key in highlight_keys]
        # A lab screenshot can show only one or two selected controls. Renumber
        # them from 1 so it never has a mysterious badge such as "17" with no
        # preceding sixteen annotations.
        callouts = [
            Callout(number=index, tag=callout.tag, rect=callout.rect)
            for index, callout in enumerate(callouts, start=1)
        ]
    return annotate(widget.grab(), callouts), callouts


# -- annotation --------------------------------------------------------

BADGE_RADIUS = 13


def annotate(pixmap, callouts):
    """Outline each documented widget and number it.

    Badges sit just outside the widget's top-right corner, clamped back
    inside the image so a control flush against the panel edge does not
    get its number cropped off.
    """
    canvas = QtGui.QPixmap(pixmap)
    painter = QtGui.QPainter(canvas)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    accent = QtGui.QColor(theme.ACCENT_CYAN)
    ink = QtGui.QColor(theme.BG_DEEP)

    for callout in callouts:
        rect = callout.rect.adjusted(-2, -2, 2, 2)
        painter.setPen(QtGui.QPen(accent, 2))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRoundedRect(rect, 4, 4)

        # Not the top-left, which is where the control's own caption sits
        # in a QFormLayout -- a badge there covers the very label the
        # reader is looking for.
        width = canvas.width() / canvas.devicePixelRatio()
        height = canvas.height() / canvas.devicePixelRatio()
        centre = QtCore.QPoint(
            max(BADGE_RADIUS, min(rect.right() + BADGE_RADIUS, int(width) - BADGE_RADIUS)),
            max(BADGE_RADIUS, min(rect.top(), int(height) - BADGE_RADIUS)),
        )
        painter.setBrush(accent)
        painter.setPen(QtGui.QPen(accent, 1))
        painter.drawEllipse(centre, BADGE_RADIUS, BADGE_RADIUS)

        font = theme.technical_font(11, bold=True)
        painter.setFont(font)
        painter.setPen(ink)
        text_rect = QtCore.QRect(
            centre.x() - BADGE_RADIUS, centre.y() - BADGE_RADIUS, BADGE_RADIUS * 2, BADGE_RADIUS * 2)
        painter.drawText(text_rect, QtCore.Qt.AlignCenter, str(callout.number))

    painter.end()
    return canvas


def to_data_uri(pixmap) -> str:
    buffer = QtCore.QBuffer()
    buffer.open(QtCore.QIODevice.WriteOnly)
    pixmap.save(buffer, "PNG")
    encoded = base64.b64encode(bytes(buffer.data())).decode("ascii")
    return f"data:image/png;base64,{encoded}"


# -- rendering ---------------------------------------------------------

def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def render_tags(callouts) -> str:
    """The numbered reference list for one screenshot, straight from the
    tags -- title, the tooltip sentence, and the longer explanation that
    would not fit in a tooltip."""
    rows = []
    for callout in callouts:
        tag = callout.tag
        detail = f'<p class="detail">{_escape(tag.detail)}</p>' if tag.detail else ""
        rows.append(
            f'<div class="entry" id="control-{_escape(tag.key)}">'
            f'<div class="num">{callout.number}</div>'
            f'<div class="entry-body"><h4>{_escape(tag.title)}</h4>'
            f'<p>{_escape(tag.body)}</p>{detail}</div></div>')
    return "\n".join(rows)


# -- guide registry -------------------------------------------------------

@dataclass(frozen=True)
class GuideSpec:
    key: str                # --guide value
    nav_label: str           # text on the cross-guide nav pill
    tab_name: str             # what the guide calls the tab in generated prose
    template: str              # filename in docs/
    output: str                  # filename in docs/
    shots_fn: object               # (app) -> {name: (pixmap, callouts)}


GUIDES = (
    GuideSpec("match", "MATCH", "MATCH", "match_guide_template.html", "match_guide.html",
              build_shots_match),
    GuideSpec("strategy", "STRATEGY", "STRATEGY", "strategy_guide_template.html",
              "strategy_guide.html", build_shots_strategy),
    GuideSpec("sweep", "SWEEP", "SWEEP", "sweep_guide_template.html", "sweep_guide.html",
              build_shots_sweep),
    GuideSpec("search", "SEARCH", "SEARCH", "guide_template.html", "param_search_guide.html",
              build_shots_search),
    GuideSpec("labs", "LABS", "STUDENT LABS", "student_labs_template.html", "student_labs_guide.html",
              build_shots_labs),
)


def _nav_html(current_key: str) -> str:
    links = []
    for spec in GUIDES:
        current = ' aria-current="page"' if spec.key == current_key else ""
        links.append(f'<a href="{spec.output}"{current}>{spec.nav_label}</a>')
    return '<nav class="crossnav">\n  ' + "\n  ".join(links) + "\n</nav>"


def build_one(spec: GuideSpec, app, style_css: str) -> Path:
    template_path = DOCS / spec.template
    if not template_path.exists():
        raise SystemExit(f"missing {template_path} -- the {spec.tab_name} guide's prose lives there")

    shots = spec.shots_fn(app)

    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    html = template_path.read_text(encoding="utf-8")
    # The template's HTML comments are instructions to whoever edits it --
    # including a literal list of the placeholder names, which would trip
    # the unfilled-placeholder check below. They are not guide content, so
    # they come out before anything else happens.
    html = re.sub(r"<!--.*?-->", "", html, flags=re.DOTALL)
    html = html.replace("{{STYLE}}", f"<style>\n{style_css}\n</style>")
    html = html.replace("{{NAV}}", _nav_html(spec.key))

    for name, (pixmap, callouts) in shots.items():
        # Written to disk as well as inlined: the loose PNGs are what you
        # look at when a callout lands in the wrong place.
        pixmap.save(str(IMAGE_DIR / f"{spec.key}_{name}.png"), "PNG")
        html = html.replace(
            "{{SHOT:%s}}" % name,
            f'<img class="shot" alt="The {spec.tab_name} tab, {name}" src="{to_data_uri(pixmap)}">')
        html = html.replace("{{TAGS:%s}}" % name, render_tags(callouts))

    documented = sum(len(c) for _, c in shots.values())
    html = html.replace("{{GENERATED}}", (
        f"Generated {date.today().isoformat()} by <code>python -m apps.build_guide</code> "
        f"from the live {spec.tab_name} tab -- {documented} documented controls across "
        f"{len(shots)} screenshots."))

    leftover = [line for line in html.splitlines() if "{{" in line]
    if leftover:
        raise SystemExit(
            f"{spec.template} has placeholders this build did not fill:\n  "
            + "\n  ".join(leftover[:5])
            + "\nEither the screenshot name is misspelled or its shots_fn no longer produces it.")

    output_path = DOCS / spec.output
    output_path.write_text(html, encoding="utf-8")
    size_kb = output_path.stat().st_size / 1024
    print(f"wrote {output_path.relative_to(REPO_ROOT)}  ({size_kb:,.0f} KB, images inlined)")
    return output_path


def build(keys=None, open_after: bool = False) -> list[Path]:
    if not STYLE_SHEET.exists():
        raise SystemExit(f"missing {STYLE_SHEET} -- shared CSS every guide inlines")
    style_css = STYLE_SHEET.read_text(encoding="utf-8")

    wanted = set(keys) if keys else {spec.key for spec in GUIDES}
    unknown = wanted - {spec.key for spec in GUIDES}
    if unknown:
        raise SystemExit(f"unknown --guide value(s): {', '.join(sorted(unknown))}")

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)

    outputs = [build_one(spec, app, style_css) for spec in GUIDES if spec.key in wanted]
    print(f"{IMAGE_DIR.relative_to(REPO_ROOT)}/  holds every guide's annotated PNGs")

    if open_after:
        for path in outputs:
            _open(path)
    return outputs


def _open(path: Path) -> None:
    if sys.platform == "win32":
        os.startfile(path)                       # noqa: S606 -- a local HTML file we just wrote
    else:
        subprocess.run(["xdg-open" if sys.platform.startswith("linux") else "open", str(path)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--guide", action="append", choices=[spec.key for spec in GUIDES],
                        help="build only this guide (repeatable); default is all four")
    parser.add_argument("--open", action="store_true", help="open the guide(s) when built")
    args = parser.parse_args()
    build(keys=args.guide, open_after=args.open)


if __name__ == "__main__":
    main()

"""
SEARCH tab wiring: the GUI front end for
`common_sim/analysis/param_search.py`.

This is the file that knows both about Qt (gui_utils/search_panel.py) and
about REEFSCAPE (game_specific/reefscape/sweep_trial.run_trial, the
strategies directory, the roster) -- the same seam apps/sweep_tab.py sits
on for the SWEEP tab.

It deliberately reuses `apps/run_param_search.py`'s roster and
variability settings rather than defining its own. A student who runs a
search from this tab and a mentor who runs one from the command line
should be comparing the same quantity; two sets of defaults would mean
two different measurements with one name.

The one thing this tab does that the CLI does not is refuse to report an
unconfirmed number as the answer -- see `VerdictPanel`.
"""
from __future__ import annotations

import json
from pathlib import Path

from pyqtgraph.Qt import QtCore, QtWidgets

from apps.reefscape_widgets import STRATEGIES_DIR, STRATEGY_NAMES
from apps.run_param_search import MATCH, SEARCH_VARIABILITY, build_roster, load_hand_written
from common_sim.analysis.hall_of_fame import Archive, HallOfFameEvaluator
from common_sim.analysis.param_search import (
    AllianceScoreEvaluator, confirm, default_population, search_parameters,
)
from common_sim.control import strategy_io, strategy_params
from game_specific.reefscape import sweep_trial
from gui_utils.doc_tags import document
from gui_utils.search_panel import (
    ParameterPanel, ProgressPanel, SearchRunController, SearchSetupPanel, VerdictPanel,
)

# What one match costs before this machine has been watched doing one.
# Replaced by the measured figure after the first generation, so it only
# has to be the right order of magnitude -- the calibrated number for
# this desktop is about 3.9 s single-core, and searches run in parallel.
INITIAL_SECONDS_PER_MATCH = 4.0

INTRO = (
    "Tune the numbers inside a strategy to fit a particular robot. The rules you wrote stay "
    "exactly as they are -- only their timings and distances move -- so what comes out is a "
    "normal strategy file, not a black box.\n"
    "New here? Start with the guide: docs/param_search_guide.html"
)


class SearchTab(QtWidgets.QWidget):
    """Setup on the left; the tunable numbers, live progress and the
    verdict on the right."""

    running_changed = QtCore.Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._payload = None
        self._refs = ()
        self._result = None
        self._seconds_per_match = None
        self._matches_done = 0
        self._matches_total = 0

        # Tagged before anything else so it is callout 1: doc-tag order is
        # declaration order, which is the order the guide walks the panel in.
        intro = document(
            QtWidgets.QLabel(INTRO), "intro", "What this tab does",
            "A one-paragraph reminder of what a parameter search is for, and a pointer to the "
            "full guide.")
        intro.setWordWrap(True)

        self.setup_panel = SearchSetupPanel(STRATEGY_NAMES)
        self.setup_panel.run_clicked.connect(self._on_run)
        self.setup_panel.stop_clicked.connect(self._on_stop)

        left_content = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_content)
        left_layout.addWidget(intro)
        left_layout.addWidget(self.setup_panel)
        left_layout.addStretch(1)

        left_column = QtWidgets.QScrollArea()
        left_column.setWidget(left_content)
        left_column.setWidgetResizable(True)
        left_column.setMinimumWidth(320)
        left_column.setFrameShape(QtWidgets.QFrame.NoFrame)

        self.parameter_panel = ParameterPanel()
        self.progress_panel = ProgressPanel()
        self.verdict_panel = VerdictPanel()
        self.verdict_panel.save_clicked.connect(self._on_save)

        self.right_tabs = QtWidgets.QTabWidget()
        self.right_tabs.addTab(self.parameter_panel, "WHAT MOVES")
        self.right_tabs.addTab(self.progress_panel, "PROGRESS")
        self.right_tabs.addTab(self.verdict_panel, "RESULT")

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(left_column)
        splitter.addWidget(self.right_tabs)
        splitter.setSizes([380, 900])

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(splitter)

        self.controller = SearchRunController()
        self.controller.generation_ready.connect(self._on_generation)
        self.controller.baseline_ready.connect(self.progress_panel.set_baseline)
        self.controller.note.connect(self._on_note)
        self.controller.finished.connect(self._on_finished)

        # Defaults chosen to match apps/run_param_search.py's, and applied
        # before `changed` is connected -- `_on_config_changed` reads the
        # panels above, so it must not be able to fire while they are still
        # being built.
        _select(self.setup_panel.strategy_combo, "cycle_coral")
        _select(self.setup_panel.partner_combo, "cycle_coral")
        _select(self.setup_panel.opponent_combo, "full_defense")
        self.setup_panel.changed.connect(self._on_config_changed)
        self._on_config_changed()

    # -- configuration -------------------------------------------------

    def _on_config_changed(self) -> None:
        """Reload the chosen strategy's tunable numbers and re-price the
        run. Cheap enough to do on every keystroke -- it reads one small
        JSON file and walks it."""
        name = self.setup_panel.strategy()
        try:
            self._payload = strategy_io.to_dict(
                strategy_io.load_strategy(STRATEGIES_DIR / f"{name}.json"))
            self._refs = strategy_params.continuous_params(self._payload)
        except Exception as exc:                      # noqa: BLE001 -- a bad strategy file, shown not raised
            self._payload, self._refs = None, ()
            self.parameter_panel.set_error(f"Could not read {name}.json: {exc}")
        else:
            self.parameter_panel.set_refs(self._refs)

        matches = self.setup_panel.total_matches(len(self._refs), hand_written_count=len(STRATEGY_NAMES))
        self._matches_total = matches
        seconds = self._estimate_seconds(matches)
        self.setup_panel.set_budget(matches, seconds, estimated=self._seconds_per_match is None)
        self.setup_panel.run_button.setEnabled(bool(self._refs) and not self.controller.is_running())

    def current_payload(self) -> dict | None:
        """The loaded strategy payload, or None if the file would not
        read. Public because `apps/build_guide.py` stages this tab with a
        realistic result to screenshot."""
        return self._payload

    def _estimate_seconds(self, matches: int) -> float:
        per_match = self._seconds_per_match or INITIAL_SECONDS_PER_MATCH
        if self._seconds_per_match is not None:
            # Already a wall-clock figure from this run's own generations,
            # so the worker count is baked in and must not be divided out
            # a second time.
            return matches * per_match
        return matches * per_match / max(1, self.setup_panel.workers())

    # -- running -------------------------------------------------------

    def _on_run(self) -> None:
        if self.controller.is_running() or not self._refs:
            return

        if self.setup_panel.hall_of_fame_enabled() and Path(self.setup_panel.archive_path()).is_dir():
            QtWidgets.QMessageBox.warning(
                self, "Archive path is a folder",
                f"{self.setup_panel.archive_path()!r} is a directory, not a file. Pick (or type) "
                "a .json file name for the archive, e.g. hall_of_fame.json.")
            return

        matches = self.setup_panel.total_matches(len(self._refs), hand_written_count=len(STRATEGY_NAMES))
        if matches > 20_000:
            reply = QtWidgets.QMessageBox.question(
                self, "Long search",
                f"These settings run {matches:,} matches. That is likely to take hours. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
            if reply != QtWidgets.QMessageBox.Yes:
                return

        self._result = None
        self._matches_done = 0
        self._seconds_per_match = None
        # Frozen for the duration: progress is reported against the
        # settings the run started with, not against whatever the form
        # says later.
        self._run_seeds = self.setup_panel.seeds()
        self._run_population = self.setup_panel.population() or default_population(
            max(1, len(self._refs)))
        self.progress_panel.clear()
        self.verdict_panel.clear()
        self.parameter_panel.set_refs(self._refs)
        self.setup_panel.set_running(True)
        self.setup_panel.set_status("Measuring the hand-written strategy first...")
        self.right_tabs.setCurrentWidget(self.progress_panel)
        self.running_changed.emit(True)
        self.controller.start(self._build_job())

    def _build_job(self):
        """A closure carrying everything the worker thread needs, read off
        the form *now* -- the form is disabled during a run, but reading
        widgets from a non-GUI thread is undefined behaviour regardless."""
        payload = self._payload
        strategy = self.setup_panel.strategy()
        partner = self.setup_panel.partner()
        opponent = self.setup_panel.opponent()
        generations = self.setup_panel.generations()
        population = self.setup_panel.population()
        seeds = self.setup_panel.seeds()
        confirm_seeds = self.setup_panel.confirm_seeds()
        sigma = self.setup_panel.sigma()
        rng_seed = self.setup_panel.rng_seed()
        workers = self.setup_panel.workers()
        hall_of_fame = self.setup_panel.hall_of_fame_enabled()
        archive_path = self.setup_panel.archive_path()
        hof_sample = self.setup_panel.hof_sample()

        def job(progress, report_baseline):
            robots = build_roster(strategy, opponent, partner)

            if hall_of_fame:
                hand_written = load_hand_written()
                archive = Archive.load(archive_path)
                evaluator = HallOfFameEvaluator(
                    sweep_trial.run_trial, robots=robots, match=MATCH,
                    variability=SEARCH_VARIABILITY, strategies_dir=STRATEGIES_DIR,
                    dt=sweep_trial.SEARCH_DT, target_label="PRIMARY", opponent_label="OPPONENT",
                    alliance="blue", opponent_alliance="red", archive=archive,
                    hand_written=hand_written, sample_size=hof_sample, seeds=seeds,
                    rng_seed=rng_seed, parallel=True, max_workers=workers,
                )
            else:
                evaluator = AllianceScoreEvaluator(
                    sweep_trial.run_trial, robots=robots, match=MATCH,
                    variability=SEARCH_VARIABILITY, strategies_dir=STRATEGIES_DIR,
                    dt=sweep_trial.SEARCH_DT, target_label="PRIMARY", alliance="blue",
                    seeds=seeds, parallel=True, max_workers=workers,
                )

            result = search_parameters(
                payload, evaluator, generations=generations, sigma=sigma,
                population_size=population, seed=rng_seed, progress=progress,
            )
            report_baseline(result.baseline_fitness)

            confirmation = None
            payoffs = None
            if confirm_seeds:
                # base_seed offset by the search's seed count, so the
                # winner is scored on matches it was never selected
                # against. Without this the confirmation measures nothing.
                progress(f"confirming on {confirm_seeds} fresh matches...")
                if hall_of_fame:
                    holdout = HallOfFameEvaluator(
                        sweep_trial.run_trial, robots=robots, match=MATCH,
                        variability=SEARCH_VARIABILITY, strategies_dir=STRATEGIES_DIR,
                        dt=sweep_trial.SEARCH_DT, target_label="PRIMARY",
                        opponent_label="OPPONENT", alliance="blue", opponent_alliance="red",
                        archive=archive, hand_written=hand_written, sample_size=hof_sample,
                        seeds=confirm_seeds, base_seed=seeds, rng_seed=rng_seed + 1,
                        parallel=True, max_workers=workers,
                    )
                    confirmation = confirm(result, holdout)
                    payoffs = holdout.last_payoffs[1]  # index 1: the tuned strategy, not the baseline
                else:
                    confirmation = confirm(result, AllianceScoreEvaluator(
                        sweep_trial.run_trial, robots=robots, match=MATCH,
                        variability=SEARCH_VARIABILITY, strategies_dir=STRATEGIES_DIR,
                        dt=sweep_trial.SEARCH_DT, target_label="PRIMARY", alliance="blue",
                        seeds=confirm_seeds, base_seed=seeds, parallel=True, max_workers=workers,
                    ))

            archive_note = ""
            if hall_of_fame:
                # Keep the strongest, not the newest -- see Archive.add.
                # The confirmed (held-out) fitness is what gets recorded
                # when one exists, so the archive's own numbers carry the
                # same honesty bar as the tab's headline.
                fitness = confirmation.tuned if confirmation is not None else result.fitness
                name = f"{strategy}_tuned"
                archive = archive.add(name, dict(result.payload, name=name), fitness, max_size=20)
                archive.save(archive_path)
                archive_note = f"Archived as {name!r} (fitness {fitness:.1f}); " \
                                f"{len(archive)} strategies now in {archive_path}"

            return result, confirmation, (payoffs, archive_note)

        return job

    def _on_stop(self) -> None:
        self.setup_panel.set_status("Stopping after this generation finishes...")
        self.controller.abort()

    def _on_generation(self, record) -> None:
        self.progress_panel.append(record)

        per_generation = self._run_population * self._run_seeds
        # Counted from the generation index rather than accumulated, and
        # including the `+1` baseline evaluation, so the bar agrees with
        # the budget it is a fraction of. Accumulating skips the baseline
        # and leaves the bar permanently short of the end.
        self._matches_done = self._run_seeds * (1 + record.index * self._run_population)
        # Wall-clock seconds per match on however many workers this run
        # was given -- the only figure that predicts the rest of the run.
        self._seconds_per_match = record.seconds / max(1, per_generation)
        self.setup_panel.set_progress(self._matches_done, self._matches_total)

        remaining = max(0, self._matches_total - self._matches_done)
        self.setup_panel.set_status(
            f"Generation {record.index}: best so far {record.best_so_far:.1f}. "
            f"About {_humanize(remaining * self._seconds_per_match)} left.")
        self.setup_panel.set_budget(self._matches_total, self._matches_total * self._seconds_per_match,
                                    estimated=False)

    def _on_note(self, text: str) -> None:
        """A free-text progress line -- an early convergence stop, or the
        start of the confirmation run, which is the only part of the
        budget that reports no generations of its own."""
        self.progress_panel.note(text)
        self.setup_panel.set_status(text)

    def _on_finished(self, ok: bool, message: str, result, confirmation, hof) -> None:
        if ok:
            self.setup_panel.set_progress(self._matches_total, self._matches_total)
        self.setup_panel.set_running(False)
        self.setup_panel.set_status(message)
        self.running_changed.emit(False)
        if not ok or result is None:
            return
        self._result = result
        self.parameter_panel.set_refs(result.refs, result.vector)
        self.verdict_panel.set_result(result, confirmation)
        payoffs, archive_note = hof if hof is not None else (None, "")
        self.verdict_panel.set_payoffs(payoffs, archive_note)
        self.right_tabs.setCurrentWidget(self.verdict_panel)

    # -- output --------------------------------------------------------

    def _on_save(self) -> None:
        if self._result is None:
            return
        default = STRATEGIES_DIR / f"{self.setup_panel.strategy()}_tuned.json"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save tuned strategy", str(default), "Strategy files (*.json)")
        if not path:
            return
        name = Path(path).stem
        payload = dict(self._result.payload, name=name)
        Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.setup_panel.set_status(f"Saved {name}.json")

    def shutdown(self) -> None:
        self.controller.shutdown()


def _select(combo, name: str) -> None:
    """Select `name` if the game ships it, otherwise leave the combo on
    its first entry -- a renamed strategy file should not be a crash."""
    index = combo.findText(name)
    if index >= 0:
        combo.setCurrentIndex(index)


def _humanize(seconds: float) -> str:
    if seconds < 90:
        return f"{seconds:.0f}s"
    if seconds < 5400:
        return f"{seconds / 60:.0f} minutes"
    return f"{seconds / 3600:.1f} hours"

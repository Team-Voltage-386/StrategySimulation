"""
Generic (game-agnostic) SEARCH-tab widgets: the setup form, the table of
parameters a search is allowed to move, the live progress plot, the
verdict panel, and the QThread plumbing that runs a search without
freezing the window.

Every control here carries a `doc_tags.document(...)` explanation. That
is not decoration -- `apps/build_guide.py` reads those tags back to draw
the numbered callouts on the user guide's screenshots and to write the
reference table under them, so an undocumented control is one that
silently vanishes from the guide.

The audience is a first-year student who has never heard of CMA-ES, and
two decisions follow from that:

* **The headline number is the held-out one.** A search's own best score
  is a maximum over hundreds of candidates scored on the same handful of
  seeds, so it is optimistic by construction -- the first real run of
  this tool reported +9.5 points and kept +0.2. `VerdictPanel` therefore
  prints the confirmed figure large and the search's own figure small and
  dim, which is the opposite of what a progress bar naturally wants to
  do, and is the single most important thing this panel does.
* **The budget is priced before it is spent, in matches and in minutes.**
  A student raising `Generations` from 12 to 40 should see the cost
  change as they type it, not discover it at midnight.

This module never imports `game_specific`: the strategy names, the
evaluator, and the worker that runs a match all arrive from
`apps/search_tab.py`.
"""
from __future__ import annotations

import random

from pyqtgraph.Qt import QtCore, QtWidgets

import matplotlib
matplotlib.use("Qt5Agg")  # after pyqtgraph.Qt, so matplotlib binds to the already-loaded Qt binding
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from common_sim.analysis.hall_of_fame import describe_payoffs
from common_sim.analysis.param_search import default_population, matches_required
from common_sim.analysis.runner import default_worker_count
from common_sim.control import strategy_params
from gui_utils import theme
from gui_utils.doc_tags import document

# Above this, a run is an overnight commitment rather than a coffee break
# and the readout says so in amber; above the second, it asks first.
AMBER_MATCH_THRESHOLD = 2_000
RED_MATCH_THRESHOLD = 20_000

# Below this many seeds per candidate, the seed-to-seed noise in a
# REEFSCAPE score is comparable to the effect a parameter search is
# chasing, and the search spends its budget ranking luck. Measured, not
# guessed: see benchmarks/README.md.
THIN_SEED_THRESHOLD = 8


class SearchSetupPanel(QtWidgets.QWidget):
    """WHAT TO TUNE / HOW HARD TO LOOK / the budget, plus RUN and STOP.

    Grouped in the order a person makes the decisions rather than the
    order the API takes them: which match, then how much compute, then
    what that costs.
    """

    run_clicked = QtCore.Signal()
    stop_clicked = QtCore.Signal()
    changed = QtCore.Signal()

    def __init__(self, strategy_names, parent=None):
        super().__init__(parent)
        names = list(strategy_names)
        self._running = False

        subject = QtWidgets.QGroupBox("1.  WHAT TO TUNE")
        subject_form = QtWidgets.QFormLayout(subject)

        self.strategy_combo = document(
            QtWidgets.QComboBox(), "strategy", "Strategy to tune",
            "The strategy whose numbers get adjusted. Its rules, triggers and tactics are "
            "never changed -- only the numbers inside them.",
            "Pick the strategy you would actually run. The search starts from the numbers "
            "already in the file, so a sensible starting point is worth a lot of compute.")
        self.strategy_combo.addItems(names)
        self.strategy_combo.currentTextChanged.connect(self.changed)
        subject_form.addRow("Strategy", self.strategy_combo)

        self.partner_combo = document(
            QtWidgets.QComboBox(), "partner", "Alliance partner",
            "Your second blue robot. It is held fixed -- only the tuned robot changes.",
            "Tuning happens in a 2v1 rather than an empty field on purpose: a strategy tuned "
            "with nobody else on the field learns timings that fall apart the moment another "
            "robot wants the same game piece.")
        self.partner_combo.addItems(names)
        self.partner_combo.currentTextChanged.connect(self.changed)
        subject_form.addRow("Partner", self.partner_combo)

        self.opponent_combo = document(
            QtWidgets.QComboBox(), "opponent", "Opponent",
            "The red robot you are tuning against.",
            "This matters more than it looks. Numbers tuned against a defensive robot are not "
            "the numbers you want against a robot that ignores you, so tune against the "
            "opponent you expect to face. Ignored when hall of fame (below) is on.")
        self.opponent_combo.addItems(names)
        self.opponent_combo.currentTextChanged.connect(self.changed)
        subject_form.addRow("Opponent", self.opponent_combo)

        self.hall_of_fame_check = document(
            QtWidgets.QCheckBox("Grade against a hall of fame instead"), "hall_of_fame",
            "Hall-of-fame field",
            "Judge every candidate against a sampled field -- past winners plus every "
            "hand-written strategy -- instead of the single Opponent above, and report how "
            "much the single best counter in that field would beat the winner by.",
            "A strategy tuned against one fixed opponent discovers counters to that opponent, "
            "and its score cannot tell you that's what happened. Turning this on costs several "
            "times as many matches per candidate -- the archive sample size plus the number of "
            "strategy files in this game.")
        self.hall_of_fame_check.toggled.connect(self._update_hof_enabled)
        self.hall_of_fame_check.toggled.connect(self.changed)
        subject_form.addRow(self.hall_of_fame_check)

        self.archive_edit = QtWidgets.QLineEdit()
        # A real default value, not just placeholder ghost text: an empty
        # QLineEdit.text() is "", and Path("") normalizes to ".", the
        # current directory -- Archive.load/save would then try to read
        # or write a *directory* as JSON and fail with a "permission
        # denied '.'" error that gives no hint what went wrong.
        self.archive_edit.setText("hall_of_fame.json")
        self.archive_edit.textChanged.connect(self.changed)
        self.archive_browse = QtWidgets.QPushButton("Browse...")
        self.archive_browse.clicked.connect(self._on_browse_archive)
        archive_row = QtWidgets.QHBoxLayout()
        archive_row.setContentsMargins(0, 0, 0, 0)
        archive_row.addWidget(self.archive_edit)
        archive_row.addWidget(self.archive_browse)
        archive_row_widget = QtWidgets.QWidget()
        archive_row_widget.setLayout(archive_row)
        archive_row_widget = document(
            archive_row_widget, "hof_archive", "Archive file",
            "Where past winners are kept, as JSON. Created if it does not exist yet; grows by "
            "one entry -- the winner -- every time a hall-of-fame search finishes.",
            "Point every search at the same file and the field gets harder to beat over time. "
            "A file that does not exist yet starts with just the hand-written strategies in "
            "the field.")
        subject_form.addRow("Archive", archive_row_widget)

        self.hof_sample_spin = document(
            QtWidgets.QSpinBox(), "hof_sample", "Archive opponents sampled",
            "How many past winners are drawn from the archive into the field each generation, "
            "on top of every hand-written strategy.",
            "Re-sampled each generation so the search cannot overfit to one lucky subset of "
            "the archive. 4 is a reasonable start; field cost is this number plus the "
            "hand-written count, per candidate.")
        self.hof_sample_spin.setRange(0, 50)
        self.hof_sample_spin.setValue(4)
        self.hof_sample_spin.valueChanged.connect(self.changed)
        subject_form.addRow("Archive sample size", self.hof_sample_spin)

        self._update_hof_enabled()

        effort = QtWidgets.QGroupBox("2.  HOW HARD TO LOOK")
        effort_form = QtWidgets.QFormLayout(effort)

        self.seeds_spin = document(
            QtWidgets.QSpinBox(), "seeds", "Matches per candidate",
            "How many matches each candidate strategy is judged on. This is the most "
            "important setting on the panel.",
            "Two identical strategies score differently from match to match, because robots "
            "and game pieces are randomised. Judge a candidate on too few matches and you "
            "measure luck. If a search's gain keeps disappearing when it is confirmed, this "
            "is the number to raise -- not Generations.")
        self.seeds_spin.setRange(1, 200)
        self.seeds_spin.setValue(16)
        self.seeds_spin.valueChanged.connect(self.changed)
        effort_form.addRow("Matches per candidate", self.seeds_spin)

        self.generations_spin = document(
            QtWidgets.QSpinBox(), "generations", "Generations",
            "How many rounds of guessing the optimiser gets. Each round tries a fresh batch "
            "of candidates and learns from how they scored.",
            "More rounds means a more thorough look, but a thorough look at a noisy "
            "measurement just finds noise more thoroughly. Get 'Matches per candidate' high "
            "enough first.")
        self.generations_spin.setRange(1, 500)
        self.generations_spin.setValue(12)
        self.generations_spin.valueChanged.connect(self.changed)
        effort_form.addRow("Generations", self.generations_spin)

        self.population_spin = document(
            QtWidgets.QSpinBox(), "population", "Candidates per generation",
            "How many candidate strategies to try each round. 0 means 'let the optimiser "
            "choose', which is almost always right.",
            "The default comes from CMA-ES's own formula and depends on how many numbers "
            "are being tuned. Override it only if you have a reason.")
        self.population_spin.setRange(0, 200)
        self.population_spin.setSpecialValueText("auto")
        self.population_spin.setValue(0)
        self.population_spin.valueChanged.connect(self.changed)
        effort_form.addRow("Candidates / generation", self.population_spin)

        self.confirm_spin = document(
            QtWidgets.QSpinBox(), "confirm_seeds", "Confirmation matches",
            "Fresh matches used to re-score the winner at the end. Set this to 0 only if you "
            "do not care whether the result is real.",
            "The search picks a winner out of hundreds of candidates, all scored on the same "
            "matches, so the winner is partly just the luckiest. Re-running it on matches it "
            "has never seen is what separates a real improvement from a lucky one -- and it "
            "is routinely most of the gain.")
        self.confirm_spin.setRange(0, 500)
        self.confirm_spin.setValue(32)
        self.confirm_spin.valueChanged.connect(self.changed)
        effort_form.addRow("Confirmation matches", self.confirm_spin)

        advanced = QtWidgets.QGridLayout()
        self.sigma_spin = document(
            QtWidgets.QDoubleSpinBox(), "sigma", "Initial step size",
            "How far the first round of guesses strays from the strategy's current numbers, "
            "as a fraction of each number's allowed range.",
            "0.25 means the first guesses are scattered about a quarter of each slider's "
            "travel away from where you started. The optimiser shrinks this on its own as it "
            "homes in.")
        self.sigma_spin.setRange(0.01, 1.0)
        self.sigma_spin.setSingleStep(0.05)
        self.sigma_spin.setValue(0.25)
        self.sigma_spin.valueChanged.connect(self.changed)
        advanced.addWidget(_caption("Step size"), 0, 0)
        advanced.addWidget(self.sigma_spin, 0, 1)

        self.rng_spin = document(
            QtWidgets.QSpinBox(), "rng_seed", "Random seed",
            "Fixes the optimiser's random guesses, so the same settings give the same run "
            "twice.",
            "Useful when you want to show someone else exactly what you saw. Change it to get "
            "an independent second opinion on the same question.")
        self.rng_spin.setRange(0, 2_000_000_000)
        self.rng_spin.valueChanged.connect(self.changed)
        randomize = QtWidgets.QPushButton("Randomize")
        randomize.clicked.connect(lambda: self.rng_spin.setValue(random.randint(0, 2_000_000_000)))
        advanced.addWidget(_caption("Random seed"), 1, 0)
        advanced.addWidget(self.rng_spin, 1, 1)
        advanced.addWidget(randomize, 1, 2)

        self.workers_spin = document(
            QtWidgets.QSpinBox(), "workers", "CPU workers",
            "How many matches run at once. Defaults to what this computer has.",
            "Turn it down if you want to keep using the machine for something else while a "
            "search runs.")
        self.workers_spin.setRange(1, 64)
        self.workers_spin.setValue(default_worker_count())
        self.workers_spin.valueChanged.connect(self.changed)
        advanced.addWidget(_caption("CPU workers"), 2, 0)
        advanced.addWidget(self.workers_spin, 2, 1)
        effort_form.addRow(advanced)

        budget = QtWidgets.QGroupBox("3.  WHAT IT COSTS")
        budget_layout = QtWidgets.QVBoxLayout(budget)
        self.budget_label = document(
            QtWidgets.QLabel("--"), "budget", "Budget readout",
            "How many matches these settings will run, and roughly how long that takes on "
            "this computer.",
            "The time estimate starts as a guess and sharpens once the first generation has "
            "actually finished. Amber means this is an evening; red means ask first.")
        self.budget_label.setFont(theme.technical_font(13, bold=True))
        self.budget_label.setWordWrap(True)
        budget_layout.addWidget(self.budget_label)

        self.warning_label = QtWidgets.QLabel("")
        self.warning_label.setStyleSheet(f"color: {theme.ACCENT_AMBER};")
        self.warning_label.setWordWrap(True)
        self.warning_label.setVisible(False)
        budget_layout.addWidget(self.warning_label)

        run_row = QtWidgets.QHBoxLayout()
        self.run_button = document(
            QtWidgets.QPushButton("RUN SEARCH"), "run", "Run",
            "Starts the search. The window stays usable while it runs.")
        self.run_button.clicked.connect(self.run_clicked)
        self.stop_button = document(
            QtWidgets.QPushButton("STOP"), "stop", "Stop",
            "Stops after the round that is currently running finishes.",
            "It cannot interrupt matches that are already in flight, so expect it to take a "
            "few seconds. Everything finished so far stays on screen.")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_clicked)
        run_row.addWidget(self.run_button)
        run_row.addWidget(self.stop_button)
        budget_layout.addLayout(run_row)

        self.progress_bar = QtWidgets.QProgressBar()
        budget_layout.addWidget(self.progress_bar)

        self.status_label = QtWidgets.QLabel("Ready.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        budget_layout.addWidget(self.status_label)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(subject)
        layout.addWidget(effort)
        layout.addWidget(budget)
        layout.addStretch(1)

    # -- read the form -------------------------------------------------

    def strategy(self) -> str:
        return self.strategy_combo.currentText()

    def partner(self) -> str:
        return self.partner_combo.currentText()

    def opponent(self) -> str:
        return self.opponent_combo.currentText()

    def hall_of_fame_enabled(self) -> bool:
        return self.hall_of_fame_check.isChecked()

    def archive_path(self) -> str:
        # Still guarded even with a real default text above: a user can
        # select-all-and-delete the field by hand.
        return self.archive_edit.text().strip() or "hall_of_fame.json"

    def hof_sample(self) -> int:
        return self.hof_sample_spin.value()

    def generations(self) -> int:
        return self.generations_spin.value()

    def population(self) -> int | None:
        """`None` for "let CMA-ES decide" -- the spin box's 0 is its
        special "auto" value, not a population of zero."""
        return self.population_spin.value() or None

    def seeds(self) -> int:
        return self.seeds_spin.value()

    def confirm_seeds(self) -> int:
        return self.confirm_spin.value()

    def sigma(self) -> float:
        return self.sigma_spin.value()

    def rng_seed(self) -> int:
        return self.rng_spin.value()

    def workers(self) -> int:
        return self.workers_spin.value()

    # -- hall of fame ----------------------------------------------------

    def _update_hof_enabled(self) -> None:
        """Shared by the checkbox's own toggle and `set_running`: the
        archive controls and the (now irrelevant) Opponent combo track
        both "is hall of fame on" and "is a search running" at once, so
        one method owns both rather than two call sites drifting apart."""
        hof = self.hall_of_fame_check.isChecked()
        running = self._running
        self.opponent_combo.setEnabled(not hof and not running)
        self.archive_edit.setEnabled(hof and not running)
        self.archive_browse.setEnabled(hof and not running)
        self.hof_sample_spin.setEnabled(hof and not running)

    def _on_browse_archive(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Hall-of-fame archive", self.archive_edit.text() or "hall_of_fame.json",
            "Archive files (*.json)")
        if path:
            self.archive_edit.setText(path)

    # -- budget / state ------------------------------------------------

    def total_matches(self, param_count: int, hand_written_count: int = 0) -> int:
        population = self.population() or default_population(max(1, param_count))
        field_size = (self.hof_sample() + hand_written_count) if self.hall_of_fame_enabled() else 1
        return matches_required(
            self.generations(), population, self.seeds() * field_size, self.confirm_seeds() * field_size)

    def set_budget(self, matches: int, seconds: float | None, *, estimated: bool = True) -> None:
        if seconds is None:
            time_text = ""
        else:
            qualifier = "~" if estimated else ""
            time_text = f"  ({qualifier}{_humanize(seconds)})"
        self.budget_label.setText(f"{matches:,} matches{time_text}")
        if matches > RED_MATCH_THRESHOLD:
            color = theme.ACCENT_RED
        elif matches > AMBER_MATCH_THRESHOLD:
            color = theme.ACCENT_AMBER
        else:
            color = theme.TEXT_PRIMARY
        self.budget_label.setStyleSheet(f"color: {color};")

        warnings = []
        if self.seeds() < THIN_SEED_THRESHOLD:
            warnings.append(
                f"{self.seeds()} matches per candidate is thin -- at this level the search is "
                "likely to be ranking luck, and any gain it reports will mostly evaporate when "
                "it is confirmed.")
        if self.confirm_seeds() == 0:
            warnings.append(
                "With 0 confirmation matches the result cannot be checked, and the number this "
                "tab reports will be too good to be true.")
        self.warning_label.setText("  ".join(warnings))
        self.warning_label.setVisible(bool(warnings))

    def set_running(self, running: bool) -> None:
        self.run_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        for widget in (self.strategy_combo, self.partner_combo,
                       self.generations_spin, self.population_spin, self.seeds_spin,
                       self.confirm_spin, self.sigma_spin, self.rng_spin, self.workers_spin,
                       self.hall_of_fame_check):
            widget.setEnabled(not running)
        # Opponent / archive controls track hof-on-or-off as well as
        # running-or-not, so one method (_update_hof_enabled) owns both --
        # a plain `not running` here would wrongly re-enable Opponent after
        # a hall-of-fame run finishes.
        self._running = running
        self._update_hof_enabled()

    def set_progress(self, done: int, total: int) -> None:
        self.progress_bar.setMaximum(max(1, total))
        self.progress_bar.setValue(done)

    def set_status(self, text: str) -> None:
        self.status_label.setText(text)


def _caption(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    label.setStyleSheet(f"color: {theme.TEXT_DIM};")
    return label


def _humanize(seconds: float) -> str:
    if seconds < 90:
        return f"{seconds:.0f}s"
    if seconds < 5400:
        return f"{seconds / 60:.0f} min"
    return f"{seconds / 3600:.1f} hours"


# -- what the search may touch -----------------------------------------

class ParameterPanel(QtWidgets.QWidget):
    """The numbers a search is allowed to move, and their limits.

    This exists to make the structure/parameter split visible rather than
    a claim in a docstring. A student can compare this table against the
    strategy file and satisfy themselves that no rule, trigger or tactic
    is on it -- only numbers.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.table = document(
            QtWidgets.QTableWidget(0, 4), "parameters", "Tunable numbers",
            "Every number the search is allowed to change in the chosen strategy, with the "
            "range it may move inside.",
            "Notice what is missing: no rule, trigger or tactic appears here. The search "
            "cannot add a rule, delete one, or reorder them -- it only turns the dials on the "
            "strategy you already wrote. That is why its output is still a strategy file you "
            "can read and edit by hand.")
        self.table.setHorizontalHeaderLabels(["parameter", "now", "lowest", "highest"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        # Parameter paths are long and the three numeric columns are short,
        # so the columns size to their contents and the last one takes the
        # slack. Stretching the path column instead leaves the numbers
        # stranded against the right edge, a screen away from their names.
        self.table.horizontalHeader().setStretchLastSection(True)

        self.summary = QtWidgets.QLabel("")
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet(f"color: {theme.TEXT_DIM};")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.summary)
        layout.addWidget(self.table)

    def set_refs(self, refs, tuned=None) -> None:
        """Show `refs`; if `tuned` is given, the "now" column becomes a
        before -> after so the winning numbers can be read straight off
        the same table the run was configured from."""
        self.table.setRowCount(len(refs))
        for row, ref in enumerate(refs):
            if tuned is None:
                value_text = f"{ref.value:.3g}"
            else:
                value_text = f"{ref.value:.3g}  ->  {tuned[row]:.3g}"
            for column, text in enumerate((ref.path, value_text, f"{ref.lower:g}", f"{ref.upper:g}")):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFont(theme.technical_font(9))
                self.table.setItem(row, column, item)
        self.table.resizeColumnsToContents()
        self.summary.setText(
            f"{len(refs)} numbers can move. The rules, triggers and tactics cannot."
            if refs else
            "This strategy has no tunable numbers -- everything in it is either structural "
            "or switched off. There is nothing for a parameter search to do here.")

    def set_error(self, message: str) -> None:
        self.table.setRowCount(0)
        self.summary.setText(message)


# -- live progress ------------------------------------------------------

class ProgressPanel(QtWidgets.QWidget):
    """Score against generation, plus the per-generation log.

    The plot answers the question a student actually has while waiting --
    "is it still finding anything?" -- which a progress bar cannot. A
    best-so-far curve that has been flat for six generations means the
    run is done learning, whatever the bar says.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(figsize=(5, 3), tight_layout=True)
        self.canvas = document(
            FigureCanvasQTAgg(self.figure), "progress_plot", "Progress plot",
            "The best score found so far (bright) and the average of each round (dim), "
            "against generation number.",
            "A rising best-so-far line means the search is still finding better numbers. When "
            "it goes flat and stays flat, extra generations are buying nothing. The dim "
            "average line climbing toward the bright one is the optimiser narrowing in.")
        self._style_axes()

        self.log = document(
            QtWidgets.QPlainTextEdit(), "log", "Generation log",
            "One line per round: its best score, its average, the best found so far, and how "
            "wide the optimiser is still searching.",
            "'step' is how far the guesses are spread. It shrinks automatically; when it gets "
            "very small the search has converged and will stop on its own.")
        self.log.setReadOnly(True)
        self.log.setFont(theme.technical_font(9))
        self.log.setMaximumHeight(150)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.canvas, stretch=1)
        layout.addWidget(self.log)
        self.clear()

    def _style_axes(self) -> None:
        self.figure.patch.set_facecolor(theme.BG_PANEL)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_facecolor(theme.BG_DEEP)
        for spine in self.axes.spines.values():
            spine.set_color(theme.BORDER)
        self.axes.tick_params(colors=theme.TEXT_DIM)
        self.axes.set_xlabel("generation", color=theme.TEXT_DIM)
        self.axes.set_ylabel("alliance score", color=theme.TEXT_DIM)
        self.axes.grid(True, color=theme.GRID_LINE)

    def clear(self) -> None:
        self._records = []
        self._baseline = None
        self.log.clear()
        self.redraw()

    def set_baseline(self, value: float) -> None:
        self._baseline = value
        self.redraw()

    def append(self, record) -> None:
        self._records.append(record)
        failed = f"  {record.failures} failed" if record.failures else ""
        self.log.appendPlainText(
            f"gen {record.index:>3}   best {record.best:7.1f}   avg {record.mean:7.1f}   "
            f"best so far {record.best_so_far:7.1f}   step {record.sigma:.3f}   "
            f"{record.seconds:5.1f}s{failed}")
        self.redraw()

    def note(self, text: str) -> None:
        self.log.appendPlainText(text)

    def redraw(self) -> None:
        self.axes.clear()
        self._style_axes_after_clear()
        if self._baseline is not None:
            self.axes.axhline(self._baseline, color=theme.TEXT_PRIMARY, linestyle="--",
                              linewidth=1, label="hand-written")
        if self._records:
            x = [r.index for r in self._records]
            self.axes.plot(x, [r.best_so_far for r in self._records],
                           color=theme.ACCENT_CYAN, linewidth=2, label="best so far")
            # TEXT_DIM rather than ACCENT_CYAN_DIM: the latter is a border
            # colour, and a 1px line in it is invisible on BG_DEEP -- which
            # takes the legend swatch with it.
            self.axes.plot(x, [r.mean for r in self._records],
                           color=theme.TEXT_DIM, linewidth=1, label="round average")
        if self._records or self._baseline is not None:
            legend = self.axes.legend(facecolor=theme.BG_RAISED, edgecolor=theme.BORDER, fontsize=8)
            for text in legend.get_texts():
                text.set_color(theme.TEXT_PRIMARY)
        self.canvas.draw_idle()

    def _style_axes_after_clear(self) -> None:
        self.axes.set_facecolor(theme.BG_DEEP)
        for spine in self.axes.spines.values():
            spine.set_color(theme.BORDER)
        self.axes.tick_params(colors=theme.TEXT_DIM)
        self.axes.set_xlabel("generation", color=theme.TEXT_DIM)
        self.axes.set_ylabel("alliance score", color=theme.TEXT_DIM)
        self.axes.grid(True, color=theme.GRID_LINE)


# -- the answer ---------------------------------------------------------

class VerdictPanel(QtWidgets.QWidget):
    """What the run actually found, with the honest number in the large
    type and the flattering one in the small.

    Getting this hierarchy right is the whole reason this panel is not
    just a label. `SearchResult.fitness` is a best-of-N over a shared seed
    set; `Confirmation` is the same two strategies re-scored on seeds
    nothing was selected against. Showing the first as the result is how
    a team ends up building a robot around a gain that was never there.
    """

    save_clicked = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.headline = document(
            QtWidgets.QLabel("No search has been run yet."), "verdict", "The verdict",
            "The result, measured on matches the search never saw. This is the number to "
            "believe.",
            "Green means the tuned strategy really did beat the hand-written one on fresh "
            "matches. Amber means the difference is too small to trust. If you quote one "
            "number from this tab to your team, quote this one.")
        self.headline.setFont(theme.technical_font(16, bold=True))
        self.headline.setWordWrap(True)

        self.detail = QtWidgets.QLabel("")
        self.detail.setWordWrap(True)
        self.detail.setFont(theme.technical_font(10))

        self.optimistic = document(
            QtWidgets.QLabel(""), "optimistic", "The search's own number",
            "What the search thought it had found, before confirmation. Almost always too "
            "good.",
            "The search tries hundreds of candidates and keeps the highest scorer, all judged "
            "on the same matches -- so the winner is partly just the one that got lucky on "
            "those particular matches. The gap between this line and the verdict above is the "
            "size of that luck. On this tool's first real run it was 98% of the reported gain.")
        self.optimistic.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.optimistic.setWordWrap(True)

        self.payoff_text = document(
            QtWidgets.QPlainTextEdit(), "payoff", "Payoff matrix",
            "How the tuned strategy did against each opponent in the hall-of-fame field, on "
            "the held-out confirmation matches, plus exploitability -- how much the single "
            "best counter in that field beat it by.",
            "Zero exploitability means the tuned strategy won every matchup in this field, not "
            "that it is unbeatable -- only that nothing in this particular archive found a "
            "hole. Shown only when a hall-of-fame run finishes.")
        self.payoff_text.setReadOnly(True)
        self.payoff_text.setFont(theme.technical_font(9))
        self.payoff_text.setMaximumHeight(160)
        self.payoff_text.setVisible(False)

        self.save_button = document(
            QtWidgets.QPushButton("Save tuned strategy..."), "save", "Save the result",
            "Writes the tuned strategy to a normal strategy file you can open in the STRATEGY "
            "tab and edit by hand.",
            "The output is an ordinary strategy -- same rules, same order, different numbers "
            "-- so nothing downstream has to know a search produced it.")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_clicked)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.headline)
        layout.addWidget(self.detail)
        layout.addWidget(self.optimistic)
        layout.addWidget(self.payoff_text)
        layout.addWidget(self.save_button, alignment=QtCore.Qt.AlignLeft)
        layout.addStretch(1)

    def clear(self) -> None:
        self.headline.setText("No search has been run yet.")
        self.headline.setStyleSheet(f"color: {theme.TEXT_PRIMARY};")
        self.detail.setText("")
        self.optimistic.setText("")
        self.payoff_text.setVisible(False)
        self.save_button.setEnabled(False)

    def set_payoffs(self, payoffs, archive_note: str = "") -> None:
        """`payoffs` is a sequence of `hall_of_fame.Payoff` (or empty/None
        for a run that did not use the hall of fame) -- kept separate from
        `set_result` so an ordinary fixed-opponent run's verdict is
        untouched by a feature it never used."""
        if not payoffs:
            self.payoff_text.setVisible(False)
            return
        text = describe_payoffs(payoffs)
        if archive_note:
            text += f"\n{archive_note}"
        self.payoff_text.setPlainText(text)
        self.payoff_text.setVisible(True)

    def set_result(self, result, confirmation) -> None:
        self.save_button.setEnabled(True)
        self.optimistic.setText(
            f"During the search it looked like {result.baseline_fitness:.1f} -> "
            f"{result.fitness:.1f} ({result.improvement:+.1f}). That number is measured on the "
            f"same matches the winner was chosen on, so it flatters itself.")

        if confirmation is None:
            self.headline.setText("Unconfirmed -- this result has not been checked.")
            self.headline.setStyleSheet(f"color: {theme.ACCENT_AMBER};")
            self.detail.setText(
                "Confirmation matches were set to 0, so there is no way to tell how much of the "
                "gain above is real. Run it again with confirmation switched on before acting "
                "on this.")
            return

        gain = confirmation.improvement
        self.detail.setText(
            f"On {confirmation.seeds} matches the search never saw: hand-written "
            f"{confirmation.baseline:.1f} -> tuned {confirmation.tuned:.1f}.\n"
            f"Searched {len(result.generations)} generations over {result.matches:,} matches.")

        # One point of alliance score is well inside the seed-to-seed
        # spread of a REEFSCAPE match, so a sub-point confirmed gain is
        # reported as "no difference found" rather than as a small win --
        # a small win is what a student will act on.
        if gain >= 1.0:
            self.headline.setText(f"Real gain: {gain:+.1f} points")
            self.headline.setStyleSheet(f"color: {theme.ACCENT_CYAN};")
        elif gain > -1.0:
            self.headline.setText("No real gain -- the hand-written numbers were already fine")
            self.headline.setStyleSheet(f"color: {theme.ACCENT_AMBER};")
        else:
            self.headline.setText(f"Worse on fresh matches: {gain:+.1f} points")
            self.headline.setStyleSheet(f"color: {theme.ACCENT_RED};")

        if result.improvement > 1.0 and gain < 0.5 * result.improvement:
            lost = 1 - gain / result.improvement
            self.detail.setText(
                self.detail.text() + f"\n\n{lost:.0%} of what the search found did not survive "
                "fresh matches. Raise 'Matches per candidate' before you raise 'Generations'.")


# -- threading ----------------------------------------------------------

class SearchAborted(Exception):
    """Raised out of the progress callback to unwind a running search.

    `search_parameters` has no cancel parameter by design -- it is a
    numerical loop that knows nothing about UIs -- so STOP is implemented
    as an exception thrown from the callback it already calls once per
    generation. The consequence, which the button's tooltip states, is
    that a stop lands at the next generation boundary rather than
    immediately.
    """


class SearchWorker(QtCore.QObject):
    """Lives on a QThread and touches no widgets.

    `job_fn(progress) -> (result, confirmation, hof)` arrives from the tab
    already closed over the game's trial function, so this class -- and
    this package -- never learns what a match is. `hof` is an opaque
    extra payload (payoff matrix, archive note, ...) that only a
    hall-of-fame run populates; a fixed-opponent run's `job_fn` returns
    `None` for it, and this class never looks inside either way.
    """

    generation_ready = QtCore.Signal(object)
    baseline_ready = QtCore.Signal(float)
    note = QtCore.Signal(str)
    finished = QtCore.Signal(bool, str, object, object, object)  # ok, message, result, confirmation, hof

    def __init__(self, job_fn):
        super().__init__()
        self._job_fn = job_fn
        self._abort = False

    def request_abort(self) -> None:
        self._abort = True

    def _progress(self, record) -> None:
        if isinstance(record, str):
            self.note.emit(record)
        else:
            self.generation_ready.emit(record)
        if self._abort:
            raise SearchAborted()

    def run(self) -> None:
        try:
            result, confirmation, hof = self._job_fn(self._progress, self.baseline_ready.emit)
        except SearchAborted:
            self.finished.emit(False, "Stopped. Nothing was saved.", None, None, None)
            return
        except Exception as exc:                      # noqa: BLE001 -- surfaced in the status line
            self.finished.emit(False, f"Search failed: {exc}", None, None, None)
            return
        self.finished.emit(True, "Search complete.", result, confirmation, hof)


class SearchRunController(QtCore.QObject):
    """Owns the QThread lifecycle for one search.

    Unlike `SweepRunController` there is no batching timer: a generation
    is tens of matches and arrives at most every few seconds, so the
    signal rate this forwards is already low enough to hand straight to
    the widgets.
    """

    generation_ready = QtCore.Signal(object)
    baseline_ready = QtCore.Signal(float)
    note = QtCore.Signal(str)
    finished = QtCore.Signal(bool, str, object, object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = None
        self._worker = None

    def is_running(self) -> bool:
        return self._thread is not None

    def start(self, job_fn) -> None:
        self._thread = QtCore.QThread()
        self._worker = SearchWorker(job_fn)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.generation_ready.connect(self.generation_ready)
        self._worker.baseline_ready.connect(self.baseline_ready)
        self._worker.note.connect(self.note)
        self._worker.finished.connect(self._on_finished)
        self._thread.start()

    def _on_finished(self, ok: bool, message: str, result, confirmation, hof) -> None:
        self.finished.emit(ok, message, result, confirmation, hof)
        self.shutdown()

    def abort(self) -> None:
        if self._worker is not None:
            self._worker.request_abort()

    def shutdown(self) -> None:
        """Abort and join -- called from `_on_finished` and from the
        window's `closeEvent`, since a live ProcessPoolExecutor inside
        the worker keeps the process alive after the window closes."""
        if self._worker is not None:
            self._worker.request_abort()
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait()
        self._thread = None
        self._worker = None


def describe_refs(refs) -> str:
    """The parameter table as text, for a log or a saved report."""
    return strategy_params.describe(refs)

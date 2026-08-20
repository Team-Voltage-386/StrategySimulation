"""
SWEEP tab wiring: owns its own roster (independent of MATCH, by design
-- an in-flight sweep must not change because someone edited the MATCH
tab), the variable table, run controls, and the results/plots panes.
This is the one file that knows both about Qt (gui_utils/sweep_panel.py)
and about REEFSCAPE specifics (apps/reefscape_widgets.py,
game_specific/reefscape/sweep_trial.py) -- it is what wires the two
together.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtWidgets

from apps.reefscape_widgets import (
    LABEL_HINTS,
    MatchSettingsPanel,
    RobotRosterConfigPanel,
    STRATEGIES_DIR,
    STRATEGY_NAMES,
    UNIT_HINTS,
    build_demo_characteristics,
)
from common_sim.analysis.results import to_dataframe
from common_sim.analysis.sweep_spec import (
    FieldDescriptor,
    MatchSpec,
    characteristics_to_spec,
    expand_jobs,
    sweepable_fields,
    total_run_count,
)
from game_specific.reefscape import sweep_trial
from gui_utils.doc_tags import document
from gui_utils.sweep_panel import (
    RED_RUN_THRESHOLD,
    SweepControlPanel,
    SweepPlotPanel,
    SweepResultsModel,
    SweepResultsTable,
    SweepRunController,
    VariableTable,
)

# How many of the first completed trials to average for the TOTAL RUNS
# readout's ETA -- plan calls for "the measured mean of the first 10
# completions" replacing the initial 2.0 s/match placeholder.
ETA_SAMPLE_SIZE = 10
INITIAL_ETA_S_PER_TRIAL = 2.0


class SweepTab(QtWidgets.QWidget):
    replay_requested = QtCore.Signal(object)   # TrialJob
    running_changed = QtCore.Signal(bool)

    def __init__(self, strategy_provider=None, parent=None):
        super().__init__(parent)
        self.strategy_provider = strategy_provider
        self._jobs: dict[int, object] = {}
        self._trial_durations: list[float] = []
        self._mean_trial_seconds: float | None = None

        intro = document(
            QtWidgets.QLabel(
                "Run many matches at once, varying one or two things about the robots, and see "
                "how the score responds. Good for questions like \"does a faster drivetrain "
                "actually matter\" that one match can't answer honestly."),
            "intro", "What this tab does",
            "A one-paragraph reminder of what a sweep is for.")
        intro.setWordWrap(True)

        self.roster_config = RobotRosterConfigPanel(sweep_mode=True)
        self.roster_config.roster_changed.connect(self._on_roster_changed)
        self.match_settings_panel = MatchSettingsPanel()

        duration_group = document(
            QtWidgets.QGroupBox("MATCH DURATION"), "duration", "Match duration",
            "How long auto and teleop last in every match this sweep runs.",
            "Independent of MATCH's own settings -- a sweep gets its own copy so a running "
            "sweep can't change because someone edited the MATCH tab mid-run.")
        duration_form = QtWidgets.QFormLayout(duration_group)
        self.auto_duration_spin = QtWidgets.QDoubleSpinBox()
        self.auto_duration_spin.setRange(0.0, 60.0)
        self.auto_duration_spin.setValue(15.0)
        self.auto_duration_spin.setSuffix(" s")
        duration_form.addRow("Auto", self.auto_duration_spin)
        self.teleop_duration_spin = QtWidgets.QDoubleSpinBox()
        self.teleop_duration_spin.setRange(1.0, 600.0)
        self.teleop_duration_spin.setValue(135.0)
        self.teleop_duration_spin.setSuffix(" s")
        duration_form.addRow("Teleop", self.teleop_duration_spin)

        left_content = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_content)
        left_layout.addWidget(intro)
        left_layout.addWidget(self.roster_config)
        left_layout.addWidget(self.match_settings_panel)
        left_layout.addWidget(duration_group)
        left_layout.addStretch(1)

        left_column = QtWidgets.QScrollArea()
        left_column.setWidget(left_content)
        left_column.setWidgetResizable(True)
        left_column.setMinimumWidth(240)
        left_column.setFrameShape(QtWidgets.QFrame.NoFrame)

        self.variable_table = VariableTable(self._targets, self._field_descriptors)
        self.variable_table.changed.connect(self._on_config_changed)

        self.control_panel = SweepControlPanel()
        self.control_panel.execute_clicked.connect(self._on_execute)
        self.control_panel.abort_clicked.connect(self._on_abort)
        self.control_panel.changed.connect(self._on_config_changed)

        top_right = QtWidgets.QWidget()
        top_right_layout = QtWidgets.QVBoxLayout(top_right)
        top_right_layout.addWidget(self.variable_table)
        top_right_layout.addWidget(self.control_panel)

        self.results_model = SweepResultsModel()
        self.results_table = SweepResultsTable(self.results_model)
        self.results_table.replay_requested.connect(self._on_replay_requested)

        self.plot_panel = SweepPlotPanel()
        self.plot_panel.set_data_provider(self._plot_dataframe)

        results_tabs = QtWidgets.QTabWidget()
        results_tabs.addTab(self.results_table, "RESULTS")
        results_tabs.addTab(self.plot_panel, "PLOTS")

        right_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right_splitter.addWidget(top_right)
        right_splitter.addWidget(results_tabs)
        right_splitter.setSizes([260, 500])

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_splitter.addWidget(left_column)
        main_splitter.addWidget(right_splitter)
        main_splitter.setSizes([360, 1000])

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(main_splitter)

        self.run_controller = SweepRunController()
        self.run_controller.batch_ready.connect(self._on_batch_ready)
        self.run_controller.progress.connect(self.control_panel.set_progress)
        self.run_controller.finished.connect(self._on_finished)

        self._on_config_changed()

    # -- sweepable-field discovery -----------------------------------------

    def _targets(self) -> list[str]:
        return self.roster_config.robot_labels()

    def _field_descriptors(self, label: str) -> tuple:
        if not label:
            return ()
        config_tab = self.roster_config.config_tab_for(label)
        overrides = config_tab.characteristics_overrides()
        char_spec = characteristics_to_spec(build_demo_characteristics(**overrides))
        extra = ()
        if STRATEGY_NAMES:
            extra = (FieldDescriptor(
                path="strategy", kind="categorical", default=STRATEGY_NAMES[0], choices=tuple(STRATEGY_NAMES),
            ),)
        return sweepable_fields(char_spec, unit_hints=UNIT_HINTS, label_hints=LABEL_HINTS, extra=extra)

    def _on_roster_changed(self) -> None:
        self.variable_table.refresh_targets()
        self._on_config_changed()

    # -- live run count / validity ------------------------------------------

    def _on_config_changed(self) -> None:
        """Variable table / roster / control-panel edits: refresh the
        live run-count readout AND resync the results table + plot
        columns to the (possibly new) set of swept variables."""
        self._refresh_run_readout()
        columns = [v.column for v in self.variable_table.variables()]
        self.results_model.set_columns(columns)
        self.plot_panel.set_columns(columns)

    def _refresh_run_readout(self) -> None:
        """Just the TOTAL RUNS / EXECUTE-enabled readout -- called on
        progress/batch/finished too, where the results table must NOT be
        reset (that would wipe the rows just appended)."""
        variables = self.variable_table.variables()
        total = total_run_count(variables, self.control_panel.repetitions())

        eta = None
        if total > 0:
            per_trial = self._mean_trial_seconds or INITIAL_ETA_S_PER_TRIAL
            workers = max(1, self.control_panel.worker_count())
            eta = total * per_trial / workers
        self.control_panel.set_total_runs(total, eta)

        valid = self.variable_table.is_valid() and total > 0
        self.control_panel.set_execute_enabled(valid)

    # -- job construction ---------------------------------------------------

    def _build_jobs(self) -> list:
        robots = self.roster_config.robot_specs(strategy_override=self.strategy_provider)
        match = MatchSpec(
            auto_duration=self.auto_duration_spin.value(), teleop_duration=self.teleop_duration_spin.value(),
            disable_friendly_collisions=self.match_settings_panel.disable_friendly_collisions(),
        )
        variables = self.variable_table.variables()
        variability = self.control_panel.variability_model()
        jobs = expand_jobs(
            robots, match, variability, variables,
            repetitions=self.control_panel.repetitions(), base_seed=self.control_panel.base_seed(),
            strategies_dir=STRATEGIES_DIR, dt=sweep_trial.SWEEP_DT,
        )
        self._jobs = {job.index: job for job in jobs}
        return jobs

    # -- run lifecycle --------------------------------------------------

    def _on_execute(self) -> None:
        if self.run_controller.is_running():
            return
        total = total_run_count(self.variable_table.variables(), self.control_panel.repetitions())
        if total > RED_RUN_THRESHOLD:
            reply = QtWidgets.QMessageBox.question(
                self, "Large sweep", f"This sweep will run {total} matches. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            )
            if reply != QtWidgets.QMessageBox.Yes:
                return

        jobs = self._build_jobs()
        columns = [v.column for v in self.variable_table.variables()]
        self.results_model.set_columns(columns)
        self.plot_panel.set_columns(columns)
        self._mean_trial_seconds = None
        self._trial_durations = []

        self.control_panel.set_running(True)
        self.control_panel.set_status(f"Running {len(jobs)} trials...")
        self.running_changed.emit(True)
        self.run_controller.start(sweep_trial.run_trial, jobs, parallel=True, max_workers=self.control_panel.worker_count())

    def _on_abort(self) -> None:
        self.run_controller.abort()

    def _on_batch_ready(self, outcomes: list) -> None:
        self.results_model.append_batch(outcomes)
        if self._mean_trial_seconds is None:
            for outcome in outcomes:
                if outcome.error is None:
                    self._trial_durations.append(outcome.duration_s)
                if len(self._trial_durations) >= ETA_SAMPLE_SIZE:
                    self._mean_trial_seconds = sum(self._trial_durations) / len(self._trial_durations)
                    break
        self.plot_panel.request_redraw()
        self._refresh_run_readout()

    def _on_finished(self, success: bool, message: str) -> None:
        self.control_panel.set_running(False)
        self.control_panel.set_status(message)
        self.running_changed.emit(False)
        self.plot_panel.request_redraw()
        self._refresh_run_readout()

    def _plot_dataframe(self):
        outcomes = self.results_model.successful_outcomes()
        return to_dataframe(outcomes) if outcomes else None

    def _on_replay_requested(self, job_index: int) -> None:
        job = self._jobs.get(job_index)
        if job is not None:
            self.replay_requested.emit(job)

    def shutdown(self) -> None:
        self.run_controller.shutdown()

"""The SEARCH tab's widgets, without running a search.

Two things are worth pinning here and the rest is Qt's problem:

* the budget the panel quotes has to be the number of matches that will
  actually run, or a student sizes an evening against a fiction;
* the verdict panel has to report the *confirmed* number, and has to say
  so loudly when a search's own figure did not survive. That is the one
  behaviour in this tab that exists because of a measured mistake, so it
  is the one that gets tests rather than trust.
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets                                      # noqa: E402

from common_sim.analysis.cmaes import CMAES                             # noqa: E402
from common_sim.analysis.param_search import (                          # noqa: E402
    Confirmation, Generation, SearchResult, default_population, matches_required,
)
from gui_utils import theme                                             # noqa: E402
from gui_utils.search_panel import (                                    # noqa: E402
    ParameterPanel, ProgressPanel, SearchSetupPanel, SearchWorker, VerdictPanel,
)

STRATEGIES = ["cycle_coral", "full_defense", "auto_then_cycle"]


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def panel(app):
    return SearchSetupPanel(STRATEGIES)


def _result(*, baseline=200.0, best=210.0, refs=(), vector=()):
    return SearchResult(
        payload={"rules": []}, fitness=best, baseline_payload={"rules": []},
        baseline_fitness=baseline, refs=refs, vector=vector,
        generations=(Generation(index=1, best=best, mean=best, best_so_far=best,
                                sigma=0.2, failures=0, seconds=1.0),),
        matches=100,
    )


# -- budget -------------------------------------------------------------

def test_the_quoted_budget_is_the_number_of_matches_that_will_run(panel):
    panel.generations_spin.setValue(10)
    panel.population_spin.setValue(8)
    panel.seeds_spin.setValue(16)
    panel.confirm_spin.setValue(32)
    assert panel.total_matches(param_count=5) == matches_required(10, 8, 16, 32)


def test_auto_population_prices_the_population_cma_es_will_actually_use(panel):
    """The spin box's 0 means "let CMA-ES decide", and the estimate has to
    decide the same way or the readout is wrong by a factor."""
    panel.population_spin.setValue(0)
    assert panel.population() is None

    panel.generations_spin.setValue(4)
    panel.seeds_spin.setValue(8)
    panel.confirm_spin.setValue(0)
    expected = matches_required(4, default_population(6), 8, 0)
    assert panel.total_matches(param_count=6) == expected


def test_the_auto_population_agrees_with_the_optimizer_itself():
    """`default_population` is a duplicate of a formula inside CMAES so a
    run can be sized before the optimizer exists. This is the test that
    keeps the duplicate honest."""
    for n in (1, 2, 5, 12, 40):
        assert default_population(n) == CMAES([0.0] * n, 0.2).population_size


def test_a_thin_seed_count_is_called_out_before_the_run_not_after(panel):
    panel.seeds_spin.setValue(4)
    panel.set_budget(100, None)
    assert panel.warning_label.isVisibleTo(panel)
    assert "ranking luck" in panel.warning_label.text()

    panel.seeds_spin.setValue(32)
    panel.set_budget(100, None)
    assert not panel.warning_label.isVisibleTo(panel)


def test_switching_confirmation_off_is_warned_about(panel):
    panel.seeds_spin.setValue(32)
    panel.confirm_spin.setValue(0)
    panel.set_budget(100, None)
    assert "too good to be true" in panel.warning_label.text()


def test_an_expensive_run_is_coloured_before_it_is_started(panel):
    panel.set_budget(50, None)
    assert theme.ACCENT_RED not in panel.budget_label.styleSheet()
    panel.set_budget(500_000, None)
    assert theme.ACCENT_RED in panel.budget_label.styleSheet()


def test_running_locks_the_form_so_a_search_cannot_be_reconfigured_mid_flight(panel):
    panel.set_running(True)
    assert not panel.run_button.isEnabled()
    assert panel.stop_button.isEnabled()
    assert not panel.seeds_spin.isEnabled()

    panel.set_running(False)
    assert panel.run_button.isEnabled()
    assert not panel.stop_button.isEnabled()
    assert panel.seeds_spin.isEnabled()


# -- the verdict --------------------------------------------------------

def test_a_confirmed_gain_is_reported_as_the_headline(app):
    verdict = VerdictPanel()
    verdict.set_result(_result(baseline=200.0, best=214.0),
                       Confirmation(baseline=200.0, tuned=207.0, seeds=32))
    assert "+7.0" in verdict.headline.text()
    assert theme.ACCENT_CYAN in verdict.headline.styleSheet()


def test_a_gain_that_did_not_survive_is_reported_as_no_gain(app):
    """The measured case: the search's own number said +9.5 and the
    held-out re-run said +0.2. The headline must follow the second."""
    verdict = VerdictPanel()
    verdict.set_result(_result(baseline=212.0, best=221.5),
                       Confirmation(baseline=209.54, tuned=209.76, seeds=12))
    assert "No real gain" in verdict.headline.text()
    assert theme.ACCENT_AMBER in verdict.headline.styleSheet()
    assert "did not survive" in verdict.detail.text()
    assert "Matches per candidate" in verdict.detail.text()


def test_the_searchs_own_number_is_shown_but_never_as_the_headline(app):
    verdict = VerdictPanel()
    verdict.set_result(_result(baseline=212.0, best=221.5),
                       Confirmation(baseline=209.54, tuned=209.76, seeds=12))
    assert "221.5" in verdict.optimistic.text()
    assert "221.5" not in verdict.headline.text()
    assert "flatters itself" in verdict.optimistic.text()


def test_a_confirmed_loss_is_reported_as_a_loss(app):
    verdict = VerdictPanel()
    verdict.set_result(_result(baseline=200.0, best=210.0),
                       Confirmation(baseline=200.0, tuned=193.0, seeds=32))
    assert "-7.0" in verdict.headline.text()
    assert theme.ACCENT_RED in verdict.headline.styleSheet()


def test_an_unconfirmed_run_refuses_to_report_a_number_at_all(app):
    """With confirmation switched off there is no honest figure to give,
    so the panel gives none rather than falling back to the optimistic
    one."""
    verdict = VerdictPanel()
    verdict.set_result(_result(baseline=200.0, best=230.0), None)
    assert "Unconfirmed" in verdict.headline.text()
    assert "230" not in verdict.headline.text()
    assert verdict.save_button.isEnabled()


def test_clearing_puts_the_panel_back_to_its_opening_state(app):
    verdict = VerdictPanel()
    verdict.set_result(_result(), Confirmation(baseline=1.0, tuned=2.0, seeds=4))
    verdict.clear()
    assert not verdict.save_button.isEnabled()
    assert verdict.optimistic.text() == ""


# -- the parameter table ------------------------------------------------

class _Ref:
    def __init__(self, path, value, lower, upper):
        self.path, self.value, self.lower, self.upper = path, value, lower, upper


def test_the_parameter_table_lists_every_tunable_number(app):
    panel = ParameterPanel()
    panel.set_refs([_Ref("rules[0].cooldown", 1.5, 0.0, 20.0),
                    _Ref("rules[1].tactic.cluster_radius", 24.0, 0.0, 360.0)])
    assert panel.table.rowCount() == 2
    assert panel.table.item(0, 0).text() == "rules[0].cooldown"
    assert "2 numbers can move" in panel.summary.text()


def test_a_tuned_vector_is_shown_against_what_it_started_from(app):
    panel = ParameterPanel()
    panel.set_refs([_Ref("rules[0].cooldown", 1.5, 0.0, 20.0)], tuned=[3.25])
    assert panel.table.item(0, 1).text() == "1.5  ->  3.25"


def test_a_strategy_with_nothing_to_tune_says_so_rather_than_showing_an_empty_table(app):
    panel = ParameterPanel()
    panel.set_refs([])
    assert panel.table.rowCount() == 0
    assert "nothing for a parameter search to do" in panel.summary.text()


# -- progress -----------------------------------------------------------

def test_each_generation_adds_one_log_line(app):
    progress = ProgressPanel()
    progress.set_baseline(200.0)
    for i in (1, 2, 3):
        progress.append(Generation(index=i, best=200.0 + i, mean=199.0, best_so_far=200.0 + i,
                                   sigma=0.2, failures=0, seconds=3.0))
    lines = progress.log.toPlainText().splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("gen   1")


# -- the worker ---------------------------------------------------------
#
# `run()` is called directly rather than through a QThread: it is
# deliberately pure (it touches no widgets), so a thread would add
# scheduling flakiness without testing anything extra.

def _record(index=1):
    return Generation(index=index, best=1.0, mean=1.0, best_so_far=1.0,
                      sigma=0.1, failures=0, seconds=1.0)


def test_a_completed_search_hands_back_both_numbers(app):
    result, confirmation = _result(), Confirmation(baseline=1.0, tuned=2.0, seeds=4)
    worker = SearchWorker(lambda progress, baseline: (result, confirmation, None))
    seen = []
    worker.finished.connect(lambda *args: seen.append(args))

    worker.run()
    assert seen == [(True, "Search complete.", result, confirmation, None)]


def test_stop_unwinds_the_search_at_the_next_generation_boundary(app):
    """STOP cannot interrupt a generation -- `search_parameters` is a
    numerical loop with no cancel hook -- so it is an exception thrown
    from the progress callback. This pins that the job actually stops
    rather than running to completion and being discarded."""
    generations_run = []

    def job(progress, baseline):
        for index in range(1, 6):
            generations_run.append(index)
            progress(_record(index))
        return _result(), None, None

    worker = SearchWorker(job)
    seen = []
    worker.finished.connect(lambda *args: seen.append(args))
    worker.generation_ready.connect(lambda _r: worker.request_abort())

    worker.run()
    assert generations_run == [1]                      # not [1, 2, 3, 4, 5]
    assert seen[0][0] is False
    assert "Stopped" in seen[0][1]
    assert seen[0][2] is None                          # and nothing to save


def test_a_crash_inside_the_search_becomes_a_status_line_not_a_dead_thread(app):
    def job(progress, baseline):
        raise ValueError("no searchable continuous parameters")

    worker = SearchWorker(job)
    seen = []
    worker.finished.connect(lambda *args: seen.append(args))

    worker.run()
    assert seen[0][0] is False
    assert "no searchable continuous parameters" in seen[0][1]


def test_a_string_from_the_progress_callback_is_a_note_not_a_generation(app):
    """`search_parameters` reports early convergence as a plain string
    through the same callback, so the two have to be told apart."""
    notes, generations = [], []
    worker = SearchWorker(lambda progress, baseline: (
        progress(_record()), progress("stopping after generation 1: converged"),
        (_result(), None, None))[-1])
    worker.note.connect(notes.append)
    worker.generation_ready.connect(generations.append)

    worker.run()
    assert notes == ["stopping after generation 1: converged"]
    assert [g.index for g in generations] == [1]


def test_clearing_progress_drops_the_previous_runs_curve(app):
    progress = ProgressPanel()
    progress.append(Generation(index=1, best=1.0, mean=1.0, best_so_far=1.0,
                               sigma=0.1, failures=0, seconds=1.0))
    progress.clear()
    assert progress.log.toPlainText() == ""
    assert progress._records == []

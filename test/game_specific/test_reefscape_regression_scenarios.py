"""Release-scale checks for the small fixed-seed REEFSCAPE scenario library."""
import pytest

from apps.run_regression_scenarios import _problems
from game_specific.reefscape.regression_scenarios import SCENARIOS, scenario_named
from game_specific.reefscape.sweep_trial import run_trial


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda scenario: scenario.key)
def test_regression_scenario_matches_its_golden_outcomes(scenario):
    outcome = run_trial(scenario.job())
    assert outcome.error is None
    assert _problems(scenario, outcome.metrics) == []


def test_scenario_names_are_stable_and_unknown_name_explains_choices():
    assert scenario_named("single_coral_cycle") is SCENARIOS[0]
    with pytest.raises(ValueError, match="cycler_vs_defense"):
        scenario_named("coral-cycle")

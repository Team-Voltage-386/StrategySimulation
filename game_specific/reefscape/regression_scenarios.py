"""Small, fixed-seed REEFSCAPE matches used as release regressions.

Unit tests are deliberately precise about one rule or one subsystem.  The
scenarios here answer the complementary release question: does a real field,
real strategy JSON, the match loop, scoring, and metrics still work together?

They are intentionally short (35 simulated seconds) so they belong in the
normal test suite.  Scores are golden values for the pinned seed and timestep,
not claims about which strategy is best.  If a value changes, inspect the
replay and explain the behavioural change before deliberately updating it.
"""
from __future__ import annotations

from dataclasses import dataclass

from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, TrialJob, characteristics_to_spec
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.robot import build_characteristics
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT


@dataclass(frozen=True)
class RegressionScenario:
    """One reproducible, headless match and the outcomes worth protecting."""

    key: str
    purpose: str
    seed: int
    robots: tuple[tuple[str, str, str], ...]  # label, alliance, strategy filename stem
    expected_scores: dict[str, float]
    expected_pieces_scored: int
    expected_misses: int

    def job(self) -> TrialJob:
        characteristics = characteristics_to_spec(build_characteristics())
        robots = tuple(
            RobotSpec(
                label=label,
                alliance=alliance,
                roster_index=-1,
                characteristics=characteristics,
                strategy=strategy,
            )
            for label, alliance, strategy in self.robots
        )
        return TrialJob(
            index=0,
            seed=self.seed,
            params={"scenario": self.key},
            robots=robots,
            match=MatchSpec(auto_duration=5.0, teleop_duration=30.0),
            variability=VariabilityModel(),
            strategies_dir=str(STRATEGIES_DIR),
            dt=SWEEP_DT,
        )


SCENARIOS = (
    RegressionScenario(
        key="single_coral_cycle",
        purpose="A single AI completes repeated CORAL cycles on the real field.",
        seed=17,
        robots=(("PRIMARY", "blue", "cycle_coral"),),
        expected_scores={"blue": 40.0, "red": 8.0},
        expected_pieces_scored=10,
        expected_misses=1,
    ),
    RegressionScenario(
        key="cycler_vs_defense",
        purpose="A cycling robot and an opposing defender share the field safely.",
        seed=23,
        robots=(
            ("BLUE_CYCLER", "blue", "cycle_coral"),
            ("RED_DEFENDER", "red", "full_defense"),
        ),
        expected_scores={"blue": 19.0, "red": 4.0},
        expected_pieces_scored=4,
        expected_misses=0,
    ),
)


def scenario_named(key: str) -> RegressionScenario:
    """Return a scenario by stable CLI/test name, with useful typo feedback."""
    for scenario in SCENARIOS:
        if scenario.key == key:
            return scenario
    choices = ", ".join(scenario.key for scenario in SCENARIOS)
    raise ValueError(f"unknown regression scenario {key!r}; choose one of: {choices}")

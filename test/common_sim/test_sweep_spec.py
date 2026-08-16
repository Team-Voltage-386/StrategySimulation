from __future__ import annotations

import pytest

from common_sim.analysis.sweep_spec import (
    FieldDescriptor,
    MatchSpec,
    NumericSampling,
    RobotSpec,
    SweepVariable,
    apply_variable,
    characteristics_from_spec,
    characteristics_to_spec,
    expand_jobs,
    sweepable_fields,
    total_run_count,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators


def _full_characteristics() -> RobotCharacteristics:
    return RobotCharacteristics(
        name="full-bot",
        max_speed=180.0, max_accel=300.0, max_angular_speed=7.0, max_angular_accel=25.0,
        width=30.0, length=30.0, mass=20.0,
        piece_capacity=2, piece_capacity_by_type={"coral": 1, "algae": 1}, starting_piece_count=1,
        intake_time=0.4, intake_time_by_type={"coral": 0.4, "algae": 0.5}, station_intake_time=0.6,
        deposit_time=0.5, deposit_time_by_action={"l1": 0.3, "l4": 1.8, "net": 1.2}, intake_range=6.0,
        accepted_piece_types=frozenset({"coral", "algae"}),
        side_manipulators={
            "front": SideManipulators(intake_piece_types=frozenset({"coral"}), score_piece_types=frozenset({"coral"})),
            "right": SideManipulators(intake_piece_types=frozenset({"algae"}), intake_source="station"),
        },
    )


def test_characteristics_spec_round_trips_fully_populated():
    original = _full_characteristics()
    spec = characteristics_to_spec(original)
    restored = characteristics_from_spec(spec)
    assert restored == original


def test_characteristics_spec_encodes_frozensets_as_sorted_lists():
    spec = characteristics_to_spec(_full_characteristics())
    assert spec["accepted_piece_types"] == ["algae", "coral"]
    assert spec["side_manipulators"]["front"]["intake_piece_types"] == ["coral"]


def test_sweepable_fields_includes_and_excludes_expected():
    spec = characteristics_to_spec(_full_characteristics())
    fields = sweepable_fields(spec)
    paths = {f.path for f in fields}
    assert "max_speed" in paths
    assert "piece_capacity" in paths
    assert "name" not in paths
    assert "deposit_time_by_action.l4" in paths
    assert "deposit_time_by_action.l1" in paths
    assert "deposit_time_by_action.net" in paths
    # only keys actually present in this robot's spec should show up
    assert "deposit_time_by_action.l2" not in paths


def test_sweepable_fields_extra_appended():
    spec = characteristics_to_spec(_full_characteristics())
    extra = (FieldDescriptor(path="strategy", kind="categorical", default="cycle_coral", choices=("cycle_coral", "algae_processor")),)
    fields = sweepable_fields(spec, extra=extra)
    assert fields[-1].path == "strategy"
    assert fields[-1].kind == "categorical"


def test_numeric_sampling_values():
    assert NumericSampling(2, 4, 3).values() == (2.0, 3.0, 4.0)
    assert NumericSampling(5, 5, 1).values() == (5,)


def _robot_spec(label="PRIMARY", **char_overrides):
    spec = characteristics_to_spec(_full_characteristics())
    spec.update(char_overrides)
    return RobotSpec(label=label, alliance="blue", roster_index=-1, characteristics=spec, strategy="cycle_coral")


def test_apply_variable_top_level():
    robot = _robot_spec()
    updated = apply_variable(robot, "max_speed", 200.0)
    assert updated.characteristics["max_speed"] == 200.0
    assert robot.characteristics["max_speed"] != 200.0  # original untouched


def test_apply_variable_dotted():
    robot = _robot_spec()
    updated = apply_variable(robot, "deposit_time_by_action.l4", 2.5)
    assert updated.characteristics["deposit_time_by_action"]["l4"] == 2.5
    assert updated.characteristics["deposit_time_by_action"]["l1"] == 0.3  # sibling key untouched


def test_apply_variable_strategy():
    robot = _robot_spec()
    updated = apply_variable(robot, "strategy", "algae_processor")
    assert updated.strategy == "algae_processor"


def test_apply_variable_raises_on_unknown_path():
    robot = _robot_spec()
    with pytest.raises(ValueError):
        apply_variable(robot, "not_a_real_field", 1.0)
    with pytest.raises(ValueError):
        apply_variable(robot, "not_a_real_dict.key", 1.0)


def test_expand_jobs_length_and_seeds():
    robots = [_robot_spec("PRIMARY")]
    match = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
    variables = [
        SweepVariable(target="PRIMARY", path="max_speed", values=(150.0, 160.0)),
        SweepVariable(target="PRIMARY", path="max_accel", values=(300.0, 320.0, 340.0)),
    ]
    jobs = expand_jobs(
        robots, match, VariabilityModel(), variables,
        repetitions=2, base_seed=100, strategies_dir="strategies", dt=1 / 60,
    )
    assert len(jobs) == 2 * 3 * 2
    seeds = [job.seed for job in jobs]
    assert seeds == [100 + i for i in range(len(jobs))]
    assert len(set(seeds)) == len(jobs)


def test_expand_jobs_applies_variable_to_correct_robot():
    robots = [_robot_spec("PRIMARY"), _robot_spec("BLUE 0")]
    match = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
    variables = [SweepVariable(target="BLUE 0", path="max_speed", values=(999.0,))]
    jobs = expand_jobs(robots, match, VariabilityModel(), variables, strategies_dir="strategies", dt=1 / 60)
    assert len(jobs) == 1
    job = jobs[0]
    by_label = {r.label: r for r in job.robots}
    assert by_label["BLUE 0"].characteristics["max_speed"] == 999.0
    assert by_label["PRIMARY"].characteristics["max_speed"] != 999.0


def test_total_run_count_agrees_with_expand_jobs():
    robots = [_robot_spec("PRIMARY")]
    match = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
    variables = [
        SweepVariable(target="PRIMARY", path="max_speed", values=(150.0, 160.0, 170.0)),
    ]
    jobs = expand_jobs(robots, match, VariabilityModel(), variables, repetitions=3, strategies_dir="strategies", dt=1 / 60)
    assert total_run_count(variables, repetitions=3) == len(jobs)


def test_total_run_count_zero_variables():
    assert total_run_count([], repetitions=1) == 1
    assert total_run_count([], repetitions=5) == 5

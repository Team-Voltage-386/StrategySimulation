from __future__ import annotations

import subprocess
import sys

from common_sim.analysis.variability import (
    VariabilityModel,
    perturb_characteristics,
    perturb_pose,
    scatter_offset,
    substream,
)


def _spec():
    return {
        "intake_time": 0.4,
        "intake_time_by_type": {"coral": 0.4, "algae": 0.5},
        "station_intake_time": 0.6,
        "deposit_time": 0.5,
        "deposit_time_by_action": {"l1": 0.3, "l4": 1.8},
        "max_speed": 150.0,
        "max_accel": 300.0,
    }


def test_same_seed_gives_identical_perturbation():
    model = VariabilityModel(enabled=True, intake_time_pct=0.2, deposit_time_pct=0.2, max_speed_pct=0.1, max_accel_pct=0.1)
    a = perturb_characteristics(_spec(), model, substream(42, "chars:PRIMARY"))
    b = perturb_characteristics(_spec(), model, substream(42, "chars:PRIMARY"))
    assert a == b


def test_disabled_returns_input_unchanged_and_draws_nothing():
    model = VariabilityModel(enabled=False, intake_time_pct=0.5)
    spec = _spec()
    rng = substream(1, "chars:PRIMARY")
    result = perturb_characteristics(spec, model, rng)
    assert result == spec
    assert result is spec

    x, y, heading = perturb_pose(1.0, 2.0, 0.5, model, rng)
    assert (x, y, heading) == (1.0, 2.0, 0.5)

    assert scatter_offset(model, rng) == (0.0, 0.0)


def test_draw_order_independent_of_dict_insertion_order():
    model = VariabilityModel(enabled=True, deposit_time_pct=0.3)
    spec_a = dict(_spec())
    spec_a["deposit_time_by_action"] = {"l1": 0.3, "l4": 1.8}
    spec_b = dict(_spec())
    spec_b["deposit_time_by_action"] = {"l4": 1.8, "l1": 0.3}  # reversed insertion order

    result_a = perturb_characteristics(spec_a, model, substream(7, "chars:X"))
    result_b = perturb_characteristics(spec_b, model, substream(7, "chars:X"))
    assert result_a["deposit_time_by_action"] == result_b["deposit_time_by_action"]


def test_multiplier_draws_stay_clamped_and_positive():
    model = VariabilityModel(enabled=True, intake_time_pct=3.0)  # deliberately huge sigma
    rng = substream(0, "stress")
    spec = _spec()
    for _ in range(10_000):
        result = perturb_characteristics(spec, model, rng)
        assert result["intake_time"] > 0.0
        assert 0.25 * spec["intake_time"] <= result["intake_time"] <= 4.0 * spec["intake_time"]


def test_substream_stable_across_fresh_interpreter():
    script = (
        "from common_sim.analysis.variability import substream; "
        "print(substream(123, 'pose:BLUE 0').random())"
    )
    first = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, check=True).stdout.strip()
    second = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, check=True).stdout.strip()
    assert first == second

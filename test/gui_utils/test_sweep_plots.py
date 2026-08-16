from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # must happen before any other matplotlib import, and before importing sweep_plots

import numpy as np
import pandas as pd
import pytest
from matplotlib.collections import QuadMesh
from matplotlib.figure import Figure

from gui_utils.sweep_plots import render_sweep


def _df(rows):
    return pd.DataFrame(rows)


def test_zero_columns_renders_summary_text():
    df = _df([{"total_score": 10.0}, {"total_score": 12.0}, {"total_score": 11.0}])
    fig = Figure()
    render_sweep(fig, df, [], metric="total_score")
    assert len(fig.axes) == 1
    assert fig.axes[0].texts  # summary text was drawn


def test_one_column_renders_line_axes():
    df = _df([
        {"max_speed": 150.0, "total_score": 10.0}, {"max_speed": 150.0, "total_score": 12.0},
        {"max_speed": 170.0, "total_score": 20.0}, {"max_speed": 170.0, "total_score": 22.0},
    ])
    fig = Figure()
    render_sweep(fig, df, ["max_speed"], metric="total_score")
    assert len(fig.axes) == 1
    ax = fig.axes[0]
    assert ax.lines or ax.containers  # errorbar (lines) or bar (containers)


def test_two_columns_renders_quadmesh():
    rows = []
    for speed in (150.0, 170.0):
        for accel in (300.0, 320.0):
            rows.append({"max_speed": speed, "max_accel": accel, "total_score": speed + accel})
    df = _df(rows)
    fig = Figure()
    render_sweep(fig, df, ["max_speed", "max_accel"], metric="total_score")
    meshes = [c for c in fig.axes[0].collections if isinstance(c, QuadMesh)]
    assert len(meshes) == 1


def test_three_columns_renders_faceted_heatmaps_with_shared_colorbar():
    rows = []
    for strategy in ("cycle_coral", "algae_processor"):
        for speed in (150.0, 170.0):
            for accel in (300.0, 320.0):
                rows.append({"strategy": strategy, "max_speed": speed, "max_accel": accel, "total_score": speed + accel})
    df = _df(rows)
    fig = Figure()
    render_sweep(fig, df, ["strategy", "max_speed", "max_accel"], metric="total_score", facet_column="strategy")
    n_facets = df["strategy"].nunique()
    assert len(fig.axes) == n_facets + 1  # + 1 shared colorbar axes


def test_nan_cell_renders_without_raising():
    # An incomplete grid -- one (speed, accel) combo missing -- produces a
    # NaN cell in the pivoted heatmap; pcolormesh must tolerate that.
    rows = [
        {"max_speed": 150.0, "max_accel": 300.0, "total_score": 10.0},
        {"max_speed": 170.0, "max_accel": 300.0, "total_score": 20.0},
        {"max_speed": 170.0, "max_accel": 320.0, "total_score": 22.0},
    ]
    df = _df(rows)
    fig = Figure()
    render_sweep(fig, df, ["max_speed", "max_accel"], metric="total_score")
    meshes = [c for c in fig.axes[0].collections if isinstance(c, QuadMesh)]
    assert len(meshes) == 1
    array = meshes[0].get_array()
    assert np.ma.is_masked(array) and array.mask.any()


def test_more_than_three_columns_raises():
    df = _df([{"a": 1, "b": 1, "c": 1, "d": 1, "total_score": 5.0}])
    fig = Figure()
    with pytest.raises(ValueError):
        render_sweep(fig, df, ["a", "b", "c", "d"], metric="total_score")

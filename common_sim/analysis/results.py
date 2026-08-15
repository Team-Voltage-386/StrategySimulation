"""
Aggregate a Monte Carlo run's TrialResults into a pandas DataFrame and
summarize by swept parameter -- the basic "which design wins" report.
"""
from __future__ import annotations

import pandas as pd

from common_sim.analysis.monte_carlo import TrialResult


def to_dataframe(results: list[TrialResult]) -> pd.DataFrame:
    rows = []
    for result in results:
        row = dict(result.params)
        m = result.metrics
        row["total_score"] = sum(m.final_scores.values())
        for alliance, score in m.final_scores.items():
            row[f"score_{alliance}"] = score
        row["pieces_scored"] = m.pieces_scored
        row["pieces_intaked"] = m.pieces_intaked
        row["pieces_deposited"] = m.pieces_deposited
        row["misses"] = m.misses
        row["mean_cycle_time"] = m.mean_cycle_time
        rows.append(row)
    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame, param_names: list[str], metric: str = "total_score") -> pd.DataFrame:
    """Group by the swept parameters and report mean/std/count of
    `metric` across repetitions -- e.g. "does a bigger piece_capacity
    actually raise mean total_score, or just add variance."""
    if not param_names:
        summary = df[metric].agg(["mean", "std", "count"]).to_frame().T
        return summary.reset_index(drop=True)
    return df.groupby(list(param_names), as_index=False)[metric].agg(["mean", "std", "count"])

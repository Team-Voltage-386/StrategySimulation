"""
matplotlib rendering for the SWEEP tab's PLOTS pane. Every function
takes a `Figure` (never calls `pyplot`, never touches a widget), so this
whole module is testable headless under the Agg backend with no Qt --
see test/gui_utils/test_sweep_plots.py.

Aggregation across repetitions is always
common_sim/analysis/results.summarize(df, columns, metric): mean is the
plotted value, std the error bars, count shown in the axis title
("mean of N=3 runs per point") so a non-engineer knows a cell is an
average, not one match.

Trap: a caller embedding this in Qt (gui_utils/sweep_panel.py) must
import pyqtgraph.Qt *before* calling matplotlib.use("Qt5Agg"), so
matplotlib binds to the already-loaded PyQt5 -- the wrong order gives an
obscure "Cannot mix incompatible Qt library" crash. This module itself
never calls matplotlib.use()/pyplot, so it has no opinion on backend.
"""
from __future__ import annotations

from common_sim.analysis.results import summarize
from gui_utils import theme

MAX_SWEEP_COLUMNS = 3


def apply_dark_style(fig) -> None:
    fig.patch.set_facecolor(theme.BG_DEEP)
    for ax in fig.axes:
        _style_axes(ax)


def _style_axes(ax) -> None:
    ax.set_facecolor(theme.BG_PANEL)
    ax.tick_params(colors=theme.TEXT_DIM)
    ax.xaxis.label.set_color(theme.TEXT_PRIMARY)
    ax.yaxis.label.set_color(theme.TEXT_PRIMARY)
    ax.title.set_color(theme.ACCENT_CYAN)
    for spine in ax.spines.values():
        spine.set_color(theme.BORDER)


def render_sweep(fig, df, columns, metric: str = "total_score", *, facet_column=None):
    """Dispatch on how many columns are swept: 0 -> summary text; 1 ->
    line with error bars; 2 -> heatmap; 3 -> small-multiple heatmaps
    faceted on the variable with the fewest distinct values (or
    `facet_column`, if given); >3 raises -- the panel is expected to
    narrow the selection before calling this."""
    fig.clear()
    columns = list(columns)
    if len(columns) > MAX_SWEEP_COLUMNS:
        raise ValueError(f"render_sweep supports at most {MAX_SWEEP_COLUMNS} swept columns, got {len(columns)}")

    if not columns:
        _render_summary_text(fig, df, metric)
    elif len(columns) == 1:
        summary = summarize(df, columns, metric)
        render_line(fig, summary, columns[0], metric)
    elif len(columns) == 2:
        summary = summarize(df, columns, metric)
        ax = fig.add_subplot(111)
        render_heatmap(fig, summary, columns[0], columns[1], metric, ax=ax)
    else:
        facet = facet_column or min(columns, key=lambda c: df[c].nunique())
        x, y = [c for c in columns if c != facet]
        summary = summarize(df, columns, metric)
        render_faceted_heatmaps(fig, summary, x, y, facet, metric)

    apply_dark_style(fig)


def _render_summary_text(fig, df, metric: str) -> None:
    ax = fig.add_subplot(111)
    agg = summarize(df, [], metric)
    mean, std, count = agg.loc[0, "mean"], agg.loc[0, "std"], int(agg.loc[0, "count"])
    std_text = "--" if std != std else f"{std:.2f}"  # std is NaN for a single run
    ax.axis("off")
    ax.text(0.5, 0.5, f"{metric}\nmean {mean:.2f} ± {std_text}  (N={count})", ha="center", va="center", fontsize=14)


def render_line(fig, summary, x: str, metric: str):
    """errorbar for a numeric x; bar (with error bars) if x is
    categorical (e.g. a swept "strategy")."""
    ax = fig.add_subplot(111)
    summary = summary.sort_values(x)
    stds = summary["std"].fillna(0)
    counts = summary["count"]
    is_categorical = summary[x].dtype == object

    if is_categorical:
        ax.bar(summary[x].astype(str), summary["mean"], yerr=stds)
    else:
        ax.errorbar(summary[x], summary["mean"], yerr=stds, marker="o", capsize=4)

    ax.set_xlabel(x)
    ax.set_ylabel(metric)
    n = int(counts.iloc[0]) if len(counts) else 0
    ax.set_title(f"mean of N={n} runs per point")
    return ax


def render_heatmap(fig, summary, x: str, y: str, metric: str, *, ax=None, vmin=None, vmax=None):
    if ax is None:
        ax = fig.add_subplot(111)
    pivot = summary.pivot(index=y, columns=x, values="mean")
    count_pivot = summary.pivot(index=y, columns=x, values="count")
    mesh = ax.pcolormesh(
        pivot.columns.astype(str), pivot.index.astype(str), pivot.values, vmin=vmin, vmax=vmax, cmap="viridis",
    )
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    counts = count_pivot.values[count_pivot.notna().values]
    n = int(counts.flat[0]) if counts.size else 0
    ax.set_title(f"mean of N={n} runs per cell")
    fig.colorbar(mesh, ax=ax)
    return mesh


def render_faceted_heatmaps(fig, summary, x: str, y: str, facet: str, metric: str):
    """Small-multiple heatmaps, one per distinct `facet` value, sharing
    one vmin/vmax scale and one colorbar so cells are comparable across
    facets at a glance."""
    facet_values = sorted(summary[facet].unique(), key=str)
    n = max(1, len(facet_values))
    vmin, vmax = summary["mean"].min(), summary["mean"].max()

    axes = fig.subplots(1, n, squeeze=False)[0]
    mesh = None
    for ax, value in zip(axes, facet_values):
        sub = summary[summary[facet] == value]
        pivot = sub.pivot(index=y, columns=x, values="mean")
        mesh = ax.pcolormesh(
            pivot.columns.astype(str), pivot.index.astype(str), pivot.values, vmin=vmin, vmax=vmax, cmap="viridis",
        )
        ax.set_title(f"{facet}={value}")
        ax.set_xlabel(x)
        ax.set_ylabel(y)

    if mesh is not None:
        fig.colorbar(mesh, ax=list(axes))
    return axes

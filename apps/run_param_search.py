"""Best response per design: tune a strategy's numbers to the robot.

The strategy sweep answers `score(design, strategy)` for six hand-written
strategies. This answers the question a build team actually asks -- what
is the best this design can do -- by giving each design point its own
tuned copy of a strategy instead of judging every robot by numbers
written for one.

Structure is held fixed. The rules, their order, their triggers and
tactics are exactly what was loaded; only continuous fields move
(`for_duration`, `min_duration`, `cooldown`, `cluster_radius`,
`max_range`, `standoff`, `engage_range`). What comes out is therefore a
strategy file a drive team can read, not a model that has to be
distilled into one -- see common_sim/control/strategy_params.py.

Run:
    python -m apps.run_param_search --help
    python -m apps.run_param_search --strategy cycle_coral --generations 12
    python -m apps.run_param_search --sweep max_speed=130,150,170 --out tuned/
    python -m apps.run_param_search --estimate-only         # just size the run

Sizing it: `--estimate-only` times this machine and prints how long the
configured search will take on it before committing anything. The
underlying measurement is `python -m apps.run_calibration`.

Phase 3, `--hall-of-fame archive.json`: grade every candidate against a
sampled field -- some of a persistent archive of past winners plus the
six hand-written strategies -- instead of one fixed `--opponent`, and
report exploitability (how much the single best counter in that field
beats the winner by) alongside score. See
`common_sim/analysis/hall_of_fame.py` for why a fixed opponent is a
trap: a strategy graded against it discovers counters to *that*
opponent, and the score alone cannot tell you that's what happened. The
winner is added back to the archive on disk when the run finishes, so
the field gets harder to beat over successive runs. Costs
`(--hof-sample + 6)` times as many matches per candidate as a single
fixed opponent -- 5-10x per the development-path artifact.
"""
from __future__ import annotations

import argparse
import json
import sys
from fractions import Fraction
from pathlib import Path

from common_sim.analysis.calibration import estimate_seconds, humanize, measure
from common_sim.analysis.hall_of_fame import Archive, HallOfFameEvaluator, describe_payoffs
from common_sim.analysis.param_search import (
    AllianceScoreEvaluator, confirm, default_population, matches_required, search_parameters,
)
from common_sim.analysis.runner import default_worker_count
from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, apply_variable, characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io, strategy_params
from game_specific.reefscape.sweep_trial import SEARCH_DT, STRATEGIES_DIR, SWEEP_DT, run_trial
from apps.run_calibration import reference_jobs
from apps.run_strategy_sweep import build_characteristics

# Seeds do nothing unless something in the trial consumes them, and with
# the default (disabled) model nothing does -- see param_search's module
# docstring. These are benchmarks/bench.py's figures, chosen there to
# exercise perturbation on characteristics, start poses and piece scatter
# at once; reusing them keeps "the noise the search sees" the same
# quantity as "the noise the benchmarks report".
SEARCH_VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10, max_speed_pct=0.08,
    max_accel_pct=0.08, start_pose_xy_in=2.0, start_pose_heading_deg=3.0, piece_scatter_in=3.0,
)

MATCH = MatchSpec(auto_duration=15.0, teleop_duration=135.0)

# The "six hand-written strategies" the development-path artifact means
# by that phrase -- every strategy file in the repo, always included in
# a hall-of-fame field alongside the archive sample, so the field can
# never shrink to nothing before an archive exists.
HAND_WRITTEN_NAMES = (
    "algae_processor", "auto_then_cycle", "cycle_coral",
    "cycle_coral_evasive", "endgame_defense", "full_defense",
)


def load_hand_written() -> dict:
    return {name: strategy_io.to_dict(strategy_io.load_strategy(STRATEGIES_DIR / f"{name}.json"))
            for name in HAND_WRITTEN_NAMES}


def build_roster(strategy_name: str, opponent: str, partner: str) -> list:
    """A 2v1: the robot being tuned, a partner that does not change, and
    an opponent. Not a bare 1v0 -- a strategy tuned with the field to
    itself learns timings that evaporate the moment anyone contests a
    piece, and contention is most of what the timing parameters are for."""
    char = characteristics_to_spec(build_characteristics())
    return [
        RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1,
                  characteristics=dict(char), strategy=strategy_name),
        RobotSpec(label="PARTNER", alliance="blue", roster_index=0,
                  characteristics=dict(char), strategy=partner),
        RobotSpec(label="OPPONENT", alliance="red", roster_index=0,
                  characteristics=dict(char), strategy=opponent),
    ]


def _parse_dt(text: str) -> float:
    return float(Fraction(text))


def _parse_design_points(text: str | None) -> list:
    """`--sweep max_speed=130,150,170` -> [("max_speed", 130.0), ...],
    or a single unnamed point when no sweep was asked for.

    One axis only, on purpose. The point of Phase 2 is a *corrected*
    design number per point, and each point costs a whole search; a
    two-axis grid of searches is an overnight run that should be sized
    with `--estimate-only` first and launched deliberately, by scripting
    this entry point, rather than reached by typing a second `--sweep`.
    """
    if not text:
        return [(None, None)]
    if "=" not in text:
        raise SystemExit(f"--sweep wants path=v1,v2,... (got {text!r})")
    path, values = text.split("=", 1)
    return [(path.strip(), float(v)) for v in values.split(",") if v.strip()]


def _label(path, value) -> str:
    return "baseline design" if path is None else f"{path}={value:g}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--strategy", default="cycle_coral",
                        help="strategy to tune, by name in the strategies dir (default cycle_coral)")
    parser.add_argument("--opponent", default="full_defense",
                        help="red robot's strategy (default full_defense)")
    parser.add_argument("--partner", default="cycle_coral",
                        help="blue partner's strategy, held fixed (default cycle_coral)")
    parser.add_argument("--generations", type=int, default=12, help="CMA-ES generations (default 12)")
    parser.add_argument("--population", type=int, default=None,
                        help="candidates per generation (default: CMA-ES's 4+3ln(N))")
    parser.add_argument("--seeds", type=int, default=8,
                        help="matches per candidate, same seed set for all (default 8)")
    parser.add_argument("--confirm-seeds", type=int, default=16,
                        help="fresh seeds to re-score the winner on, correcting the best-of-N "
                             "selection bias in the search's own number (default 16; 0 to skip)")
    parser.add_argument("--sigma", type=float, default=0.25,
                        help="initial step, as a fraction of each parameter's range (default 0.25)")
    parser.add_argument("--rng-seed", type=int, default=0, help="CMA-ES sampling seed (default 0)")
    parser.add_argument("--dt", type=_parse_dt, default=SEARCH_DT,
                        help=f"control timestep (default {SEARCH_DT:.5f} = SEARCH_DT = 1/30)")
    parser.add_argument("--sweep", default=None,
                        help="design axis to search per point, e.g. max_speed=130,150,170")
    parser.add_argument("--out", default=None,
                        help="directory to write each tuned strategy into as JSON")
    parser.add_argument("--estimate-only", action="store_true",
                        help="time this machine, print what the run would cost, and exit")
    parser.add_argument("--serial", action="store_true", help="run trials in-process (debugging)")
    parser.add_argument("--hall-of-fame", default=None, metavar="ARCHIVE.JSON",
                        help="Phase 3: grade against an archive of past winners plus the six "
                             "hand-written strategies instead of --opponent alone, report "
                             "exploitability, and add the winner back to this file (default: off)")
    parser.add_argument("--hof-sample", type=int, default=4,
                        help="archive entries sampled into the field per generation, "
                             "on top of the six hand-written strategies (default 4)")
    parser.add_argument("--hof-archive-cap", type=int, default=20,
                        help="max archive entries kept -- the weakest is evicted, not the "
                             "oldest (default 20)")
    args = parser.parse_args()

    strategy_path = STRATEGIES_DIR / f"{args.strategy}.json"
    payload = strategy_io.to_dict(strategy_io.load_strategy(strategy_path))
    refs = strategy_params.continuous_params(payload)
    if not refs:
        raise SystemExit(
            f"{strategy_path.name} has no searchable continuous parameters.\n"
            "  Every number in it is either structural (priority, counts) or unset (null).\n"
            "  Nothing here can improve it -- that is a job for a structure search.")

    design_points = _parse_design_points(args.sweep)
    population = args.population or default_population(len(refs))

    hand_written = load_hand_written() if args.hall_of_fame else {}
    archive = Archive.load(args.hall_of_fame) if args.hall_of_fame else Archive()
    field_size = args.hof_sample + len(hand_written) if args.hall_of_fame else 1
    per_search = matches_required(
        args.generations, population, args.seeds * field_size, args.confirm_seeds * field_size)
    total = per_search * len(design_points)

    if args.hall_of_fame:
        print(f"Tuning {args.strategy!r} vs a hall-of-fame field "
              f"({len(archive)} archived, sampling {args.hof_sample} + {len(hand_written)} "
              f"hand-written = {field_size} opponents/candidate) at dt=1/{1 / args.dt:.0f}")
    else:
        print(f"Tuning {args.strategy!r} vs {args.opponent!r} at dt=1/{1 / args.dt:.0f}")
    print(f"{len(refs)} searchable parameters:")
    print(strategy_params.describe(refs))
    confirm_note = "" if not args.confirm_seeds else f", + {2 * args.confirm_seeds * field_size} to confirm"
    print(f"\n{len(design_points)} design point(s) x {args.generations} generations "
          f"x {population} candidates x {args.seeds} seeds x {field_size} opponent(s)"
          f"{confirm_note} = {total:,} matches\n")

    if args.estimate_only:
        workers = 1 if args.serial else default_worker_count()
        print(f"Timing 16 reference matches on {workers} worker(s)...")
        throughput = measure(run_trial, reference_jobs(16, args.dt), parallel=not args.serial)
        print(f"  {throughput.matches_per_hour:,.0f} matches/hour on this machine")
        print(f"  this run: {humanize(estimate_seconds(total, throughput))}")
        print(f"  per design point: {humanize(estimate_seconds(per_search, throughput))}")
        return

    out_dir = Path(args.out) if args.out else None
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for path, value in design_points:
        label = _label(path, value)
        robots = build_roster(args.strategy, args.opponent, args.partner)
        if path is not None:
            robots = [apply_variable(r, path, value) if r.label == "PRIMARY" else r for r in robots]

        if args.hall_of_fame:
            evaluator = HallOfFameEvaluator(
                run_trial, robots=robots, match=MATCH, variability=SEARCH_VARIABILITY,
                strategies_dir=STRATEGIES_DIR, dt=args.dt, target_label="PRIMARY",
                opponent_label="OPPONENT", alliance="blue", opponent_alliance="red",
                archive=archive, hand_written=hand_written, sample_size=args.hof_sample,
                seeds=args.seeds, rng_seed=args.rng_seed, parallel=not args.serial,
            )
        else:
            evaluator = AllianceScoreEvaluator(
                run_trial, robots=robots, match=MATCH, variability=SEARCH_VARIABILITY,
                strategies_dir=STRATEGIES_DIR, dt=args.dt, target_label="PRIMARY",
                alliance="blue", seeds=args.seeds, parallel=not args.serial,
            )

        print(f"--- {label} " + "-" * max(0, 60 - len(label)))
        result = search_parameters(
            payload, evaluator, generations=args.generations, sigma=args.sigma,
            population_size=args.population, seed=args.rng_seed, progress=_print_progress,
        )
        print(result.summary())

        winner_fitness = result.fitness
        if args.hall_of_fame:
            # The search's own best-so-far can come from any generation;
            # re-grade the actual winner once more so the payoff matrix
            # and exploitability reported below describe *it*, not
            # whichever candidate happened to run last.
            winner_fitness = evaluator([result.payload])[0]
            print(f"  payoff matrix ({field_size} opponents):")
            print(describe_payoffs(evaluator.last_payoffs[0]))

        checked = None
        if args.confirm_seeds:
            # A seed set disjoint from the search's, so the winner is
            # scored on matches it was never selected against. Without
            # this the reported gain includes a best-of-N fit to the
            # search's own handful of piece scatters and start poses.
            if args.hall_of_fame:
                holdout = HallOfFameEvaluator(
                    run_trial, robots=robots, match=MATCH, variability=SEARCH_VARIABILITY,
                    strategies_dir=STRATEGIES_DIR, dt=args.dt, target_label="PRIMARY",
                    opponent_label="OPPONENT", alliance="blue", opponent_alliance="red",
                    archive=archive, hand_written=hand_written, sample_size=args.hof_sample,
                    seeds=args.confirm_seeds, base_seed=args.seeds, rng_seed=args.rng_seed + 1,
                    parallel=not args.serial,
                )
            else:
                holdout = AllianceScoreEvaluator(
                    run_trial, robots=robots, match=MATCH, variability=SEARCH_VARIABILITY,
                    strategies_dir=STRATEGIES_DIR, dt=args.dt, target_label="PRIMARY",
                    alliance="blue", seeds=args.confirm_seeds, base_seed=args.seeds,
                    parallel=not args.serial,
                )
            checked = confirm(result, holdout)
            print(f"  {checked.summary()}")
            if args.hall_of_fame:
                winner_fitness = checked.tuned
                print(f"  holdout payoff matrix ({field_size} opponents):")
                print(describe_payoffs(holdout.last_payoffs[1]))
            # Measured on the first real run of this tool: a 6-generation
            # search over 4 seeds reported +9.5 points and held on to
            # +0.2 of them. Best-of-N over a thin seed set buys a fit to
            # those particular piece scatters, and the search's own
            # number cannot show that -- so say it here, where the two
            # figures sit next to each other.
            if result.improvement > 1.0 and checked.improvement < 0.5 * result.improvement:
                print(f"  NOTE: {1 - checked.improvement / result.improvement:.0%} of the "
                      f"search's gain did not survive fresh seeds. {args.seeds} seeds per "
                      "candidate is\n        too thin to separate strategies from luck at this "
                      "effect size -- raise --seeds before --generations.")
        print()
        results.append((label, result, checked))

        name = f"{args.strategy}_tuned" + ("" if path is None else f"_{path}_{value:g}")
        if args.hall_of_fame:
            # Keep the strongest, not the newest: a design point whose
            # tuned strategy scored worse than the field it faced does
            # not get to push a real winner out of a capped archive.
            archive = archive.add(name, dict(result.payload, name=name), winner_fitness,
                                   max_size=args.hof_archive_cap)
            archive.save(args.hall_of_fame)
            print(f"  archived as {name!r} (fitness {winner_fitness:.1f}); "
                  f"{len(archive)} strategies now in {args.hall_of_fame}")
            print()

        if out_dir is not None:
            tuned = dict(result.payload, name=name)
            (out_dir / f"{name}.json").write_text(json.dumps(tuned, indent=2), encoding="utf-8")
            print(f"wrote {out_dir / (name + '.json')}")
            print()

    if len(results) > 1:
        confirmed = all(c is not None for _, _, c in results)
        source = "held-out seeds" if confirmed else "the search's own seeds"
        print(f"Corrected design numbers -- mean blue score on {source}:")
        print(f"  {'design point':<24} {'hand-written':>13} {'tuned':>9} {'gain':>8}")
        for label, result, checked in results:
            pair = checked if checked is not None else result
            baseline = pair.baseline if checked is not None else result.baseline_fitness
            tuned = pair.tuned if checked is not None else result.fitness
            print(f"  {label:<24} {baseline:13.1f} {tuned:9.1f} {tuned - baseline:+8.1f}")
        print("\nThe 'hand-written' column is what a strategy sweep would have reported for "
              "each design.\nThe 'tuned' column is what that design can actually do.")
        if not confirmed:
            print("Both columns carry the search's best-of-N selection bias -- "
                  "rerun without --confirm-seeds 0.")

    print(f"\nWinners are worth re-running at dt={SWEEP_DT:.5f} (1/60) for inspection and "
          "MATCH-tab replay;\nthe search step is coarser on purpose (see sweep_trial.SEARCH_DT).")


def _print_progress(record) -> None:
    if isinstance(record, str):
        print(f"  {record}")
        return
    failed = f"  {record.failures} failed" if record.failures else ""
    print(f"  gen {record.index:>3}  best {record.best:7.1f}  mean {record.mean:7.1f}  "
          f"best-so-far {record.best_so_far:7.1f}  sigma {record.sigma:.3f}  "
          f"{record.seconds:5.1f}s{failed}")
    sys.stdout.flush()


if __name__ == "__main__":
    main()

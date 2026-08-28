"""Campaign control flow, artifact retention, and the morning report.

These cover the paths a *clean* night never exercises. A campaign that passes
tells you nothing about what happens when a match fails: whether the WPILOG is
kept, whether the report names the seed, whether three consecutive setup
failures stop the run instead of burning eight hours. Those are exactly the
paths somebody depends on at 7am, on the one morning they matter.

Driven by a fake runner rather than a real JVM, so they run in CI -- which is
why `bridge.harness` imports without pyntcore or websocket-client.
"""
from __future__ import annotations

import json

import pytest

from bridge import harness as hz
from bridge import operator as op
from bridge.scenario import Action, ScenarioGenerator
from bridge.harness import (
    FAIL,
    HARNESS_ERROR,
    PASS,
    Campaign,
    MatchResult,
    MatchRunner,
    render_report,
)
from bridge.oracles import ERROR, WARNING, Finding


def _result(index: int, seed: int, status: str, findings=(), **kw) -> MatchResult:
    return MatchResult(
        index=index,
        seed=seed,
        status=status,
        started_at="2026-08-24T20:00:00",
        wall_seconds=kw.pop("wall_seconds", 40.0),
        boot_seconds=kw.pop("boot_seconds", 11.0),
        findings=[f.__dict__ for f in findings],
        **kw,
    )


class FakeRunner:
    """Stands in for MatchRunner. Returns whatever the test asks for."""

    def __init__(self, statuses, findings_by_index=None):
        self.statuses = statuses
        self.findings_by_index = findings_by_index or {}
        self.calls: list[tuple[int, int]] = []

    def run(self, index: int, seed: int) -> MatchResult:
        self.calls.append((index, seed))
        return _result(
            index, seed, self.statuses[index], self.findings_by_index.get(index, ())
        )


# ---------------------------------------------------------------------------
# campaign control flow
# ---------------------------------------------------------------------------


def test_seeds_advance_and_every_match_is_recorded(tmp_path):
    runner = FakeRunner([PASS, PASS, PASS])
    campaign = Campaign(runner, tmp_path, matches=3, first_seed=4711)
    campaign.run()

    assert runner.calls == [(0, 4711), (1, 4712), (2, 4713)]
    written = [json.loads(l) for l in campaign.jsonl_path.read_text().splitlines()]
    assert [r["seed"] for r in written] == [4711, 4712, 4713]
    assert campaign.stopped_early is None


def test_consecutive_harness_errors_abort_the_night(tmp_path):
    """Three setup failures in a row means the setup is broken, not the robot."""
    runner = FakeRunner([HARNESS_ERROR] * 10)
    campaign = Campaign(runner, tmp_path, matches=10, first_seed=1,
                        abort_after_consecutive_errors=3)
    campaign.run()

    assert len(campaign.results) == 3, "should have given up rather than run all ten"
    assert "consecutive harness errors" in campaign.stopped_early


def test_errors_that_are_not_consecutive_do_not_abort(tmp_path):
    runner = FakeRunner([HARNESS_ERROR, PASS, HARNESS_ERROR, PASS, HARNESS_ERROR])
    campaign = Campaign(runner, tmp_path, matches=5, first_seed=1,
                        abort_after_consecutive_errors=3)
    campaign.run()

    assert len(campaign.results) == 5
    assert campaign.stopped_early is None


def test_results_are_flushed_as_they_happen(tmp_path):
    """The JSONL must survive the machine dying at 3am."""
    seen: list[int] = []

    def peek(result):
        # Read the file from inside the loop: what is on disk right now is what
        # would survive a power cut at this instant.
        seen.append(len(campaign.jsonl_path.read_text().splitlines()))

    runner = FakeRunner([PASS, PASS, PASS])
    campaign = Campaign(runner, tmp_path, matches=3, first_seed=1, on_result=peek)
    campaign.run()

    assert seen == [1, 2, 3]


# ---------------------------------------------------------------------------
# artifact retention -- the ~8.5 GB question
# ---------------------------------------------------------------------------


def _fake_repo(tmp_path):
    repo = tmp_path / "robot"
    (repo / hz.BRIDGE_LOG_DIR).mkdir(parents=True)
    (repo / "build.gradle").write_text("// fake", encoding="utf-8")
    return repo


def test_passing_matches_delete_their_logs(tmp_path):
    repo = _fake_repo(tmp_path)
    workdir = tmp_path / "runs"
    runner = MatchRunner(repo, workdir)

    scratch = workdir / "match-0000-seed1"
    scratch.mkdir(parents=True)
    (scratch / "console.log").write_text("clean", encoding="utf-8")
    produced = repo / hz.BRIDGE_LOG_DIR / "akit_new.wpilog"
    produced.write_bytes(b"x" * 1024)

    result = _result(0, 1, PASS)
    runner._collect_artifacts(set(), scratch, result)

    assert not produced.exists(), "a passing match must not leave 45 MB behind"
    assert not scratch.exists()
    assert result.artifacts == {}


def test_failing_matches_keep_everything(tmp_path):
    repo = _fake_repo(tmp_path)
    workdir = tmp_path / "runs"
    runner = MatchRunner(repo, workdir)

    scratch = workdir / "match-0007-seed4711"
    scratch.mkdir(parents=True)
    (scratch / "console.log").write_text("boom", encoding="utf-8")
    produced = repo / hz.BRIDGE_LOG_DIR / "akit_new.wpilog"
    produced.write_bytes(b"x" * 1024)

    result = _result(7, 4711, FAIL)
    runner._collect_artifacts(set(), scratch, result)

    assert not produced.exists(), "the log should have moved, not been copied"
    assert (scratch / "robot.wpilog").read_bytes() == b"x" * 1024
    assert (scratch / "findings.json").is_file()
    assert set(result.artifacts) == {"robot", "console", "findings"}


def test_pre_existing_logs_are_left_alone(tmp_path):
    """Only logs this match produced are ours to delete."""
    repo = _fake_repo(tmp_path)
    workdir = tmp_path / "runs"
    runner = MatchRunner(repo, workdir)

    older = repo / hz.BRIDGE_LOG_DIR / "akit_someone_elses.wpilog"
    older.write_bytes(b"keep me")
    before = {older}

    scratch = workdir / "match-0000-seed1"
    scratch.mkdir(parents=True)
    runner._collect_artifacts(before, scratch, _result(0, 1, PASS))

    assert older.exists()


def test_an_unrenamed_log_is_still_reaped(tmp_path):
    """A crash between opening and renaming the log must not leak it.

    AdvantageKit opens under a hash name and renames once it knows the time.
    Snapshotting the directory catches either name; parsing the console for
    the rename line would have missed exactly this case.
    """
    repo = _fake_repo(tmp_path)
    workdir = tmp_path / "runs"
    runner = MatchRunner(repo, workdir)

    scratch = workdir / "match-0000-seed1"
    scratch.mkdir(parents=True)
    hashed = repo / hz.BRIDGE_LOG_DIR / "akit_e935953bc0c959d7.wpilog"
    hashed.write_bytes(b"x")

    runner._collect_artifacts(set(), scratch, _result(0, 1, PASS))
    assert not hashed.exists()


# ---------------------------------------------------------------------------
# what the operator does next -- where the false-positive rate is really set
# ---------------------------------------------------------------------------


class _Pose:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def distance_to(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


MID_FIELD = _Pose(8.0, 4.0)
DRIVING = Action("drive", 1.0, axes={op.AXIS_LEFT_Y: -0.5})


def _runner(tmp_path):
    return MatchRunner(tmp_path / "robot", tmp_path / "runs")


def _is_back_off(action) -> bool:
    # Labels carry the attempt number ("back-off1"), which escalates.
    return action.label.startswith("back-off")


def test_a_moving_robot_just_keeps_playing(tmp_path):
    gen = ScenarioGenerator(1)
    action, backoffs = _runner(tmp_path).choose_action(
        gen, _Pose(9.0, 4.0), MID_FIELD, DRIVING, 0
    )
    assert not _is_back_off(action)
    assert backoffs == 0


def test_running_into_something_produces_a_back_off(tmp_path):
    """The hub, most often -- mid-field, so the wall margin never sees it."""
    gen = ScenarioGenerator(1)
    barely_moved = _Pose(MID_FIELD.x + 0.01, MID_FIELD.y)
    action, backoffs = _runner(tmp_path).choose_action(
        gen, barely_moved, MID_FIELD, DRIVING, 0
    )
    assert _is_back_off(action)
    assert backoffs == 1


def test_the_operator_keeps_trying_up_to_its_budget(tmp_path):
    gen = ScenarioGenerator(1)
    runner = _runner(tmp_path)
    barely_moved = _Pose(MID_FIELD.x + 0.01, MID_FIELD.y)

    for attempt in range(runner.MAX_CONSECUTIVE_BACKOFFS):
        action, backoffs = runner.choose_action(gen, barely_moved, MID_FIELD, DRIVING, attempt)
        assert _is_back_off(action), f"gave up after {attempt}, budget is {runner.MAX_CONSECUTIVE_BACKOFFS}"
        assert backoffs == attempt + 1


def test_the_operator_eventually_gives_up(tmp_path):
    """A robot that will not reverse after several tries is genuinely wedged.

    Helping forever would suppress exactly the finding the campaign exists to
    produce, so the detector has to be allowed to see it.
    """
    gen = ScenarioGenerator(1)
    runner = _runner(tmp_path)
    barely_moved = _Pose(MID_FIELD.x + 0.01, MID_FIELD.y)
    spent = runner.MAX_CONSECUTIVE_BACKOFFS

    action, backoffs = runner.choose_action(gen, barely_moved, MID_FIELD, DRIVING, spent)
    assert not _is_back_off(action), "must stop helping and let frozen-robot fire"
    assert backoffs == spent, "still stuck, so patience must not reset"


def test_a_wall_takes_priority_and_resets_patience(tmp_path):
    gen = ScenarioGenerator(1)
    runner = _runner(tmp_path)
    corner = _Pose(0.4, 0.4)
    action, backoffs = runner.choose_action(
        gen, corner, corner, DRIVING, runner.MAX_CONSECUTIVE_BACKOFFS
    )
    assert action.label == "recover"
    assert backoffs == 0


def test_not_moving_during_a_non_drive_action_is_not_stuck(tmp_path):
    """Standing still while spinning up the flywheel is the correct behaviour."""
    gen = ScenarioGenerator(1)
    spin_up = Action("spin-up", 1.5, buttons=frozenset({op.BTN_LEFT_BUMPER}))
    action, _ = _runner(tmp_path).choose_action(gen, MID_FIELD, MID_FIELD, spin_up, 0)
    assert not _is_back_off(action)


def test_a_very_short_action_is_not_judged(tmp_path):
    """0.15 s of drive covers almost nothing even when everything works."""
    gen = ScenarioGenerator(1)
    blink = Action("drive(cut)", 0.2, axes={op.AXIS_LEFT_Y: -0.5})
    action, _ = _runner(tmp_path).choose_action(gen, MID_FIELD, MID_FIELD, blink, 0)
    assert not _is_back_off(action)


def test_no_pose_yet_does_not_crash_or_back_off(tmp_path):
    gen = ScenarioGenerator(1)
    action, backoffs = _runner(tmp_path).choose_action(gen, None, None, None, 0)
    assert not _is_back_off(action)
    assert backoffs == 0


# ---------------------------------------------------------------------------
# the morning report
# ---------------------------------------------------------------------------


def _campaign_with(results, tmp_path, matches=None, stopped=None):
    campaign = Campaign(FakeRunner([]), tmp_path, matches=matches or len(results), first_seed=1)
    campaign.results = results
    campaign.stopped_early = stopped
    return campaign


def test_report_names_the_seed_and_how_to_reproduce_it(tmp_path):
    finding = Finding("liveness", "frozen-robot", ERROR, "commanded but stationary", "t=41.2s")
    results = [_result(0, 4711, PASS), _result(1, 4712, FAIL, [finding])]
    report = render_report(_campaign_with(results, tmp_path))

    assert "1 pass, 1 FAIL" in report
    assert "WHAT TO LOOK AT" in report
    assert "seed 4712" in report
    assert "frozen-robot" in report
    assert "--first-seed 4712" in report, "the report must say how to re-run the failure"


def test_a_finding_in_every_match_is_flagged_as_environmental(tmp_path):
    """Otherwise the same known-good warning drowns the report every morning."""
    everywhere = Finding("faults", "alert-warning", WARNING, "Vision camera 0 is disconnected.", "alerts")
    once = Finding("liveness", "frozen-robot", ERROR, "stuck", "t=9s")
    results = [
        _result(0, 1, PASS, [everywhere]),
        _result(1, 2, PASS, [everywhere]),
        _result(2, 3, FAIL, [everywhere, once]),
    ]
    report = render_report(_campaign_with(results, tmp_path))

    environmental_line = next(l for l in report.splitlines() if "alert-warning" in l)
    assert "environmental" in environmental_line
    frozen_line = next(l for l in report.splitlines() if "frozen-robot" in l and "matches" in l)
    assert "environmental" not in frozen_line


def test_a_one_match_campaign_calls_nothing_environmental(tmp_path):
    """In a campaign of one, every finding is "in every match" and none of it means anything.

    Reproducing a single reported failure is exactly the case where the report
    must not tell the reader to ignore the finding they came to look at.
    """
    finding = Finding("liveness", "frozen-robot", ERROR, "stuck", "t=16.9s")
    report = render_report(_campaign_with([_result(0, 31337, FAIL, [finding])], tmp_path))

    assert "frozen-robot" in report
    assert "environmental" not in report


def test_an_empty_campaign_is_not_reported_as_a_clean_night(tmp_path):
    report = render_report(_campaign_with([], tmp_path, matches=50))
    assert "Nothing ran" in report
    assert "clean" not in report.lower().replace("do not read this as a clean night", "")


def test_harness_errors_are_not_counted_as_passes(tmp_path):
    results = [_result(0, 1, PASS), _result(1, 2, HARNESS_ERROR, note="boot timed out")]
    report = render_report(_campaign_with(results, tmp_path))

    assert "1 pass, 0 FAIL, 1 harness error" in report
    assert "boot timed out" in report
    assert "No failures" not in report


def test_report_says_when_the_campaign_stopped_short(tmp_path):
    results = [_result(0, 1, PASS)]
    report = render_report(_campaign_with(results, tmp_path, matches=200, stopped="interrupted"))

    assert "STOPPED" in report
    assert "1 run of 200 planned" in report


def test_report_records_whether_the_oracles_were_preflighted(tmp_path):
    report = render_report(_campaign_with([_result(0, 1, PASS)], tmp_path),
                           preflight="skipped -- oracle 02 UNVERIFIED for this campaign")
    assert "UNVERIFIED" in report


def test_clean_night_says_so_plainly(tmp_path):
    results = [_result(i, i, PASS) for i in range(5)]
    report = render_report(_campaign_with(results, tmp_path))
    assert "No failures" in report
    assert "WHAT TO LOOK AT" not in report


# ---------------------------------------------------------------------------
# driver selection
# ---------------------------------------------------------------------------


def test_an_unknown_driver_is_refused_at_construction(tmp_path):
    """Not at match time. A campaign that discovers its driver name is a
    typo after booting the first JVM has already spent a minute on it, and
    an overnight run would spend the whole night on `error` statuses."""
    with pytest.raises(ValueError, match="unknown driver"):
        hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver="stragety")


def test_both_drivers_are_selectable_and_default_to_scripted(tmp_path):
    """Scripted stays the default deliberately: it is what the
    false-positive work in step 3 was tuned against, and changing what an
    unqualified `run_bridge_overnight.py` does would silently re-baseline
    every number in this README."""
    assert hz.MatchRunner(repo=tmp_path, workdir=tmp_path).driver == hz.SCRIPTED
    for name in hz.DRIVERS:
        assert hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=name).driver == name


def test_the_drive_model_is_measured_once_and_reused(tmp_path):
    """A property of the robot code, not of a match. Re-measuring per
    match would spend four seconds each time confirming a constant."""
    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY)
    assert runner.limits is None, "nothing measured until a match needs it"

    handed_in = object()
    reused = hz.MatchRunner(
        repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY, limits=handed_in
    )
    assert reused.limits is handed_in


def test_the_strategy_is_overridable_without_touching_the_harness(tmp_path):
    """The harness runs *a* strategy many times; it does not own which
    one. Overriding this is how a campaign compares two."""
    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY, shoot_at=7)
    strategy = runner.strategy(seed=1)
    assert [r.name for r in strategy.rules] == ["shoot_fuel", "collect_fuel", "wait_at_the_goal"]
    staging = next(r for r in strategy.rules if r.name == "wait_at_the_goal")
    assert staging.priority < min(
        r.priority for r in strategy.rules if r.name != "wait_at_the_goal"
    ), "staging is what to do when there is nothing else to do; collecting outranks it"

    shoot = next(r for r in strategy.rules if r.name == "shoot_fuel")
    held = next(t for t in shoot.trigger.triggers if hasattr(t, "min_count"))
    assert held.min_count == 7, "shoot_at reaches the trigger it configures"


def test_the_shoot_rule_also_waits_for_a_live_hub(tmp_path):
    """REBUILT's 25-second clock, in the campaign as well as the demo. A
    shoot rule that fires on ball count alone sends the robot to stand in
    a goal that is not accepting."""
    from common_sim.control.triggers import ScoringAvailable

    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY)
    shoot = next(r for r in runner.strategy(seed=1).rules if r.name == "shoot_fuel")
    assert any(isinstance(t, ScoringAvailable) for t in shoot.trigger.triggers)


class _Check:
    """Stands in for drive_model.Calibration."""

    def __init__(self, label="probe", speed=0.0, direction=0.0, omega=0.0):
        self.label = label
        self.speed_error = speed
        self.direction_error_deg = direction
        self.omega_error = omega


def test_a_drive_model_that_agrees_lets_the_campaign_start(tmp_path):
    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY)
    runner._require_model_agrees([_Check(), _Check(speed=0.01, direction=2.0)])


def test_a_wrong_drive_model_aborts_rather_than_logging_a_finding(tmp_path):
    """A finding says "this match went wrong". This says "every match from
    here commands the wrong velocity", which is a harness error -- and
    three of those end the campaign. The alternative is eight hours of
    matches that all look plausible and all navigated to the wrong place.
    """
    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY)
    with pytest.raises(RuntimeError, match="disagrees with the drive"):
        runner._require_model_agrees([_Check(label="forward full", speed=1.2)])


def test_direction_gets_its_own_budget(tmp_path):
    """Speed and direction have different noise floors -- the direction
    comparison goes through a frame round trip whose two headings are
    sampled a frame apart. One lumped tolerance would be either loose
    enough to hide a scaling error or tight enough to fail every run."""
    runner = hz.MatchRunner(repo=tmp_path, workdir=tmp_path, driver=hz.STRATEGY)
    runner._require_model_agrees([_Check(direction=runner.DIRECTION_TOLERANCE_DEG - 0.1)])
    with pytest.raises(RuntimeError):
        runner._require_model_agrees([_Check(direction=runner.DIRECTION_TOLERANCE_DEG + 0.1)])


# -- the populated field ------------------------------------------------


def test_a_campaign_is_solo_unless_asked_otherwise(tmp_path):
    """The default must be the field this ran on before the extras existed.

    A populated field is a different experiment, not a better version of
    the same one -- it reaches states a solo run cannot, and it spends
    real time with somebody wedged against somebody else. Two nights are
    worth comparing; one night silently swapped for the other is not.
    """
    runner = MatchRunner(tmp_path / "robot", tmp_path / "runs")
    assert (runner.opponents, runner.partners) == (0, 0)
    assert runner._deploy_cast(state=None, view=None) is None


def test_a_contested_campaign_needs_the_strategy_driver(monkeypatch, tmp_path):
    """Refused at the command line rather than ignored at run time.

    The extras have no decisions without a strategy layer, so a scripted
    campaign asked for opponents would run solo matches and file them
    under a report that says "3 opponents" -- and nothing in the output
    would say otherwise.

    Deliberately run with no robot project reachable. Whether two flags
    contradict each other is a fact about the command line, so the check has
    to come before the project is located -- and without pinning that here,
    this test passes on any machine with a robot checkout beside it and fails
    only in CI, which is the worst place to find out.
    """
    from apps import run_bridge_overnight as app
    from bridge import sim_process

    monkeypatch.setenv(sim_process.ROBOT_REPO_ENV, str(tmp_path / "no-such-project"))

    with pytest.raises(SystemExit):
        app.main(["--driver", hz.SCRIPTED, "--opponents", "3", "--matches", "1"])


def test_the_offline_topic_fallbacks_match_the_real_ones():
    """`harness` hardcodes two NT keys for the no-pyntcore path. They drift.

    The fallbacks exist so the report half of this module imports in CI, where
    `robot_state` cannot. That is also exactly why nothing else would ever
    notice them going stale -- the path that uses them is the path with no way
    to check itself.
    """
    rs = pytest.importorskip("bridge.robot_state")
    assert hz.POSE_TRUTH == rs.POSE_TRUTH
    assert hz.BRIDGE_ROBOT_POSES == rs.BRIDGE_ROBOT_POSES


def _one_match_campaign(tmp_path, **result_fields):
    runner = FakeRunner([PASS])
    campaign = Campaign(runner, tmp_path, matches=1, first_seed=1)
    campaign.run()
    for key, value in result_fields.items():
        setattr(campaign.results[0], key, value)
    return campaign


def test_a_campaign_report_names_the_invariants_it_could_not_check(tmp_path):
    """"No findings" from a stood-down detector reads like a clean night."""
    campaign = _one_match_campaign(
        tmp_path,
        invariants_inactive=[
            "command-out-of-range: no calibrated drive limits were supplied"
        ],
    )
    report = hz.render_report(campaign)
    assert "INVARIANTS NOT CHECKED" in report
    assert "command-out-of-range" in report


def test_a_clean_campaign_does_not_grow_a_not_checked_block(tmp_path):
    assert "INVARIANTS NOT CHECKED" not in hz.render_report(_one_match_campaign(tmp_path))

"""Is the pin rule live or inert?

648 matches produced zero pin fouls with PinRule(max_seconds=3.0)
configured. Two very different explanations:

  (a) Defend._respect_pin_limit releases at _PIN_RELEASE_FRACTION (0.7,
      so 2.1s of the 3.0s limit) and the defender self-regulates. Zero
      fouls is then the system working.
  (b) is_pinning's four-way conjunction never holds, the clock never
      starts, and defenders face no pin constraint at all -- in which
      case a search will evolve a defender that pins forever and score
      it as legal.

Telling them apart needs the *clock*, not the foul count. If clocks climb
to ~2.1s and reset, it is (a). If they never accumulate, it is (b), and
the per-condition counters say which condition is the blocker.
"""
import collections
import statistics
import sys

sys.argv = ["x"]
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, SweepVariable, characteristics_to_spec, expand_jobs
from common_sim.analysis.variability import VariabilityModel
from common_sim.match.match import Match
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, build_match_for_job
from apps.run_strategy_sweep import build_characteristics

VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10, max_speed_pct=0.08,
    max_accel_pct=0.08, start_pose_xy_in=2.0, start_pose_heading_deg=3.0, piece_scatter_in=3.0,
)

C = collections.Counter()
peaks = []
_real_is_pinning = Match.is_pinning
_real_step_pins = Match._step_pins


def is_pinning(self, offender, victim):
    rule = self.field.pin_rule
    if rule is None:
        return False
    C["pair_ticks"] += 1
    asking = victim.commanded_speed > rule.stopped_speed
    stopped = victim.speed <= rule.stopped_speed
    contact = self.robots_in_contact(offender, victim)
    # Evaluate each independently -- the real method short-circuits, so
    # its own ordering would hide how often the later conditions hold.
    trapped = self._trapped_behind(victim, offender)
    C["asking"] += asking
    C["stopped"] += stopped
    C["contact"] += contact
    C["trapped"] += trapped
    C["asking+stopped"] += asking and stopped
    C["asking+stopped+contact"] += asking and stopped and contact
    C["all four"] += asking and stopped and contact and trapped
    return asking and stopped and contact and trapped


def step_pins(self, dt):
    _real_step_pins(self, dt)
    for timer in self._pin_timers.values():
        peaks.append(timer[0])


Match.is_pinning = is_pinning
Match._step_pins = step_pins

char = characteristics_to_spec(build_characteristics())
robots = [
    RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1, characteristics=dict(char), strategy="cycle_coral"),
    RobotSpec(label="PARTNER", alliance="blue", roster_index=0, characteristics=dict(char), strategy="cycle_coral"),
    RobotSpec(label="OPPONENT", alliance="red", roster_index=0, characteristics=dict(char), strategy="full_defense"),
]
jobs = expand_jobs(robots, MatchSpec(auto_duration=15.0, teleop_duration=135.0), VARIABILITY,
                   [SweepVariable(target="OPPONENT", path="strategy",
                                  values=("full_defense", "endgame_defense"))],
                   repetitions=3, strategies_dir=STRATEGIES_DIR, dt=1 / 60)

pin_events = 0
for job in jobs:
    match, _, _ = build_match_for_job(job)
    run_match_to_completion(match, dt=job.dt)
    pin_events += len(match.events.of_kind("pin_foul"))
    pin_events += sum(match.pin_fouls.values())

print(f"{len(jobs)} defensive matches at dt=1/60, rule = "
      f"max {match.field.pin_rule.max_seconds}s, release {match.field.pin_rule.release_seconds}s, "
      f"stopped_speed {match.field.pin_rule.stopped_speed}\n")

pt = C["pair_ticks"]
print(f"ordered-pair ticks evaluated: {pt:,}\n")
print("how often each condition holds on its own:")
for k in ("asking", "stopped", "contact", "trapped"):
    print(f"  {k:<28} {C[k]:>9,}  {C[k]/pt:6.2%}")
print("\nconjunctions (this is where it dies):")
for k in ("asking+stopped", "asking+stopped+contact", "all four"):
    print(f"  {k:<28} {C[k]:>9,}  {C[k]/pt:6.2%}")

print(f"\npin clock: {len(peaks):,} tick-samples with a live timer")
if peaks:
    nz = [p for p in peaks if p > 0]
    print(f"  max reached      {max(peaks):.3f}s   (limit {match.field.pin_rule.max_seconds}s, "
          f"Defend releases at {0.7 * match.field.pin_rule.max_seconds:.2f}s)")
    if nz:
        print(f"  mean while live  {statistics.mean(nz):.3f}s")
        print(f"  p90 while live   {sorted(nz)[int(len(nz) * 0.9)]:.3f}s")
else:
    print("  clock NEVER started -- the conjunction never held")
print(f"\npin fouls charged: {pin_events}")

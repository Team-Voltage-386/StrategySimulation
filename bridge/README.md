# The maple-sim bridge

Drives the team's **real robot code**, running under maple-sim in WPILib's
simulator, from Python — so a competitive scenario can be generated on a night
when the robot is on the mechanical team's cart and the field doesn't exist yet.

Background and rationale: [The maple-sim Bridge](https://claude.ai/code/artifact/648dfe02-ea7d-4b2f-b0ee-44094eb28407).

**Status: steps 1–2 of 7 done.** The loop is closed, and the campaign can now
fail: both oracles are armed, quiet on clean operation, and proven to fire.
Next is the overnight harness.

## Run it

```
pip install -r bridge/requirements.txt
python apps/run_bridge_smoke.py      # step 1: does the loop close?
python apps/run_bridge_oracles.py    # step 2: can it tell a failure from a run?
```

Each takes ~90 s: gradle compiles, the JVM boots, the checks run, the sim is killed.

```
python apps/run_bridge_smoke.py --dump-topics    # everything the robot publishes
python apps/run_bridge_smoke.py --echo           # stream the robot console
python apps/run_bridge_smoke.py --attach         # against a sim you started yourself
python apps/run_bridge_oracles.py --no-provoke   # exercise only (leaves oracle 02 unverified)
```

`pytest test/test_bridge_oracles.py` covers oracle 01's rules with no JVM
involved, so they run in CI.

No JAVA_HOME needed — `sim_process.find_java_home` locates the WPILib JDK.

## Shape

| Module | Direction | Transport |
|---|---|---|
| `operator.py` | Python → robot | HALSim WebSocket, `ws://127.0.0.1:3300/wpilibws` |
| `robot_state.py` | robot → Python | NetworkTables 4, via `pyntcore` |
| `sim_process.py` | — | launches and reaps `gradlew simulateJava -Pbridge` |
| `oracles.py` | — | decides whether a run failed |

Input is injected **at the joystick layer**, not at the command layer. The
binding layer and its interlocks are a substantial part of what is being
tested, so bypassing them with NT-driven `Trigger`s would test less for no
less work.

The only robot-repo change is a gated block in `build.gradle`: `-Pbridge` swaps
the Sim GUI and the real-DriverStation extension for `halsim_ws_server`. Without
the flag, `simulateJava` behaves exactly as it always has.

## What the smoke test proves

Four checks, ordered so the first failure names the layer at fault.

1. **LINK** — WebSocket open, NT publishing. Transport only.
2. **ECHO** — with the robot still *disabled*, a distinctive stick pattern is
   pushed and read back off AdvantageKit's own DriverStation log. This is the
   important one: it separates "the wire works" from "the robot code reacted".
3. **AXIS** — hold the left stick; maple-sim's ground-truth pose moves ~1.25 m
   in 2 s and stops on release.
4. **BUTTON** — hold the left bumper; `flywheel.shootCommand` runs, the setpoint
   goes to 2200 and simulated wheel speed follows to ~230. A non-drive
   subsystem, so it isn't the drive path twice.

## The oracles

**01 — hard faults** (`FaultOracle`). Reads the captured console after the run,
plus AdvantageKit's `Alerts` arrays live. Reports stack traces (one finding
carrying its frames, not one per frame), `DriverStation.reportError/Warning`,
hard stops, and sustained loop overruns.

The muffle list is the part that decides whether anyone reads the morning report
twice. Every entry carries a written reason, so the next person can re-litigate
it instead of guessing what it was hiding. It currently muffles gradle output,
javac warnings, the JNI boilerplate line that contains the word "errors" and
reports nothing, our own NT teardown, and unpopulated-joystick warnings.

**02 — liveness** (`LivenessMonitor`). Samples NT at 20 Hz while the run
proceeds. Six detectors, each debounced by *duration* — a single sample of "not
moving" is scheduling jitter, two seconds of it is a bug — and each latched to
fire once per episode, re-arming only after the condition clears.

| kind | fires when |
|---|---|
| `robot-code-stalled` | AdvantageKit's timestamp stops advancing while NT is still up |
| `input-ignored` | stick pushed, drive commands nothing — the binding layer dropped it |
| `frozen-robot` | drive commanded, world not changing — pinned, wedged, or dead |
| `mechanism-not-following` | flywheel setpoint commanded, speed never gets near it |
| `loop-overrun-sustained` | cycle time above threshold continuously |
| `brownout` | the robot says so |

As with the Python-side stall audit, these are **detectors, not watchdogs**.
Nothing intervenes or nudges the robot back into motion; a detector that fixes
what it detects destroys its own evidence.

### An oracle that has never fired is not known to work

`run_bridge_oracles.py` runs two phases and requires both to come out right: an
**exercise** phase of ordinary operation that must be clean, and a **provoke**
phase that pins the robot against the field wall while still commanding it
forward, which must produce `frozen-robot`. Without the second, "0 findings"
from a broken detector is indistinguishable from "0 findings" from a clean run,
and the first silent regression turns every subsequent night into a rubber stamp.

Oracle 01 has no runtime equivalent — deliberately crashing robot code to prove
a grep works is a poor trade — so `test/test_bridge_oracles.py` stands in for it,
which is why `bridge.oracles` imports without pyntcore.

The report also tracks whether each phase *ran*. "No findings" and "never ran"
are the same empty list, and the first version of this cheerfully reported a run
that died during gradle configuration as "exercise phase clean".

## Two things found on the way

**Odometry is not an independent channel here.** `SimContainer.simulationPeriodic`
feeds maple-sim's ground-truth pose to `addVisionMeasurement` with standard
deviations of `(0, 0, 0)` — infinite confidence — so the pose estimator is pinned
to truth and re-snaps every cycle. `Odometry/Robot` and
`FieldSimulation/RobotPosition` are the same number, and anything that writes
odometry (`drive.setPose`, bound to the Y button) is undone within one loop. An
odometry-vs-truth divergence oracle would read zero forever until those std devs
are made realistic.

**`--no-daemon` is load-bearing.** With the gradle daemon, `gradlew` is a thin
client and the robot JVM is a child of the *daemon*; killing the client orphans a
robot still holding port 3300, and the next run fails to bind with no visible
cause. Without it the robot is a descendant of the process we started, so a
tree-kill ends it.

**Log volume.** `-Pbridge` turns on AdvantageKit's `WPILOGWriter` (it was
commented out in SIM), because a detected failure with nothing to replay is half
a deliverable. Logs land in the robot repo's gitignored `logs/bridge/` at roughly
**45 MB per 150 s match** — about 8.5 GB across an overnight run. The harness
will have to keep the failures and delete the rest.

## Next

Step 3 is the overnight harness: N seeded matches, a pass/fail summary, failing
logs kept and passing ones deleted. Everything it needs now exists — the console
is captured, the oracles report structured `Finding`s, and each run leaves a
replayable WPILOG.

After that: the NT world-state reader and `MapleMatchView` adapter, at which
point the strategy layer starts reacting to the live field and the game mismatch
starts to matter.

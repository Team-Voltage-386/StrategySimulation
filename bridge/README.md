# The maple-sim bridge

Drives the team's **real robot code**, running under maple-sim in WPILib's
simulator, from Python — so a competitive scenario can be generated on a night
when the robot is on the mechanical team's cart and the field doesn't exist yet.

Background and rationale: [The maple-sim Bridge](https://claude.ai/code/artifact/648dfe02-ea7d-4b2f-b0ee-44094eb28407).

**Status: step 1 of 7 — the loop is closed.** Python can launch the robot code,
read its state, and command it. Everything after this widens what it can command
and teaches it to recognise a failure.

## Run it

```
pip install -r bridge/requirements.txt
python apps/run_bridge_smoke.py
```

Expect ~90 s: gradle compiles, the JVM boots, four checks run, the sim is killed.

```
python apps/run_bridge_smoke.py --dump-topics   # everything the robot publishes
python apps/run_bridge_smoke.py --echo          # stream the robot console
python apps/run_bridge_smoke.py --attach        # against a sim you started yourself
```

No JAVA_HOME needed — `sim_process.find_java_home` locates the WPILib JDK.

## Shape

| Module | Direction | Transport |
|---|---|---|
| `operator.py` | Python → robot | HALSim WebSocket, `ws://127.0.0.1:3300/wpilibws` |
| `robot_state.py` | robot → Python | NetworkTables 4, via `pyntcore` |
| `sim_process.py` | — | launches and reaps `gradlew simulateJava -Pbridge` |

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

## Next

Per the brief's sequencing, the remaining spine is oracles 01 + 02 (fault grep
over the console log — already captured to `build/bridge/robot-console.log` — and
a liveness detector ported from the Python stall audit), then the overnight
harness. After that: the NT world-state reader and `MapleMatchView` adapter, at
which point the strategy layer starts reacting to the live field.

"""
Combined launcher for the mechanism-orchestration sandbox: starts/stops the
Java/WPILib sim (MechanismOrchestrationSandbox, a sibling repo) as a managed
subprocess, drives its actions from on-screen buttons instead of a physical
controller, and shows the same side-view (X-Z) MechanismCanvas as
run_mechanism_view.py -- all from one window, so testing a coordination
change doesn't mean juggling a separate terminal or a gamepad alongside
this one.

Action buttons press the robot's *actual* Xbox-controller bindings over the
HALSim WebSocket, the same technique (and the same bridge/operator.py
OperatorLink class) the sibling StrategySimulation repo's other robot-bridge
feature already uses -- see bridge/README.md's "Shape" table. That's a
deliberate choice, not just reuse for its own sake: it exercises the actual
binding layer (RobotContainer.configureBindings()) instead of a parallel
NT-driven shortcut that could drift out of sync with what a real driver's
controller would trigger. The action list below is scenario-specific --
swap it out (or make it a constructor argument) when a second scenario
needs different buttons.

Run: `python -m apps.run_mechanism_sandbox <path to MechanismOrchestrationSandbox>`
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from pyqtgraph.Qt import QtCore, QtWidgets

from bridge import operator as op
from bridge.nt4_client import NT4MechanismClient
from gui_utils import theme
from gui_utils.gearpeg_canvas import GearPegCanvas
from gui_utils.mechanism_canvas import MechanismCanvas
from gui_utils.ring_stack_canvas import RingStackCanvas

SIM_TASK = "simulateJavaRelease"
GRADLE_ARGS = [SIM_TASK, "--console=plain"]

POLL_HZ = 50
# Gap between a confirmed Stop and the next Start (see reset_sim) -- a
# small safety margin for the OS to finish tearing down the just-killed
# robot process before the next one tries to bind the same NT4 port.
RESTART_DELAY_MS = 1000
NT4_PORT = 5810

BRIDGE_CONNECT_TIMEOUT_S = 90.0
# How long a button press is held before releasing -- well over one DS
# period (20 ms) so CommandScheduler's onTrue() sees a real rising edge
# followed by a falling one; matches bridge/scenario.py's own tap duration.
TAP_HOLD_MS = 150

# How long a single continuous-cycle step (pickup / score / reset-settle) may
# run before it's declared stalled -- generous against any real coordinated
# move so a genuine stall (e.g. a collision interlock parking the arm) stops
# the loop with a log line instead of spinning silently forever.
CYCLE_STEP_TIMEOUT_S = 15.0

# (label, WPILib button number) -- drives RobotContainer.configureBindings()
# in the CubeShelfScenario demo.
CUBE_SHELF_ACTIONS = [
    ("Pickup Cube", op.BTN_LEFT_BUMPER),
    ("Score Low", op.BTN_X),
    ("Score High", op.BTN_Y),
]

# Drives GearPegRobotContainer.configureBindings() in the gear_peg demo --
# same three buttons, different verbs, since that demo has no "Cube" to name.
GEAR_PEG_ACTIONS = [
    ("Pickup Gear", op.BTN_LEFT_BUMPER),
    ("Score Low Peg", op.BTN_X),
    ("Score High Peg", op.BTN_Y),
]

# Drives RingStackRobotContainer.configureBindings() in the ring_stack demo. Only two actions,
# not three -- this demo has one pole with a growing stack, not a Low/High choice. The shared
# Continuous-cycle Low/High radios still exist in this window regardless (they aren't per-demo),
# so on the Java side BTN_Y is *also* bound to the same "Score Ring" command as BTN_X (see
# RingStackRobotContainer's "Only one score action" doc) -- picking "High" just presses the
# button that isn't listed as its own action button here, with the identical effect.
RING_STACK_ACTIONS = [
    ("Pickup Ring", op.BTN_LEFT_BUMPER),
    ("Score Ring", op.BTN_X),
]

# One robot process runs exactly one demo (see DemoContainer.java on the
# Java side), chosen by the "env" value below being passed through as the
# SANDBOX_DEMO environment variable when the sim JVM launches. Keyed by the
# same strings DemoContainer.create() switches on, so adding a demo means
# adding a case there *and* an entry here under the identical string.
DEMOS = {
    "cube_shelf": {
        "env": "cube_shelf",
        "canvas": MechanismCanvas,
        "actions": CUBE_SHELF_ACTIONS,
        "cycle_tooltip": (
            "Repeats Pickup Cube -> Score, backing safely out of the slot after each score and "
            "then respawning just the piece (not the mechanism) so there's always a fresh one "
            "to grab -- runs on loop until stopped."
        ),
    },
    "gear_peg": {
        "env": "gear_peg",
        "canvas": GearPegCanvas,
        "actions": GEAR_PEG_ACTIONS,
        "cycle_tooltip": (
            "Repeats Pickup Gear -> Score, waiting for the whole score sequence to finish "
            "before respawning just the piece (not the mechanism) so there's always a fresh "
            "one to grab -- runs on loop until stopped."
        ),
    },
    "ring_stack": {
        "env": "ring_stack",
        "canvas": RingStackCanvas,
        "actions": RING_STACK_ACTIONS,
        "cycle_tooltip": (
            "Repeats Pickup Ring -> Score onto the pole's next open slot, respawning just the "
            "ring (not the pole's own stack) between pieces, so the pole keeps climbing across "
            "cycles -- runs until the pole fills up and the loop naturally stalls out, or until "
            "stopped."
        ),
    },
}


def _pids_listening_on(port: int) -> list[int]:
    """PIDs of any process with a listening TCP socket on `port`.

    GradleRIO's simulate task forks the actual robot JVM and then lets the
    build task that forked it finish -- by design, so the robot keeps
    running independent of the build tool's own lifetime (e.g. so an IDE
    can attach a debugger to a long-lived process rather than one tied to
    a build invocation). That means the robot JVM is *not* a lasting
    descendant of the gradlew process we launch by the time Stop is
    clicked, so a tree-kill of our own launched process (taskkill /T)
    doesn't reach it -- confirmed by tracing the actual process tree live:
    the robot JVM's parent had already exited within seconds of startup.
    Hunting by NT4 port ownership instead unambiguously finds the real
    running robot program no matter how gradle forked it."""
    result = subprocess.run(["netstat", "-ano", "-p", "TCP"], capture_output=True, text=True)
    pids = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0] == "TCP" and parts[3] == "LISTENING" and parts[1].rsplit(":", 1)[-1] == str(port):
            pids.append(int(parts[4]))
    return pids


def _resolve_java_home(sandbox_path: Path) -> str | None:
    """Mirrors settings.gradle's own PUBLIC-folder lookup, so the launched
    gradlew sees the same JDK a manual `./gradlew` run would -- needed
    because a bare `java` often isn't on PATH at all on a machine that only
    has WPILib's bundled JDK."""
    prefs_path = sandbox_path / ".wpilib" / "wpilib_preferences.json"
    try:
        prefs = json.loads(prefs_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    year = prefs.get("projectYear")
    if not year:
        return None
    public = os.environ.get("PUBLIC", r"C:\Users\Public")
    jdk = Path(public) / "wpilib" / str(year) / "jdk"
    return str(jdk) if jdk.is_dir() else None


class SandboxWindow(QtWidgets.QMainWindow):
    def __init__(self, sandbox_path: Path, server: str, demo: str, gear_grip: str = "parallel"):
        super().__init__()
        self.sandbox_path = sandbox_path
        self.demo = demo
        self._demo_entry = DEMOS[demo]
        self.gear_grip = gear_grip
        title = f"sparky-sim -- mechanism sandbox [{sandbox_path.name}] -- {demo}"
        if demo == "gear_peg":
            title += f" ({gear_grip} grip)"
        self.setWindowTitle(title)

        self.client = NT4MechanismClient(server=server)
        self.canvas = self._demo_entry["canvas"]()

        # Created fresh per sim launch (see start_sim/stop_sim) -- it's
        # tied to that one JVM's WebSocket server instance.
        self.operator_link: op.OperatorLink | None = None
        self._operator_was_connected = False
        # Set from the background connect thread, read/cleared from _tick
        # on the GUI thread -- appending to a QPlainTextEdit off-thread
        # isn't safe, so the error crosses over as plain data instead.
        self._operator_link_error: str | None = None

        # Continuous-cycle state machine -- see _tick_cycle. Drives the same
        # action buttons a user would click, in a loop, so a coordination
        # change can be watched running back-to-back instead of one press
        # at a time.
        self.cycle_active = False
        self.cycle_level = op.BTN_X  # updated from the radio buttons in _start_cycle
        self.cycle_state: str | None = None  # "PICKUP" | "SCORE" | "RESET"
        self.cycle_step_started = False
        self.cycle_step_deadline = 0.0
        self.cycle_count = 0

        self.process = QtCore.QProcess(self)
        self.process.setWorkingDirectory(str(sandbox_path))
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._on_process_output)
        self.process.stateChanged.connect(self._on_process_state_changed)
        self.process.finished.connect(self._on_process_finished)

        java_home = _resolve_java_home(sandbox_path)
        # Built unconditionally (rather than only inside the `if java_home:` this used to be)
        # so SANDBOX_DEMO always gets set, even when java_home lookup fails and the launched
        # gradlew just falls back to inheriting the parent process's own environment.
        env = QtCore.QProcessEnvironment.systemEnvironment()
        if java_home:
            env.insert("JAVA_HOME", java_home)
            env.insert("PATH", str(Path(java_home) / "bin") + os.pathsep + env.value("PATH"))
        env.insert("SANDBOX_DEMO", self._demo_entry["env"])
        # Only meaningful to gear_peg's RobotContainer (see GearPegRobotContainer's "Grip mode"
        # doc), but harmless to always set -- cube_shelf's RobotContainer never reads it.
        env.insert("SANDBOX_GEAR_GRIP", self.gear_grip)
        self.process.setProcessEnvironment(env)

        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / POLL_HZ))

        # Launch the sim immediately rather than waiting for a manual "Run Sim" click -- every
        # demo this window can open is meant to be watched running, so requiring that first click
        # every time was just friction, not a real choice point (the button, and Stop/Restart,
        # are still there for whenever a code change needs a relaunch).
        self.start_sim()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)

        controls = QtWidgets.QHBoxLayout()
        self.start_stop_button = QtWidgets.QPushButton("Run Sim")
        self.start_stop_button.clicked.connect(self._on_start_stop_clicked)
        controls.addWidget(self.start_stop_button)

        self.reset_mechanism_button = QtWidgets.QPushButton("Reset Mechanism")
        self.reset_mechanism_button.setEnabled(False)
        self.reset_mechanism_button.setToolTip(
            "Snaps the elevator/arm/chassis back to their starting state in-process -- fast, no JVM restart."
        )
        self.reset_mechanism_button.clicked.connect(self._on_reset_mechanism_clicked)
        controls.addWidget(self.reset_mechanism_button)

        self.restart_button = QtWidgets.QPushButton("Restart Sim")
        self.restart_button.setEnabled(False)
        self.restart_button.setToolTip("Kills and relaunches the whole robot JVM -- slower, only needed after a code change.")
        self.restart_button.clicked.connect(self.reset_sim)
        controls.addWidget(self.restart_button)

        self.sim_gui_checkbox = QtWidgets.QCheckBox("Show WPILib Sim GUI (for testing with a real controller instead)")
        controls.addWidget(self.sim_gui_checkbox)

        controls.addStretch(1)
        self.status_label = QtWidgets.QLabel("Idle")
        self.status_label.setFont(theme.technical_font(11, bold=True))
        controls.addWidget(self.status_label)
        layout.addLayout(controls)

        actions = QtWidgets.QHBoxLayout()
        bridge_label = QtWidgets.QLabel("Actions:")
        actions.addWidget(bridge_label)
        self.action_buttons: list[QtWidgets.QPushButton] = []
        for label, button in self._demo_entry["actions"]:
            btn = QtWidgets.QPushButton(label)
            btn.setEnabled(False)
            btn.clicked.connect(lambda checked=False, b=button: self._trigger_action(b))
            actions.addWidget(btn)
            self.action_buttons.append(btn)

        actions.addSpacing(16)
        actions.addWidget(QtWidgets.QLabel("Continuous cycle:"))
        self.cycle_level_low = QtWidgets.QRadioButton("Low")
        self.cycle_level_low.setChecked(True)
        self.cycle_level_high = QtWidgets.QRadioButton("High")
        self.cycle_level_group = QtWidgets.QButtonGroup(self)
        self.cycle_level_group.addButton(self.cycle_level_low)
        self.cycle_level_group.addButton(self.cycle_level_high)
        actions.addWidget(self.cycle_level_low)
        actions.addWidget(self.cycle_level_high)

        self.cycle_button = QtWidgets.QPushButton("Start Continuous Cycle")
        self.cycle_button.setCheckable(True)
        self.cycle_button.setEnabled(False)
        self.cycle_button.setToolTip(self._demo_entry["cycle_tooltip"])
        self.cycle_button.toggled.connect(self._on_cycle_toggled)
        actions.addWidget(self.cycle_button)

        self.cycle_count_label = QtWidgets.QLabel("Pieces scored: 0")
        self.cycle_count_label.setFont(theme.technical_font(10))
        actions.addWidget(self.cycle_count_label)

        actions.addStretch(1)
        self.bridge_status_label = QtWidgets.QLabel("bridge: --")
        self.bridge_status_label.setFont(theme.technical_font(10))
        actions.addWidget(self.bridge_status_label)
        layout.addLayout(actions)

        layout.addWidget(self.canvas, stretch=1)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setFont(theme.technical_font(9))
        self.log.setFixedHeight(140)
        self.log.setStyleSheet(
            f"background-color: {theme.BG_PANEL}; color: {theme.TEXT_PRIMARY}; border: 1px solid {theme.BORDER};"
        )
        layout.addWidget(self.log)

        self.setCentralWidget(central)
        self.resize(900, 760)

    # -- sim process lifecycle -------------------------------------------

    def _on_start_stop_clicked(self) -> None:
        if self.process.state() == QtCore.QProcess.NotRunning:
            self.start_sim()
        else:
            self.stop_sim()

    def start_sim(self) -> None:
        gradlew = self.sandbox_path / "gradlew.bat"
        if not gradlew.is_file():
            self._append_log(f"No gradlew.bat found at {gradlew} -- wrong sandbox path?")
            return
        args = list(GRADLE_ARGS)
        if self.sim_gui_checkbox.isChecked():
            args.append("-PsimGui")
        self._append_log(f"$ gradlew.bat {' '.join(args)}")
        self.process.start(str(gradlew), args)
        self._start_operator_link()

    def _start_operator_link(self) -> None:
        """Connects a fresh OperatorLink in the background -- connect()
        itself blocks, retrying, until the JVM boots and its WebSocket
        server binds (see bridge/operator.py), which is too slow to do on
        the GUI thread. Progress crosses back to _tick via plain
        attributes rather than a Qt signal, since that's the only
        GUI-thread-safe channel a background thread has here."""
        self.operator_link = op.OperatorLink()
        self._operator_was_connected = False
        link = self.operator_link

        def _connect() -> None:
            try:
                link.connect(timeout=BRIDGE_CONNECT_TIMEOUT_S)
            except Exception as exc:  # surfaced via _tick, not raised here -- see class doc
                self._operator_link_error = str(exc)

        threading.Thread(target=_connect, name="operator-link-connect", daemon=True).start()

    def stop_sim(self) -> None:
        """Kills the launched gradlew process tree, then separately hunts
        down and kills whatever actually holds the NT4 port -- see
        _pids_listening_on for why the two aren't the same process."""
        if self.operator_link is not None:
            self.operator_link.close()
            self.operator_link = None
        if self.process.state() != QtCore.QProcess.NotRunning:
            pid = self.process.processId()
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)], capture_output=True)
            self.process.waitForFinished(3000)
        for pid in _pids_listening_on(NT4_PORT):
            self._append_log(f"--- killing orphaned robot process (PID {pid}, holding NT4 port {NT4_PORT}) ---")
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)], capture_output=True)

    def reset_sim(self) -> None:
        """Full restart (kill + relaunch the JVM) -- only needed after a
        code change. For snapping mechanism state back between test runs,
        _on_reset_mechanism_clicked is the fast path."""
        self._append_log("--- restarting sim (JVM relaunch) ---")
        self.stop_sim()
        QtCore.QTimer.singleShot(RESTART_DELAY_MS, self.start_sim)

    def _on_reset_mechanism_clicked(self) -> None:
        self.client.request_reset()
        self._append_log("--- reset mechanism requested (in-process, no JVM restart) ---")

    def _trigger_action(self, button: int) -> None:
        """Presses `button` for TAP_HOLD_MS then releases -- a rising edge
        followed by a falling one, the same shape bridge/scenario.py's
        `tap()` produces, but via a QTimer instead of a blocking sleep so
        the GUI thread never stalls on a click."""
        if self.operator_link is None or not self.operator_link.connected:
            return
        self.operator_link.set_button(button, True)
        QtCore.QTimer.singleShot(TAP_HOLD_MS, lambda: self._release_action(button))

    def _release_action(self, button: int) -> None:
        if self.operator_link is not None:
            self.operator_link.set_button(button, False)

    # -- continuous cycle --------------------------------------------------
    #
    # Repeats Pickup Cube -> Score (Low or High) -> new piece, forever, so a
    # coordination change can be watched running back-to-back instead of one
    # click at a time. Steps are sequenced off the same HoldingPiece /
    # PieceScored / SequenceBusy telemetry the canvas already draws from,
    # not a fixed delay -- a real coordinated move's duration depends on the
    # config under test, and a fixed wait would either lag a fast mechanism
    # or cut off a slow one. In particular, the SCORE step waits out
    # SequenceBusy, not just PieceScored: release() marks the cube scored
    # while the tip is still inside the wall, and only the rest of
    # ScoreOnShelf's bound command -- extracting back out along the tip axis
    # and retracting to PRE_SCORE -- actually backs the arm safely out of
    # the slot. Only the cube gets reset between pieces (request_new_piece,
    # not request_reset): a full mechanism reset would snap the elevator/arm
    # back instantly and cut that retreat short if it landed mid-sequence,
    # instead of letting the loop watch the same safe exit a single manual
    # score press already takes.

    def _on_cycle_toggled(self, checked: bool) -> None:
        if checked:
            self._start_cycle()
        else:
            self._stop_cycle()

    def _start_cycle(self) -> None:
        if self.operator_link is None or not self.operator_link.connected:
            self.cycle_button.blockSignals(True)
            self.cycle_button.setChecked(False)
            self.cycle_button.blockSignals(False)
            return
        self.cycle_level = op.BTN_Y if self.cycle_level_high.isChecked() else op.BTN_X
        self.cycle_count = 0
        self._update_cycle_count_label()
        self.cycle_active = True
        self.cycle_button.setText("Stop Continuous Cycle")
        self.cycle_level_low.setEnabled(False)
        self.cycle_level_high.setEnabled(False)
        for btn in self.action_buttons:
            btn.setEnabled(False)
        self.reset_mechanism_button.setEnabled(False)
        level_name = "High" if self.cycle_level == op.BTN_Y else "Low"
        self._append_log(f"--- starting continuous cycle (Score {level_name}) ---")
        self._enter_cycle_step("PICKUP")

    def _stop_cycle(self, reason: str | None = None) -> None:
        if not self.cycle_active:
            return
        self.cycle_active = False
        self.cycle_state = None
        if reason:
            self._append_log(f"--- continuous cycle stopped ({reason}); scored {self.cycle_count} piece(s) ---")
        else:
            self._append_log(f"--- continuous cycle stopped; scored {self.cycle_count} piece(s) ---")
        self.cycle_button.blockSignals(True)
        self.cycle_button.setChecked(False)
        self.cycle_button.blockSignals(False)
        self.cycle_button.setText("Start Continuous Cycle")
        self.cycle_level_low.setEnabled(True)
        self.cycle_level_high.setEnabled(True)
        connected = self.operator_link is not None and self.operator_link.connected
        for btn in self.action_buttons:
            btn.setEnabled(connected)

    def _enter_cycle_step(self, state: str) -> None:
        self.cycle_state = state
        self.cycle_step_started = False
        self.cycle_step_deadline = time.monotonic() + CYCLE_STEP_TIMEOUT_S

    def _update_cycle_count_label(self) -> None:
        self.cycle_count_label.setText(f"Pieces scored: {self.cycle_count}")

    def _tick_cycle(self) -> None:
        if not self.cycle_active:
            return
        if self.operator_link is None or not self.operator_link.connected:
            self._stop_cycle("bridge disconnected")
            return
        if time.monotonic() > self.cycle_step_deadline:
            self._stop_cycle(f"step '{self.cycle_state}' stalled for {CYCLE_STEP_TIMEOUT_S:.0f}s")
            return

        snapshot = self.canvas.snapshot
        if self.cycle_state == "PICKUP":
            if not self.cycle_step_started:
                self._trigger_action(op.BTN_LEFT_BUMPER)
                self.cycle_step_started = True
            elif snapshot.holding_piece:
                self._enter_cycle_step("SCORE")
        elif self.cycle_state == "SCORE":
            if not self.cycle_step_started:
                self._trigger_action(self.cycle_level)
                self.cycle_step_started = True
            elif snapshot.piece_scored and not snapshot.sequence_busy:
                # The score sequence -- including its retreat back out of the wall -- has now
                # fully finished, not just the release() call partway through it.
                self.cycle_count += 1
                self._update_cycle_count_label()
                self._enter_cycle_step("NEW_PIECE")
        elif self.cycle_state == "NEW_PIECE":
            if not self.cycle_step_started:
                self.client.request_new_piece()
                self.cycle_step_started = True
            elif not snapshot.piece_scored:
                self._enter_cycle_step("PICKUP")

    def _on_process_state_changed(self, state) -> None:
        running = state != QtCore.QProcess.NotRunning
        self.start_stop_button.setText("Stop Sim" if running else "Run Sim")
        self.restart_button.setEnabled(running)
        self.sim_gui_checkbox.setEnabled(not running)
        if state == QtCore.QProcess.Starting:
            self.status_label.setText("Starting...")
        elif state == QtCore.QProcess.Running:
            self.status_label.setText("Running")
        if not running:
            self._stop_cycle("sim stopped")
            for btn in self.action_buttons:
                btn.setEnabled(False)
            self.cycle_button.setEnabled(False)
            self.bridge_status_label.setText("bridge: --")

    def _on_process_finished(self, exit_code, exit_status) -> None:
        self.status_label.setText(f"Stopped (exit {exit_code})")
        self._append_log(f"--- sim exited, code {exit_code} ---")

    def _on_process_output(self) -> None:
        text = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in text.splitlines():
            self._append_log(line)

    def _append_log(self, line: str) -> None:
        self.log.appendPlainText(line)

    # -- mechanism view ----------------------------------------------------

    def _tick(self) -> None:
        self.canvas.snapshot = self.client.poll()
        self.canvas.update()
        self.reset_mechanism_button.setEnabled(self.canvas.snapshot.connected and not self.cycle_active)
        self._tick_operator_link()
        self._tick_cycle()

    def _tick_operator_link(self) -> None:
        if self._operator_link_error is not None:
            self._append_log(f"--- bridge connection failed: {self._operator_link_error} ---")
            self._operator_link_error = None

        link = self.operator_link
        connected = link is not None and link.connected
        for btn in self.action_buttons:
            btn.setEnabled(connected and not self.cycle_active)
        self.cycle_button.setEnabled(connected)
        if not connected:
            self._stop_cycle("bridge disconnected")

        if connected and not self._operator_was_connected:
            # Rising edge: the WebSocket just came up. Auto-enable teleop so
            # the action buttons work immediately -- there's no on-screen
            # equivalent of the Sim GUI's manual Enable toggle, and leaving
            # the robot disabled would make every click a silent no-op
            # (CommandScheduler drops non-runsWhenDisabled commands).
            link.teleop_enable()
            self._append_log("--- bridge connected, robot enabled ---")
        self._operator_was_connected = connected

        if link is None:
            self.bridge_status_label.setText("bridge: --")
        elif connected:
            self.bridge_status_label.setText("bridge: connected")
        else:
            self.bridge_status_label.setText("bridge: connecting...")

    def closeEvent(self, event) -> None:
        self.stop_sim()
        self.client.close()
        super().closeEvent(event)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sandbox_path", type=Path, help="path to the MechanismOrchestrationSandbox WPILib project")
    parser.add_argument("--server", default="127.0.0.1", help="NT4 server address (default: 127.0.0.1)")
    parser.add_argument(
        "--demo",
        choices=sorted(DEMOS),
        default="cube_shelf",
        help="which sandbox demo to launch (default: cube_shelf)",
    )
    parser.add_argument(
        "--gear-grip",
        choices=["parallel", "normal"],
        default="parallel",
        help=(
            "gear_peg only: how the wrist closes on the gear -- 'parallel' grabs it in-line with "
            "its own long axis (wrist angle == piece angle throughout); 'normal' grabs it facing "
            "straight down onto its upward-facing edge, wrist parallel to the floor at score "
            "(default: parallel). Ignored for cube_shelf."
        ),
    )
    args = parser.parse_args()

    sandbox_path = args.sandbox_path.resolve()
    if not (sandbox_path / "gradlew.bat").is_file():
        parser.error(f"{sandbox_path} doesn't look like a WPILib project (no gradlew.bat)")

    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = SandboxWindow(sandbox_path, args.server, args.demo, args.gear_grip)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()

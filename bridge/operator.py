"""Virtual operator: DriverStation and joystick input over the HALSim WebSocket.

The robot program loads WPILib's `halsim_ws_server` extension (see the robot
repo's build.gradle, `-Pbridge`) and listens on ws://127.0.0.1:3300/wpilibws.
Everything a Driver Station would normally supply then arrives from here
instead: enable/disable, mode, alliance station, match clock, and the stick.

Protocol, from allwpilib/simulation/halsim_ws_core/doc/hardware_ws_api.md --
each frame is one JSON object `{"type", "device", "data"}`, and keys prefixed
`>` flow toward the robot:

    {"type": "Joystick", "device": "0",
     "data": {">axes": [...], ">buttons": [...], ">povs": [...]}}
    {"type": "DriverStation", "device": "",
     "data": {">enabled": true, ">new_data": true, ...}}

`>new_data` is the latch. Joystick values written before it become visible to
robot code together, in one atomic step, exactly as a real DS packet would --
which is what keeps a two-axis motion from being seen half-applied.

Two threads, because one socket has to be both written and drained:

* the send thread pushes the current state at `TICK_HZ`, the way a real DS
  sends every 20 ms. Robot code that stops hearing from the DS is entitled to
  disable itself, so this is a heartbeat, not just a transport.
* `WebSocketApp.run_forever` reads. It has to: `halsim_ws_server` mirrors every
  HAL device write outward, thousands per second, and an unread socket would
  back up until the server blocked.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field

try:
    import websocket  # websocket-client
except ImportError as exc:  # pragma: no cover - depends on the install
    # The button numbers, the axis map and the state dataclasses are plain
    # data and have no business requiring a transport library. Keeping this
    # module importable without one is what lets the scenario generator and
    # the campaign report be unit-tested where no JVM exists.
    websocket = None  # type: ignore[assignment]
    _WEBSOCKET_ERROR = exc
else:
    _WEBSOCKET_ERROR = None

DEFAULT_URL = "ws://127.0.0.1:3300/wpilibws"

# A real DS sends at 50 Hz. Matching it keeps timing-dependent robot code --
# debounces, `whileTrue` durations, trigger thresholds -- in its normal regime.
TICK_HZ = 50.0

# WPILib's XboxController.Axis / .Button, so that a caller reads like the
# robot-side binding it is trying to trip.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 3
AXIS_RIGHT_X = 4
AXIS_RIGHT_Y = 5
AXIS_COUNT = 6

BTN_A = 1
BTN_B = 2
BTN_X = 3
BTN_Y = 4
BTN_LEFT_BUMPER = 5
BTN_RIGHT_BUMPER = 6
BTN_BACK = 7
BTN_START = 8
BTN_LEFT_STICK = 9
BTN_RIGHT_STICK = 10
BUTTON_COUNT = 10

POV_RELEASED = -1


@dataclass
class JoystickState:
    """One controller's worth of input, in the wire's own shape.

    Axis and button counts are fixed at Xbox-controller size rather than grown
    on demand: WPILib decides whether an axis "exists" from the length of the
    array it last received, and a short array makes `getRawAxis` report an
    unplugged-controller error instead of returning zero.
    """

    axes: list[float] = field(default_factory=lambda: [0.0] * AXIS_COUNT)
    buttons: list[bool] = field(default_factory=lambda: [False] * BUTTON_COUNT)
    povs: list[int] = field(default_factory=lambda: [POV_RELEASED])

    def payload(self) -> dict:
        return {">axes": list(self.axes), ">buttons": list(self.buttons), ">povs": list(self.povs)}


@dataclass
class DriverStationState:
    enabled: bool = False
    autonomous: bool = False
    test: bool = False
    estop: bool = False
    fms: bool = False
    ds: bool = True
    station: str = "blue1"
    match_time: float = -1.0
    game_data: str = ""

    def payload(self) -> dict:
        return {
            ">enabled": self.enabled,
            ">autonomous": self.autonomous,
            ">test": self.test,
            ">estop": self.estop,
            ">fms": self.fms,
            ">ds": self.ds,
            ">station": self.station,
            ">match_time": self.match_time,
            ">game_data": self.game_data,
        }


class OperatorLink:
    """A connection to the robot's HALSim WebSocket server.

    Usable as a context manager. All the mutators are safe to call from the
    scripting thread; they take a lock and the send thread picks the new state
    up on its next tick.
    """

    def __init__(
        self,
        url: str = DEFAULT_URL,
        num_joysticks: int = 2,
        tick_hz: float = TICK_HZ,
        connect_timeout: float = 60.0,
    ):
        self.url = url
        self.tick_hz = tick_hz
        # Used by `__enter__`, where there is nowhere to pass one. A cold
        # gradle build is well over the 60 s that suffices once it is warm.
        self.connect_timeout = connect_timeout
        self._lock = threading.Lock()
        self._ds = DriverStationState()
        self._joysticks = [JoystickState() for _ in range(num_joysticks)]

        self._ws: websocket.WebSocketApp | None = None
        self._reader: threading.Thread | None = None
        self._sender: threading.Thread | None = None
        self._stop = threading.Event()
        self._open = threading.Event()
        self._closed_reason: str | None = None

        # Counted rather than stored: the inbound stream is device mirroring we
        # do not consume yet, but "did the robot say anything at all" is the
        # cheapest liveness check there is.
        self.rx_count = 0
        self.tx_count = 0

    # -- lifecycle ---------------------------------------------------------

    def connect(self, timeout: float = 60.0, poll: float = 0.5) -> None:
        """Open the socket, retrying until the robot's JVM is listening.

        The retry loop is the point: gradle has to compile, the JVM has to
        boot, and the extension binds its port somewhere in the middle of that.
        Waiting here is simpler than trying to parse readiness out of the
        build log.
        """
        if websocket is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "the operator link needs websocket-client: "
                "pip install -r bridge/requirements.txt"
            ) from _WEBSOCKET_ERROR

        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                probe = websocket.create_connection(self.url, timeout=2.0)
                probe.close()
                break
            except Exception as exc:  # socket refused, DNS, handshake -- all "not yet"
                last_error = exc
                time.sleep(poll)
        else:
            raise TimeoutError(f"no HALSim WebSocket server at {self.url} after {timeout:.0f}s "
                               f"(last error: {last_error!r})")

        self._stop.clear()
        self._open.clear()
        self._ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        self._reader = threading.Thread(target=self._ws.run_forever, name="halsim-rx", daemon=True)
        self._reader.start()

        if not self._open.wait(timeout=10.0):
            raise TimeoutError(f"connected to {self.url} but the socket never opened")

        self._sender = threading.Thread(target=self._send_loop, name="halsim-tx", daemon=True)
        self._sender.start()

    def close(self) -> None:
        """Disable first, then hang up.

        Order matters. Dropping the socket on an enabled robot leaves motors
        commanded and, in an overnight run, leaves the next match's baseline
        polluted by the last one's momentum.
        """
        if self._ws is not None and self._open.is_set():
            try:
                self.set_enabled(False)
                self.neutral()
                time.sleep(3.0 / self.tick_hz)  # let a few packets carry it
            except Exception:
                pass
        self._stop.set()
        if self._sender is not None:
            self._sender.join(timeout=2.0)
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
        if self._reader is not None:
            self._reader.join(timeout=2.0)

    def __enter__(self) -> "OperatorLink":
        self.connect(timeout=self.connect_timeout)
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        return self._open.is_set() and not self._stop.is_set()

    @property
    def closed_reason(self) -> str | None:
        return self._closed_reason

    # -- driver station ----------------------------------------------------

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._ds.enabled = enabled

    def set_mode(self, *, autonomous: bool = False, test: bool = False) -> None:
        with self._lock:
            self._ds.autonomous = autonomous
            self._ds.test = test

    def set_station(self, station: str) -> None:
        """e.g. "blue1" / "red3". This is where DriverStation.getAlliance() comes from."""
        with self._lock:
            self._ds.station = station

    def set_match_time(self, seconds: float) -> None:
        with self._lock:
            self._ds.match_time = seconds

    def set_game_data(self, data: str) -> None:
        with self._lock:
            self._ds.game_data = data

    def teleop_enable(self, station: str | None = None) -> None:
        with self._lock:
            if station is not None:
                self._ds.station = station
            self._ds.autonomous = False
            self._ds.test = False
            self._ds.enabled = True

    def autonomous_enable(self, station: str | None = None) -> None:
        with self._lock:
            if station is not None:
                self._ds.station = station
            self._ds.autonomous = True
            self._ds.test = False
            self._ds.enabled = True

    def disable(self) -> None:
        self.set_enabled(False)

    # -- joystick ----------------------------------------------------------

    def set_axis(self, axis: int, value: float, *, joystick: int = 0) -> None:
        with self._lock:
            self._joysticks[joystick].axes[axis] = max(-1.0, min(1.0, float(value)))

    def set_button(self, button: int, pressed: bool, *, joystick: int = 0) -> None:
        """`button` is 1-indexed, matching WPILib's numbering."""
        with self._lock:
            self._joysticks[joystick].buttons[button - 1] = bool(pressed)

    def set_pov(self, degrees: int, *, joystick: int = 0, index: int = 0) -> None:
        with self._lock:
            self._joysticks[joystick].povs[index] = int(degrees)

    def neutral(self, *, joystick: int | None = None) -> None:
        """Release everything. Cheap insurance between scenario steps."""
        with self._lock:
            targets = self._joysticks if joystick is None else [self._joysticks[joystick]]
            for js in targets:
                js.axes = [0.0] * AXIS_COUNT
                js.buttons = [False] * BUTTON_COUNT
                js.povs = [POV_RELEASED] * len(js.povs)

    def tap(self, button: int, *, joystick: int = 0, hold: float = 0.12) -> None:
        """Press and release, blocking for `hold`.

        `hold` defaults to well over one DS period so an `onTrue` binding sees
        a genuine rising edge followed by a falling one -- a press shorter than
        a tick can be swallowed whole.
        """
        self.set_button(button, True, joystick=joystick)
        time.sleep(hold)
        self.set_button(button, False, joystick=joystick)

    # -- internals ---------------------------------------------------------

    def _on_open(self, _ws) -> None:
        self._open.set()

    def _on_message(self, _ws, _message) -> None:
        self.rx_count += 1

    def _on_error(self, _ws, error) -> None:
        self._closed_reason = repr(error)

    def _on_close(self, _ws, status_code, msg) -> None:
        self._open.clear()
        if self._closed_reason is None:
            self._closed_reason = f"closed status={status_code} msg={msg}"

    def _send_loop(self) -> None:
        period = 1.0 / self.tick_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            try:
                self._send_once()
            except Exception as exc:
                self._closed_reason = repr(exc)
                break
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # Fell behind (GC, a slow console write). Re-base rather than
                # try to catch up -- a burst of packets is worse than a gap.
                next_tick = time.monotonic()

    def _send_once(self) -> None:
        with self._lock:
            frames = [
                {"type": "Joystick", "device": str(i), "data": js.payload()}
                for i, js in enumerate(self._joysticks)
            ]
            ds = dict(self._ds.payload())
        # The latch goes last, after every joystick has been written, so the
        # robot sees one coherent snapshot.
        ds[">new_data"] = True
        frames.append({"type": "DriverStation", "device": "", "data": ds})

        assert self._ws is not None
        for frame in frames:
            self._ws.send(json.dumps(frame))
        self.tx_count += 1

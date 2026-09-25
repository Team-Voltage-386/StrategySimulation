"""
NT4 client for the Java mechanism-orchestration sandbox (a sibling repo,
MechanismOrchestrationSandbox -- see its telemetry/MechanismPublisher.java)
-- subscribes to the /Mechanism/* topics it publishes and hands back the
latest values as a plain snapshot for gui_utils/mechanism_canvas.py to draw.

Lives in bridge/, not common_sim/, because it needs a live NetworkTables
connection to a robot process -- the same reason robot_state.py does. See
bridge/__init__.py's import direction and test/test_import_contract.py,
which enforces it. common_sim.telemetry.mechanism_snapshot holds the
dataclass shape alone, with no ntcore dependency, for the drawing code to
import instead of this module.

The Java side publishes plain doubles rather than a struct-typed Pose2d, so
this only needs pyntcore -- not the separate wpimath Python package -- to
decode chassis pose.
"""
from __future__ import annotations

from common_sim.telemetry.mechanism_snapshot import MechanismSnapshot

try:
    import ntcore  # pyntcore
except ImportError:  # pragma: no cover - depends on the install
    # Matches robot_state.py's pattern: importable without pyntcore installed,
    # so a headless caller that never constructs NT4MechanismClient doesn't
    # need the dependency at all. The constructor below fails loudly instead.
    ntcore = None  # type: ignore[assignment]

TABLE_NAME = "Mechanism"


class NT4MechanismClient:
    """Connects as an NT4 client to a robot (real or simulated) publishing
    under /Mechanism. Polled from a Qt timer rather than pushed, so the
    GUI thread never blocks on network I/O -- ntcore runs its own
    listener thread; reading a subscriber's last value is just a
    lock-protected field access."""

    def __init__(self, server: str = "127.0.0.1", identity: str = "mechanism-viewer"):
        if ntcore is None:
            raise RuntimeError(
                "pyntcore is not installed, so nothing can be read back from the robot. "
                "pip install -r bridge/requirements.txt"
            )
        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._inst.startClient4(identity)
        self._inst.setServer(server)
        table = self._inst.getTable(TABLE_NAME)
        self._x = table.getDoubleTopic("ChassisXMeters").subscribe(0.0)
        self._y = table.getDoubleTopic("ChassisYMeters").subscribe(0.0)
        self._heading = table.getDoubleTopic("ChassisHeadingRad").subscribe(0.0)
        self._elevator = table.getDoubleTopic("ElevatorHeightMeters").subscribe(0.0)
        self._arm = table.getDoubleTopic("ArmAngleRadians").subscribe(0.0)
        self._piece = table.getBooleanTopic("HoldingPiece").subscribe(False)
        self._piece_x = table.getDoubleTopic("PieceXMeters").subscribe(0.0)
        self._piece_z = table.getDoubleTopic("PieceZMeters").subscribe(0.0)
        self._piece_angle = table.getDoubleTopic("PieceAngleRad").subscribe(0.0)
        self._piece_scored = table.getBooleanTopic("PieceScored").subscribe(False)
        self._gripper_colliding = table.getBooleanTopic("GripperColliding").subscribe(False)
        self._sequence_busy = table.getBooleanTopic("SequenceBusy").subscribe(False)
        # Only published by the gear_peg and ring_stack demos' publishers; subscribing here
        # regardless of which demo is running is harmless -- an absent topic just holds its
        # default.
        self._wrist = table.getDoubleTopic("WristAngleRadians").subscribe(0.0)
        # Only published by the ring_stack demo's RingStackPublisher.
        self._arm_length = table.getDoubleTopic("ArmLengthMeters").subscribe(0.0)
        self._rings_on_pole = table.getDoubleTopic("RingsOnPole").subscribe(0.0)
        # A monotonically increasing counter, not a boolean pulse -- see
        # RobotContainer.checkResetRequest() on the Java side for why (a
        # pulse could land between two of the robot's polls and be missed;
        # an id *change* can't be).
        self._reset_request_id_pub = table.getIntegerTopic("ResetRequestId").publish()
        self._reset_request_id = 0
        # Same pattern, but for respawning just the Cube -- see
        # RobotContainer.checkNewPieceRequest() on the Java side.
        self._new_piece_request_id_pub = table.getIntegerTopic("NewPieceRequestId").publish()
        self._new_piece_request_id = 0

    def request_reset(self) -> None:
        """Resets the mechanism (elevator/arm/chassis pose) in-process on
        the robot, without restarting its JVM -- see Superstructure.reset()
        and RobotContainer.checkResetRequest() on the Java side."""
        self._reset_request_id += 1
        self._reset_request_id_pub.set(self._reset_request_id)

    def request_new_piece(self) -> None:
        """Respawns the Cube in-process, leaving the elevator/arm/chassis wherever they
        already are -- see RobotContainer.checkNewPieceRequest() on the Java side. This is
        the piece-only counterpart to request_reset(), for a continuous pickup/score loop
        that wants the next piece without snapping the mechanism out from under a sequence
        that's still retreating from the wall."""
        self._new_piece_request_id += 1
        self._new_piece_request_id_pub.set(self._new_piece_request_id)

    def close(self) -> None:
        self._inst.stopClient()

    def poll(self) -> MechanismSnapshot:
        return MechanismSnapshot(
            connected=self._inst.isConnected(),
            chassis_x_m=self._x.get(),
            chassis_y_m=self._y.get(),
            chassis_heading_rad=self._heading.get(),
            elevator_height_m=self._elevator.get(),
            arm_angle_rad=self._arm.get(),
            holding_piece=self._piece.get(),
            piece_x_m=self._piece_x.get(),
            piece_z_m=self._piece_z.get(),
            piece_angle_rad=self._piece_angle.get(),
            piece_scored=self._piece_scored.get(),
            gripper_colliding=self._gripper_colliding.get(),
            sequence_busy=self._sequence_busy.get(),
            wrist_angle_rad=self._wrist.get(),
            arm_length_m=self._arm_length.get(),
            rings_on_pole=int(self._rings_on_pole.get()),
        )

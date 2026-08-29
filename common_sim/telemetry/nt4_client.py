"""
NT4 client for the Java mechanism-orchestration sandbox (a sibling repo,
MechanismOrchestrationSandbox -- see its telemetry/MechanismPublisher.java)
-- subscribes to the /Mechanism/* topics it publishes and hands back the
latest values as a plain snapshot for gui_utils/mechanism_canvas.py to draw.

The Java side publishes plain doubles rather than a struct-typed Pose2d, so
this only needs pyntcore -- not the separate wpimath Python package -- to
decode chassis pose.
"""
from __future__ import annotations

from dataclasses import dataclass

import ntcore

TABLE_NAME = "Mechanism"


@dataclass
class MechanismSnapshot:
    connected: bool = False
    chassis_x_m: float = 0.0
    chassis_y_m: float = 0.0
    chassis_heading_rad: float = 0.0
    elevator_height_m: float = 0.0
    arm_angle_rad: float = 0.0
    holding_piece: bool = False
    piece_x_m: float = 0.0
    piece_z_m: float = 0.0
    # A held piece is rigid with the arm, so it tilts with it -- the canvas has
    # to draw that, since the robot-side collision check now uses the tilted
    # footprint (see Shelf.orientedBoxCollides on the Java side).
    piece_angle_rad: float = 0.0
    piece_scored: bool = False
    gripper_colliding: bool = False


class NT4MechanismClient:
    """Connects as an NT4 client to a robot (real or simulated) publishing
    under /Mechanism. Polled from a Qt timer rather than pushed, so the
    GUI thread never blocks on network I/O -- ntcore runs its own
    listener thread; reading a subscriber's last value is just a
    lock-protected field access."""

    def __init__(self, server: str = "127.0.0.1", identity: str = "mechanism-viewer"):
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
        # A monotonically increasing counter, not a boolean pulse -- see
        # RobotContainer.checkResetRequest() on the Java side for why (a
        # pulse could land between two of the robot's polls and be missed;
        # an id *change* can't be).
        self._reset_request_id_pub = table.getIntegerTopic("ResetRequestId").publish()
        self._reset_request_id = 0

    def request_reset(self) -> None:
        """Resets the mechanism (elevator/arm/chassis pose) in-process on
        the robot, without restarting its JVM -- see Superstructure.reset()
        and RobotContainer.checkResetRequest() on the Java side."""
        self._reset_request_id += 1
        self._reset_request_id_pub.set(self._reset_request_id)

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
        )

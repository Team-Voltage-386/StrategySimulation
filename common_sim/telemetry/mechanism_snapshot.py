"""
The data shape published by the Java mechanism-orchestration sandbox (a
sibling repo, MechanismOrchestrationSandbox -- see its telemetry/
MechanismPublisher.java) over NT4, and read back by
gui_utils/mechanism_canvas.py and gui_utils/gearpeg_canvas.py to draw.

Split out from the NT4 client itself (see bridge/nt4_client.py) so that a
plain dataclass with no ntcore dependency is what the drawing code imports.
common_sim and gui_utils must never depend on ntcore or the bridge -- see
ARCHITECTURE.md's import contract and test/test_import_contract.py -- and
the client that actually talks NT4 belongs in bridge/ for the same reason
the maple-sim bridge does: it needs a live NetworkTables connection, which
a headless sweep or a spawn worker has no business requiring.
"""
from __future__ import annotations

from dataclasses import dataclass


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
    # True while a bound pickup/score sequence is still running on the robot -- including a
    # score sequence's retreat back out of the wall, which finishes well after PieceScored
    # flips true (see MechanismPublisher.java). A continuous-cycle caller waits for this to
    # clear before asking for the next piece, so it never interrupts that retreat mid-motion.
    sequence_busy: bool = False
    # gear_peg and ring_stack demos only -- angle of the wrist joint relative to the arm link
    # (see frc.robot.gearpeg.subsystems.Wrist / frc.robot.ringstack.subsystems.Wrist on the Java
    # side). Stays at its default 0.0 when polling a cube_shelf robot, which publishes no
    # WristAngleRadians topic at all.
    wrist_angle_rad: float = 0.0
    # ring_stack demo only -- extension length of the telescoping arm (see
    # frc.robot.ringstack.subsystems.ExtendingArm on the Java side). This demo's arm has no
    # angle to publish (it never rotates), so it publishes ArmLengthMeters instead of relying on
    # arm_angle_rad, which stays at its default 0.0 here.
    arm_length_m: float = 0.0
    # ring_stack demo only -- how many rings are already stacked on the pole (see
    # frc.robot.ringstack.game.Pole on the Java side). Stays at its default 0 for the other two
    # demos, which publish no RingsOnPole topic.
    rings_on_pole: int = 0

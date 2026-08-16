from common_sim.control.input_sources import (
    DriveCommand,
    GamepadInput,
    KeyBindings,
    KeyboardInput,
    OperatorCommand,
)

# Stand-ins for real Qt key codes -- KeyboardInput never interprets
# these, just does set-membership checks, so plain ints work fine.
KEY_W, KEY_S, KEY_A, KEY_D = 1, 2, 3, 4
KEY_LEFT, KEY_RIGHT = 5, 6
KEY_SPACE, KEY_F = 7, 8

BINDINGS = KeyBindings(
    forward=KEY_W, backward=KEY_S, left=KEY_A, right=KEY_D,
    rotate_ccw=KEY_LEFT, rotate_cw=KEY_RIGHT,
    intake=KEY_SPACE, deposit=KEY_F,
)


def test_keyboard_input_no_keys_pressed_is_zero():
    src = KeyboardInput(pressed_keys=set, bindings=BINDINGS)
    drive, operator = src.poll()
    assert drive == DriveCommand(0.0, 0.0, 0.0)
    assert operator == OperatorCommand(False, False, None)


def test_keyboard_input_maps_wasd_to_drive_axes():
    pressed = {KEY_W, KEY_D}
    src = KeyboardInput(pressed_keys=lambda: pressed, bindings=BINDINGS)
    drive, _ = src.poll()
    assert drive.vx == 1.0
    assert drive.vy == 1.0
    assert drive.omega == 0.0


def test_keyboard_input_opposite_keys_cancel_out():
    pressed = {KEY_W, KEY_S, KEY_A, KEY_D}
    src = KeyboardInput(pressed_keys=lambda: pressed, bindings=BINDINGS)
    drive, _ = src.poll()
    assert drive.vx == 0.0
    assert drive.vy == 0.0


def test_keyboard_input_reads_live_pressed_set_each_poll():
    pressed = set()
    src = KeyboardInput(pressed_keys=lambda: pressed, bindings=BINDINGS)
    assert src.poll()[0].vy == 0.0
    pressed.add(KEY_W)
    assert src.poll()[0].vy == 1.0


def test_keyboard_input_operator_commands_and_deposit_action():
    pressed = {KEY_SPACE, KEY_F}
    src = KeyboardInput(pressed_keys=lambda: pressed, bindings=BINDINGS, deposit_action="l4")
    _, operator = src.poll()
    assert operator.intake_active is True
    assert operator.deposit_active is True
    assert operator.deposit_action == "l4"


class FakeJoystick:
    def __init__(self, axes, buttons):
        self._axes = axes
        self._buttons = buttons

    def get_numaxes(self):
        return len(self._axes)

    def get_axis(self, index):
        return self._axes[index]

    def get_numbuttons(self):
        return len(self._buttons)

    def get_button(self, index):
        return self._buttons[index]


def test_gamepad_input_maps_sticks_to_drive_axes():
    # SDL: left stick Y is inverted (up = negative); GamepadInput flips it.
    # Rotation reads the right stick's X axis (index 2, standard Windows
    # XInput layout) -- left/right, not up/down.
    joystick = FakeJoystick(axes=[0.6, -0.6, 0.4, 0.0], buttons=[0, 0])
    src = GamepadInput(joystick=joystick)
    assert src.available
    drive, _ = src.poll()
    assert drive.vx == 0.6
    assert drive.vy == 0.6
    assert drive.omega == -0.4


def test_gamepad_input_rotation_ignores_right_stick_y():
    # Right stick pushed straight up/down (axis 3) should not rotate the
    # robot -- only axis 2 (right stick X) does.
    joystick = FakeJoystick(axes=[0.0, 0.0, 0.0, 0.9], buttons=[0, 0])
    src = GamepadInput(joystick=joystick)
    drive, _ = src.poll()
    assert drive.omega == 0.0


def test_gamepad_input_applies_deadband():
    joystick = FakeJoystick(axes=[0.05, -0.05, 0.05, 0.0], buttons=[0, 0])
    src = GamepadInput(joystick=joystick)
    drive, _ = src.poll()
    assert drive.vx == 0.0
    assert drive.vy == 0.0
    assert drive.omega == 0.0


def test_gamepad_input_start_button_pause_toggle_is_edge_triggered():
    buttons = [0] * 8
    joystick = FakeJoystick(axes=[0, 0, 0, 0], buttons=buttons)
    src = GamepadInput(joystick=joystick)

    _, operator = src.poll()
    assert operator.pause_toggle is False

    buttons[7] = 1  # Start pressed
    _, operator = src.poll()
    assert operator.pause_toggle is True

    _, operator = src.poll()  # still held down -- no repeat toggle
    assert operator.pause_toggle is False

    buttons[7] = 0
    src.poll()
    buttons[7] = 1  # pressed again
    _, operator = src.poll()
    assert operator.pause_toggle is True


def test_gamepad_input_x_button_cycle_level_is_edge_triggered():
    buttons = [0] * 8
    joystick = FakeJoystick(axes=[0, 0, 0, 0], buttons=buttons)
    src = GamepadInput(joystick=joystick)

    _, operator = src.poll()
    assert operator.cycle_level is False

    buttons[2] = 1  # X pressed
    _, operator = src.poll()
    assert operator.cycle_level is True

    _, operator = src.poll()  # still held down -- no repeat trigger
    assert operator.cycle_level is False

    buttons[2] = 0
    src.poll()
    buttons[2] = 1  # pressed again
    _, operator = src.poll()
    assert operator.cycle_level is True


def test_gamepad_input_a_button_maps_to_intake():
    joystick = FakeJoystick(axes=[0, 0, 0, 0], buttons=[1, 0])
    src = GamepadInput(joystick=joystick, deposit_action="l2")
    _, operator = src.poll()
    assert operator.intake_active is True
    assert operator.deposit_active is False
    assert operator.deposit_action == "l2"


def test_gamepad_input_right_trigger_maps_to_deposit():
    # Axis 5 (RT), 0.0 released to 1.0 fully pressed -- below the
    # half-press threshold should not register as deposit.
    joystick = FakeJoystick(axes=[0, 0, 0, 0, 0, 0.3], buttons=[0, 0])
    src = GamepadInput(joystick=joystick, deposit_action="l2")
    _, operator = src.poll()
    assert operator.deposit_active is False

    joystick = FakeJoystick(axes=[0, 0, 0, 0, 0, 0.8], buttons=[0, 0])
    src = GamepadInput(joystick=joystick, deposit_action="l2")
    _, operator = src.poll()
    assert operator.deposit_active is True
    assert operator.deposit_action == "l2"


def test_gamepad_input_unavailable_when_no_device_and_no_injected_joystick():
    # This environment/CI box has no physical controller attached, so
    # auto-detection should degrade gracefully rather than raising.
    src = GamepadInput(index=99)
    assert src.available is False
    drive, operator = src.poll()
    assert drive == DriveCommand()
    assert operator == OperatorCommand()

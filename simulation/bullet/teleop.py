"""
Interactive PyBullet teleop for the Hexapod.

Drives the same HexapodController / kinematics / gait code that runs on the
real robot, but with a PyBullet interface instead of the UART link. Because
there is no hardware here, there are no serial round-trips and no servo lag.

Input
-----
A game controller (PS3/Xbox-style) read through pygame is the primary input.
If no controller is found (or --input keyboard is given) the script falls back
to PyBullet's own keyboard events, so it stays usable without a pad.

    Joystick (default mapping, SDL/pygame indices — see notes below)
        left stick  Y         forward / backward   (vx)
        left stick  X         strafe left / right  (vy)
        right stick X         turn (yaw)           (wz)
        right stick Y         body pitch (look up/down)
        R1 (deadman)          hold to enable motion; release to stop
        D-pad up / down       body height up / down
        button A              cycle gait (tripod -> wave -> ripple)
        button B              shutdown sequence
        button Y              setup / restart (after shutdown)

    Keyboard (fallback)
        W / S                 forward / backward
        A / D                 strafe left / right
        Q / E                 turn left / right
        R / F                 body height up / down
        T / G                 body pitch up / down
        1 / 2 / 3             select tripod / wave / ripple
        X                     shutdown
        Z                     setup / restart

Note on controller indices
--------------------------
SDL/pygame axis & button indices are NOT the same as the ROS `joy` node
indices used in hexapod_gazebo/joy_teleop_node.py, they depend on the pad and
the OS. Run with `--calibrate` to print live axis/button values and read off
the right indices for your controller, then adjust the constants below.
"""

import os
import argparse
import time
import yaml

# PyBullet
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

from simulation.bridge import get_hardware_path, get_controller_path
from simulation.bullet.interface import PyBulletInterface
from controller import HexapodController


# Controller mapping (SDL / pygame indices, adjust with --calibrate)

# Axes (Xbox-style defaults; many PS3 pads use right-stick X/Y = 2/3)
AXIS_LINEAR_X  = 1   # left stick vertical   -> forward/back
AXIS_LINEAR_Y  = 0   # left stick horizontal -> strafe
AXIS_ANGULAR_Z = 3   # right stick horizontal-> yaw
AXIS_PITCH     = 4   # right stick vertical  -> body pitch

# Sign so that "push up = forward", "push right = right/clockwise", etc.
# pygame reports up/left as negative on most pads.
SIGN_LINEAR_X  = -1.0
SIGN_LINEAR_Y  = +1.0
SIGN_ANGULAR_Z = -1.0
SIGN_PITCH     = -1.0

# Buttons
BUTTON_DEADMAN  = 5   # R1: hold to allow motion
BUTTON_GAIT     = 0   # cycle gait
BUTTON_SHUTDOWN = 1   # shutdown sequence
BUTTON_SETUP    = 3   # setup / restart

AXIS_DEADZONE = 0.12  # ignore small stick noise around center

# Body pose increments
HEIGHT_STEP = 5.0     # mm per D-pad / key press
GAITS = ['tripod', 'wave', 'ripple']


def _deadzone(value: float, threshold: float = AXIS_DEADZONE) -> float:
    """Zero out small stick deflection and rescale the rest to [-1, 1]."""
    if abs(value) < threshold:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)


class JoystickInput:
    """Reads a game controller through pygame and produces command values."""

    def __init__(self, require_deadman: bool = True):
        import pygame  # local import so the script runs without pygame installed
        self.pygame = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected")
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.require_deadman = require_deadman
        self._prev_buttons = [0] * self.joy.get_numbuttons()
        self._prev_hat_y = 0
        print(f"Joystick: {self.joy.get_name()} "
              f"({self.joy.get_numaxes()} axes, {self.joy.get_numbuttons()} buttons)")

    def _axis(self, index: int) -> float:
        if index >= self.joy.get_numaxes():
            return 0.0
        return _deadzone(self.joy.get_axis(index))

    def _button(self, index: int) -> int:
        if index >= self.joy.get_numbuttons():
            return 0
        return self.joy.get_button(index)

    def read(self) -> dict:
        """Return the current command snapshot (see main loop for the schema)."""
        self.pygame.event.pump()

        enabled = (not self.require_deadman) or bool(self._button(BUTTON_DEADMAN))

        # Edge-triggered actions
        buttons = [self._button(i) for i in range(self.joy.get_numbuttons())]
        def pressed(idx):
            return idx < len(buttons) and buttons[idx] and not self._prev_buttons[idx]

        cycle_gait = pressed(BUTTON_GAIT)
        shutdown = pressed(BUTTON_SHUTDOWN)
        setup = pressed(BUTTON_SETUP)

        # D-pad (hat) for height, edge-triggered on the vertical axis
        height_delta = 0.0
        if self.joy.get_numhats() > 0:
            hat_x, hat_y = self.joy.get_hat(0)
            if hat_y != self._prev_hat_y and hat_y != 0:
                height_delta = HEIGHT_STEP * hat_y
            self._prev_hat_y = hat_y

        self._prev_buttons = buttons

        return {
            'enabled': enabled,
            'vx_norm': SIGN_LINEAR_X * self._axis(AXIS_LINEAR_X),
            'vy_norm': SIGN_LINEAR_Y * self._axis(AXIS_LINEAR_Y),
            'wz_norm': SIGN_ANGULAR_Z * self._axis(AXIS_ANGULAR_Z),
            'pitch_norm': SIGN_PITCH * self._axis(AXIS_PITCH),
            'height_delta': height_delta,
            'cycle_gait': cycle_gait,
            'shutdown': shutdown,
            'setup': setup,
        }


class KeyboardInput:
    """Reads PyBullet keyboard events (no extra dependency, needs the GUI window)."""

    def __init__(self, physics):
        self.physics = physics

    def read(self) -> dict:
        keys = self.physics.getKeyboardEvents()
        down = self.physics.KEY_IS_DOWN
        triggered = self.physics.KEY_WAS_TRIGGERED

        def is_down(ch):
            return ord(ch) in keys and keys[ord(ch)] & down

        def was_pressed(ch):
            return ord(ch) in keys and keys[ord(ch)] & triggered

        vx = (1.0 if is_down('w') else 0.0) - (1.0 if is_down('s') else 0.0)
        vy = (1.0 if is_down('d') else 0.0) - (1.0 if is_down('a') else 0.0)
        wz = (1.0 if is_down('q') else 0.0) - (1.0 if is_down('e') else 0.0)
        pitch = (1.0 if is_down('t') else 0.0) - (1.0 if is_down('g') else 0.0)

        height_delta = 0.0
        if was_pressed('r'):
            height_delta = HEIGHT_STEP
        elif was_pressed('f'):
            height_delta = -HEIGHT_STEP

        cycle_gait = was_pressed('1') or was_pressed('2') or was_pressed('3')
        gait_override = None
        if was_pressed('1'):
            gait_override = 'tripod'
        elif was_pressed('2'):
            gait_override = 'wave'
        elif was_pressed('3'):
            gait_override = 'ripple'

        return {
            'enabled': True,
            'vx_norm': vx,
            'vy_norm': vy,
            'wz_norm': wz,
            'pitch_norm': pitch,
            'height_delta': height_delta,
            'cycle_gait': cycle_gait,
            'gait_override': gait_override,
            'shutdown': was_pressed('x'),
            'setup': was_pressed('z'),
        }


def run_calibrate():
    """Print live axis/button/hat values to help map a specific controller."""
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Calibrating: {joy.get_name()} — move sticks / press buttons. Ctrl+C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            axes = [f"{joy.get_axis(i):+.2f}" for i in range(joy.get_numaxes())]
            buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
            hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
            print(f"\raxes={axes} buttons={buttons} hats={hats}      ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def main():
    parser = argparse.ArgumentParser(description='Interactive PyBullet Hexapod teleop')
    parser.add_argument('--gait', '-g', type=str, default='tripod',
                        choices=GAITS, help='Initial gait pattern')
    parser.add_argument('--input', '-i', type=str, default='auto',
                        choices=['auto', 'joystick', 'keyboard'],
                        help='Input source. auto = joystick if present else keyboard')
    parser.add_argument('--no-deadman', action='store_true',
                        help='Disable the deadman button (motion always enabled on joystick)')
    parser.add_argument('--simulation-rate', '-s', type=float, default=240,
                        help='Physics step rate in Hz. Default 240')
    parser.add_argument('--controller-rate', '-c', type=float, default=40,
                        help='Controller update rate in Hz. Default 40 (matches the robot)')
    parser.add_argument('--width', '-w', type=int, default=1280,
                        help='Simulation window width')
    parser.add_argument('--height', '-e', type=int, default=720,
                        help='Simulation window height')
    parser.add_argument('--verbose', '-t', action='store_true',
                        help='Log controller status messages')
    parser.add_argument('--calibrate', action='store_true',
                        help='Print live joystick axis/button values and exit (no sim)')
    args = parser.parse_args()

    if args.calibrate:
        run_calibrate()
        return

    # PyBullet setup (mirrors bullet/main.py)
    physics = bc.BulletClient(connection_mode=p.GUI,
                              options=f"--width={args.width} --height={args.height}")
    physics.configureDebugVisualizer(physics.COV_ENABLE_GUI, 0)
    physics.configureDebugVisualizer(physics.COV_ENABLE_SHADOWS, 0)
    physics.resetDebugVisualizerCamera(
        cameraDistance=0.6, cameraYaw=180, cameraPitch=-35,
        cameraTargetPosition=[0, 0, 0])

    physics.setAdditionalSearchPath(pybullet_data.getDataPath())
    physics.setGravity(0, 0, -9.81)
    physics.loadURDF("plane.urdf")

    urdf_file = get_hardware_path("hexapod.urdf")
    robot_id = physics.loadURDF(str(urdf_file), [0, 0, 0.2],
                                physics.getQuaternionFromEuler([0, 0, 0]),
                                flags=physics.URDF_USE_INERTIA_FROM_FILE)

    # Load configuration
    config_path = get_controller_path("controller", "config", "config.yml")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    lin_vel_max = config['safety'].get('lin_vel_max', 250.0)   # mm/s
    ang_vel_max = config['safety'].get('ang_vel_max', 30.0)    # deg/s
    pitch_range = config['safety'].get('pitch_range', (-10, 10))
    pitch_max = float(pitch_range[1])

    # Create the controller (same code as the real robot, PyBullet interface)
    interface = PyBulletInterface(config, robot_id, physics)
    controller = HexapodController(interface, config, verbose=args.verbose)
    controller.set_gait(args.gait)
    gait_index = GAITS.index(args.gait)

    # Select input source
    source = None
    if args.input in ('auto', 'joystick'):
        try:
            source = JoystickInput(require_deadman=not args.no_deadman)
        except Exception as exc:
            if args.input == 'joystick':
                raise
            print(f"No joystick ({exc}); falling back to keyboard. "
                  f"Focus the PyBullet window and use WASD/QE.")
    if source is None:
        source = KeyboardInput(physics)

    print(f"Teleop ready — gait={GAITS[gait_index]}, "
          f"controller {args.controller_rate} Hz, physics {args.simulation_rate} Hz")

    controller_dt = 1.0 / args.controller_rate
    simulation_dt = 1.0 / args.simulation_rate
    height_offset = 0.0
    hud_id = -1
    hud_accumulator = 0.0
    last_frame = time.perf_counter()

    try:
        while physics.isConnected():
            now = time.perf_counter()
            actual_dt = now - last_frame
            last_frame = now

            cmd = source.read()

            # Lifecycle
            if cmd.get('shutdown'):
                controller.shutdown()
            if cmd.get('setup'):
                controller.setup()

            # Gait selection
            if cmd.get('gait_override'):
                gait_index = GAITS.index(cmd['gait_override'])
                controller.set_gait(GAITS[gait_index])
            elif cmd.get('cycle_gait'):
                gait_index = (gait_index + 1) % len(GAITS)
                controller.set_gait(GAITS[gait_index])

            # Velocity (held convention; zeroed when deadman released)
            if cmd['enabled']:
                controller.set_linear_velocity(cmd['vx_norm'] * lin_vel_max,
                                               cmd['vy_norm'] * lin_vel_max, 0.0)
                controller.set_angular_velocity(cmd['wz_norm'] * ang_vel_max)
            else:
                controller.set_linear_velocity(0.0, 0.0, 0.0)
                controller.set_angular_velocity(0.0)

            # Body pose
            if cmd['height_delta']:
                height_offset += cmd['height_delta']
            controller.set_body_orientation(0.0, cmd['pitch_norm'] * pitch_max, 0.0)
            controller.set_body_position(0.0, 0.0, height_offset)

            # Step the controller (same call the robot makes)
            controller.update(actual_dt)

            # Step physics at its own rate
            for _ in range(max(1, round(actual_dt / simulation_dt))):
                physics.stepSimulation()

            # Lightweight HUD (~5 Hz)
            hud_accumulator += actual_dt
            if hud_accumulator >= 0.2:
                hud_accumulator = 0.0
                status = (f"state={controller.state.name}  gait={GAITS[gait_index]}  "
                          f"vx={controller.linear_velocity[0]:+.0f} "
                          f"vy={controller.linear_velocity[1]:+.0f} mm/s  "
                          f"wz={controller.angular_velocity:+.0f} deg/s  "
                          f"h={height_offset:+.0f} mm")
                hud_id = physics.addUserDebugText(
                    status, [-0.25, 0, 0.25], textColorRGB=[1, 1, 1], textSize=1.1,
                    replaceItemUniqueId=hud_id if hud_id >= 0 else -1)

            # Real-time pacing
            elapsed = time.perf_counter() - now
            if elapsed < controller_dt:
                time.sleep(controller_dt - elapsed)

    except KeyboardInterrupt:
        controller.emergency_stop()
    finally:
        time.sleep(0.1)
        if physics.isConnected():
            physics.disconnect()


if __name__ == '__main__':
    main()

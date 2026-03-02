"""
Quick Viser-based Hexapod demo.
"""

import yaml
import time
import argparse

# Viser
import viser
import yourdfpy
from viser.extras import ViserUrdf

from simulation.bridge import get_controller_path, get_hardware_path
from interface import ViserInterface
from controller import HexapodController


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Visualize Hexapod demo on Viser')
    parser.add_argument('--gait', '-g', type=str, default='tripod',
                        choices=['tripod', 'wave', 'ripple'],
                        help='Gait pattern to visualize')
    parser.add_argument('--vx', '-x', type=float, default=50.0,
                        help='Forward velocity (mm/s)')
    parser.add_argument('--vy', '-y', type=float, default=0.0,
                        help='Strafe velocity (mm/s)')
    parser.add_argument('--vz', '-z', type=float, default=0.0,
                        help='Upward velocity (mm/s)')
    parser.add_argument('--yaw', '-v', type=float, default=0.0,
                        help="Yaw velocity (deg/s)")
    parser.add_argument('--simulation-rate', '-s', type=float, default=80,
                        help="Simulation update rate in Hz. Default is 80 Hz")
    parser.add_argument('--controller-rate', '-c', type=float, default=40,
                        help="Controller update rate in Hz. Default is 50 Hz")
    parser.add_argument('--port', '-p', type=int, default=8080,
                        help='Viser server port')

    args = parser.parse_args()

    # Viser setup
    server = viser.ViserServer(port=args.port)

    # Load configuration
    config_path = get_controller_path("controller", "config", "config.yml")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Robot frame
    root_frame = "/robot"
    urdf_path = get_hardware_path("hexapod.urdf")
    mesh_path = get_hardware_path("CAD")
    server.scene.add_frame(root_frame, show_axes=False)
    robot = yourdfpy.URDF.load(
        urdf_path,
        mesh_dir=mesh_path,
    )
    urdf = ViserUrdf(
        server,
        robot,
        root_node_name=root_frame,
    )

    # Determine actuated joints (order matters)
    joint_names = urdf.get_actuated_joint_names()
    print("Actuated joints from URDF:")
    for i, name in enumerate(joint_names):
        print(f"  {i}: {name}")

    # Create the controller
    interface = ViserInterface(config, urdf)  # Viser demo interface
    controller = HexapodController(interface, config)

    # Simulation loop
    try:

        # Time tracking
        simulation_dt = 1. / args.simulation_rate
        controller_dt = 1. / args.controller_rate
        controller_time_accumulator = 0.0
        t = 0.0

        # Examples
        t0 = 5.0
        t1 = 10.0
        t2 = 15.0
        t3 = 20.0
        t4 = 25.0
        t5 = 30.0

        while True:

            # Accumulate time
            controller_time_accumulator += simulation_dt

            # Only update controller when enough time has passed
            if controller_time_accumulator >= controller_dt:
                outcome = controller.update(controller_dt)
                controller_time_accumulator -= controller_dt  # Keep remainder for accuracy

            # Example: at t0, we start the gait
            if t0 and t >= t0:
                t0 = None  # invalidate so that we won't change it again
                controller.set_linear_velocity(args.vx, args.vy, args.vz)
                # controller.set_angular_velocity(args.yaw)

            # Example: at t1, we change body position (lower body to 50 mm)
            if t1 and t >= t1:
                t1 = None
                controller.set_body_position(0, 0, 50)

            # Example: at t2, we change body orientation
            if t2 and t >= t2:
                t2 = None
                controller.set_body_orientation(5, 5, 5)

            # Example: at t3, we speed up
            if t3 and t >= t3:
                t3 = None
                controller.set_linear_velocity(args.vx * 10, args.vy * 10, args.vz * 10)

            # Example: at t4, we stop
            if t4 and t >= t4:
                t4 = None
                controller.set_linear_velocity(0, 0, 0)

            if t5 and t >= t5:
                break

            time.sleep(simulation_dt)
            t += simulation_dt

    except KeyboardInterrupt:

        print("Visualization stopped")

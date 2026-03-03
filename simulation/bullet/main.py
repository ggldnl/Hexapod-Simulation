"""
Quick PyBullet-based simulation of the Hexapod robot.
"""

from pathlib import Path
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


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='PyBullet Hexapod simulation')
    parser.add_argument('--gait', '-g', type=str, default='tripod',
                        choices=['tripod', 'wave', 'ripple'],
                        help='Gait pattern to visualize')
    parser.add_argument('--vx', '-x', type=float, default=50.0,
                        help='Forward velocity (mm/s)')
    parser.add_argument('--vy', '-y', type=float, default=0.0,
                        help='Strafe velocity (mm/s)')
    parser.add_argument('--vz', '-z', type=float, default=0.0,
                        help='Upward velocity (mm/s)')
    parser.add_argument('--yaw', '-v', type=float, default=10.0,
                        help="Yaw velocity (deg/s)")
    parser.add_argument('--simulation-rate', '-s', type=float, default=80,
                        help="Simulation update rate in Hz. Default is 80 Hz")
    parser.add_argument('--controller-rate', '-c', type=float, default=20,
                        help="Controller update rate in Hz. Default is 20 Hz")
    parser.add_argument('--width', '-w', type=int, default=1980,
                        help="Width of the simulation window. Default is 1980")
    parser.add_argument('--height', '-e', type=int, default=1080,
                        help="Height of the simulation window. Default is 1080")
    parser.add_argument('--video_path', '-p', type=str, default=None,
                        help="If provided, the script will save an mp4 of the simulation on the path")

    args = parser.parse_args()

    # PyBullet setup
    physics = bc.BulletClient(connection_mode=p.GUI, options=f"--width={args.width} --height={args.height}")
    physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  # Disable shadows

    # Set initial camera view
    default_camera_distance = 0.5
    default_camera_roll = 0
    default_camera_yaw = 180
    default_camera_pitch = -45
    default_camera_target = [0, 0, 0]

    physics.resetDebugVisualizerCamera(
        cameraDistance=default_camera_distance,
        cameraYaw=default_camera_yaw,
        cameraPitch=default_camera_pitch,
        cameraTargetPosition=default_camera_target
    )

    # Load additional data and set gravity
    physics.setAdditionalSearchPath(pybullet_data.getDataPath())
    physics.setGravity(0, 0, -9.81)

    # Load plane and robot URDF files
    planeID = physics.loadURDF("plane.urdf")

    urdf_file = get_hardware_path("hexapod.urdf")
    pos = [0, 0, 0.2]
    orn = p.getQuaternionFromEuler([0, 0, 0])
    robotID = physics.loadURDF(str(urdf_file), pos, orn, flags=p.URDF_USE_INERTIA_FROM_FILE)

    # Compute correction so bottom sits on plane (z=0)
    aabb_min, aabb_max = p.getAABB(robotID)
    bottom_z = aabb_min[2]
    z_correction = -bottom_z + 0.1
    pos, orn = p.getBasePositionAndOrientation(robotID)
    p.resetBasePositionAndOrientation(robotID, [pos[0], pos[1], pos[2] + z_correction], orn)

    # Adjust friction
    # p.changeDynamics(robotID, -1, lateralFriction=5.0, spinningFriction=0.1, rollingFriction=0.1)
    # p.changeDynamics(planeID, -1, lateralFriction=5.0)

    # List the revolute joints
    revolute_joints = []
    num_joints = physics.getNumJoints(robotID)
    for joint_index in range(num_joints):
        joint_info = physics.getJointInfo(robotID, joint_index)
        joint_type = joint_info[2]  # Joint type is the third element in joint_info tuple
        if joint_type == physics.JOINT_REVOLUTE:
            revolute_joints.append(joint_index)
            print(f"Joint {joint_index} is revolute: {joint_info[1].decode('utf-8')}")

    # Load configuration
    config_path = get_controller_path("controller", "config", "config.yml")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # We know the joints are well formatted (always coxa, femur and tibia for each leg, for leg from 1 to 6)
    leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]
    joint_mapping = {
        leg_name: [
            revolute_joints[0 + i * 3],
            revolute_joints[1 + i * 3],
            revolute_joints[2 + i * 3]
        ] for i, leg_name in enumerate(leg_names)
    }

    # Create the controller
    interface = PyBulletInterface(config, robotID, joint_mapping)  # PyBullet simulation interface
    controller = HexapodController(interface, config)

    # Video
    if args.video_path:
        Path(args.video_path).parent.mkdir(exist_ok=True, parents=True)
        physics.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.video_path)

    # Simulation loop
    try:

        # Time tracking
        simulation_dt = 1. / args.simulation_rate
        controller_dt = 1. / args.controller_rate
        controller_time_accumulator = 0.0
        t = 0.0

        # Examples
        t0 = 5.0    # look in one direction
        t1 = 6.0    # look another direction
        t2 = 7.0    # reset body orientation
        t3 = 8.0    # set linear velocity
        t4 = 12.0   # set angular velocity
        t5 = 35.0   # stop
        t6 = 37.0   # shutdown

        while p.isConnected():

            # Accumulate time
            controller_time_accumulator += simulation_dt

            # Only update controller when enough time has passed
            if controller_time_accumulator >= controller_dt:
                outcome = controller.update(controller_dt)
                controller_time_accumulator -= controller_dt  # Keep remainder for accuracy

            # Example: at t0, look in one direction (change body orientation)
            if t0 and t >= t0:
                t0 = None  # invalidate so that we won't change it again
                controller.set_body_orientation(5, -5, 5)

            # Example: at t1, look to another direction (change again body orientation)
            if t1 and t >= t1:
                t1 = None
                controller.set_body_orientation(-5, -5, -5)

            # Example: at t2, reset body orientation
            if t2 and t >= t2:
                t2 = None
                controller.set_body_orientation(0, 0, 0)

            # Example: at t3, start walking
            if t3 and t >= t3:
                t3 = None
                controller.set_linear_velocity(args.vx, args.vy, args.vz)

            # Example: at t4, change angular velocity
            if t4 and t >= t4:
                t4 = None
                controller.set_angular_velocity(args.yaw)

            # Example: at t5, stop walking
            if t5 and t >= t5:
                t5 = None
                controller.set_linear_velocity(0, 0, 0)
                controller.set_angular_velocity(0)

            if t6 and t >= t6:
                t6 = None
                controller.shutdown()

            # Step the simulation at its own rate
            physics.stepSimulation()

            time.sleep(simulation_dt)
            t += simulation_dt

    finally:

        # Stop the recording
        if args.video_path:
            physics.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)

        # Disconnect from the simulations when done
        physics.disconnect()

import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

from pathlib import Path
import numpy as np
import argparse
import time

from sinusoidal_signals.generator import SinusoidalSignalGaitGenerator
from sinusoidal_signals.gaits import Gaits


def map_signals(signals, min_angles, max_angles):
    return ((signals + 1) / 2) * (max_angles - min_angles) + min_angles


if __name__ == '__main__':
    """
    Showcase sinusoidal open-loop gait patterns.
    - Open-loop: do not take into account external factors such as ground contact.
    - Sinusoidal: use sinusoidal signals to control each joint during the gait. Each signal
        is defined by amplitude, phase, duty_cycle and vertical_offset. 
    The code is based on the paper "Robots that can adapt like animals" by Cully et al., Nature, 2015
    but adapted to our use case (e.g. introduced vertical offset of the signals, tweaked gait parameters, ...).
    """

    parser = argparse.ArgumentParser(description="Showing predefined open-loop gait patterns.")
    parser.add_argument("-g", "--gait", type=str, default='TRI_GAIT', help="Gait")
    parser.add_argument("-u", "--URDF", type=str, default='Hexapod-Hardware/hexapod.urdf', help="Path to the robot's URDF")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-d', '--dt', type=float, default=0.02, help="Time delta for update (default=0.02=50Hz)")
    parser.add_argument('-v', '--video_path', type=str, default=None, help="If provided, the script will save an mp4 of the simulation on the path")

    args = parser.parse_args()

    # ------------------------------ Gait generator ------------------------------ #

    selected_gait = Gaits[args.gait]
    print(f'Selected gait: {selected_gait.label}')
    open_loop_gait_generator = SinusoidalSignalGaitGenerator(selected_gait.data)

    # ------------------------------- Joint limits ------------------------------- #

    # Out of simplicity, we can say that all the angles should be in range -np.pi/2, np.pi/2
    min_angles = np.full((6, 3), -np.pi/2)
    max_angles = np.full((6, 3), np.pi/2)

    # --------------------------------- PyBullet --------------------------------- #

    # Connect to physics server
    # physics = p.connect(p.GUI)

    physics = bc.BulletClient(connection_mode=p.GUI, options="--width=1980 --height=1080")
    physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  # Disable shadows

    # Set initial camera view
    default_camera_distance = 0.5
    default_camera_roll = 0
    default_camera_yaw = -45
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
    planeId = physics.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 0]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    repo_dir = Path(__file__).parent.parent
    # urdf_file = Path(repo_dir, '../data/URDF/hexapod.urdf')
    urdf_file = Path(repo_dir, args.URDF)
    robotID = physics.loadURDF(str(urdf_file), cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

    # List the revolute joints
    revolute_joints = []
    num_joints = physics.getNumJoints(robotID)
    for joint_index in range(num_joints):
        joint_info = physics.getJointInfo(robotID, joint_index)
        joint_type = joint_info[2]  # Joint type is the third element in joint_info tuple
        if joint_type == physics.JOINT_REVOLUTE:
            revolute_joints.append(joint_index)
            print(f"Joint {joint_index} is revolute: {joint_info[1].decode('utf-8')}")

    # We know the joints are well formatted (always coxa, femur and tibia for each leg, for leg from 1 to 6)
    joint_mapping = {
        i: {
            'coxa_joint': revolute_joints[0 + i * 3],
            'femur_joint': revolute_joints[1 + i * 3],
            'tibia_joint': revolute_joints[2 + i * 3]
        } for i in range(6)
    }

    # ------------------------------ Simulation loop ----------------------------- #

    if args.video_path:

        video_path = Path(args.video_path)
        folder_path = video_path.parent
        if not folder_path.exists():
            folder_path.mkdir(exist_ok=True, parents=True)

        physics.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.video_path)

    try:

        t = 0.0
        # dt = 1. / 240.
        dt = args.dt

        while p.isConnected():

            # Step the simulations
            physics.stepSimulation()

            # Get the joint angles
            signals = open_loop_gait_generator.step(t)
            joint_angles = map_signals(signals, min_angles, max_angles)

            for i in range(6):

                joints = joint_mapping[i]
                angles = joint_angles[i]

                # Move the joints to the target position
                p.setJointMotorControl2(robotID, joints['coxa_joint'], p.POSITION_CONTROL, targetPosition=angles[0])
                p.setJointMotorControl2(robotID, joints['femur_joint'], p.POSITION_CONTROL, targetPosition=angles[1])
                p.setJointMotorControl2(robotID, joints['tibia_joint'], p.POSITION_CONTROL, targetPosition=angles[2])

            time.sleep(dt)
            t += dt

    finally:

        print('Disconnected.')

        # Stop the recording
        if args.video_path:
            p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)

        # Disconnect from the simulations when done
        physics.disconnect()

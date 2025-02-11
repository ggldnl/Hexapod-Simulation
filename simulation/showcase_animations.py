import sys
sys.path.append('../Hexapod-Controller/controller')
print(f'Controller successfully imported.')

import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

from pathlib import Path
import numpy as np
import argparse
import json
import time

from controller import Controller
from hexapod import VirtualHexapod


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="PyBullet simulation.")
    parser.add_argument("-u", "--URDF", type=str, default='Hexapod-Hardware/hexapod.urdf', help="Path to the robot's URDF")
    parser.add_argument("-c", "--config", type=str, default='Hexapod-Controller/config/config.json', help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-d', '--dt', type=float, default=0.02, help="Time delta for update (default=0.02=50Hz)")
    parser.add_argument('-v', '--video_path', type=str, default=None, help="If provided, the script will save an mp4 of the simulation on the path")

    args = parser.parse_args()

    # -------------------------------- Controller -------------------------------- #

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a virtual robot to use in simulation
    hexapod = VirtualHexapod(config[args.name])
    controller = Controller(hexapod)

    # --------------------------------- PyBullet --------------------------------- #

    # Connect to physics server
    # physics = p.connect(p.GUI)

    physics = bc.BulletClient(connection_mode=p.GUI)
    physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  # Disable shadows

    # Set initial camera view
    default_camera_distance = 0.5
    default_camera_roll = 0
    default_camera_yaw = 0
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

        # Define the actions the robot will execute

        # Stand and wait a little
        controller.stand(5)

        # Look around
        controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(10), np.deg2rad(10)]))
        controller.set_body_pose(2, body_orientation=np.array([0, np.deg2rad(-10), np.deg2rad(-10)]))
        controller.set_body_pose(2, body_orientation=np.array([0, 0, 0]))

        # Move a leg along its relative axis
        leg_index = 0
        offset = 50
        x, y, z = 80, 0, 0  # Add an offset to the x to fall in the reachable workspace

        # Move by a certain amount in the leg's x-axis
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x + offset, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)

        # Move by a certain amount in the leg's y-axis
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y + offset, z]]), indices=[leg_index], leg_frame=True)
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index], leg_frame=True)

        # Move by a certain amount in the leg's z-axis by following a straight line
        delta=10
        x, y = 150, 0
        for _ in range(3):
            for i in range(-50, 50, delta):
                controller.set_legs_positions(0.1, np.array([[x, y, i]]), indices=[leg_index], leg_frame=True)
            for i in range(50, -50, -delta):
                controller.set_legs_positions(0.1, np.array([[x, y, i]]), indices=[leg_index], leg_frame=True)

        # Move the same leg on points expressed in the origin frame, drawing a square
        x, y, z = 150, 150, 100
        offset = 50

        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x + offset, y, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x + offset, y + offset, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x, y + offset, z]]), indices=[leg_index])
        controller.set_legs_positions(2, np.array([[x, y, z]]), indices=[leg_index])  # Go back to the starting point

        # Sit and terminate the test script
        controller.sit(5)

        while p.isConnected():

            # Step the simulations
            physics.stepSimulation()

            # Get the joint angles
            joint_angles = controller.step(dt)

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

        # Disconnect from the simulations when done
        physics.disconnect()

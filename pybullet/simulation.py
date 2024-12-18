import numpy as np

import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
from pathlib import Path
import time

from hexapod_controller import HexapodController


if __name__ == '__main__':

    # --------------------------------- PyBullet --------------------------------- #

    # Connect to physics server
    # physics = p.connect(p.GUI)

    physics = bc.BulletClient(connection_mode=p.GUI)
    physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    # Set initial camera view
    default_camera_distance = 1
    default_camera_yaw = -45
    default_camera_pitch = -45
    default_camera_target = [0, 0, 0]

    physics.resetDebugVisualizerCamera(cameraDistance=default_camera_distance,
                                    cameraYaw=default_camera_yaw,
                                    cameraPitch=default_camera_pitch,
                                    cameraTargetPosition=default_camera_target)

    # Load additional data and set gravity
    physics.setAdditionalSearchPath(pybullet_data.getDataPath())
    physics.setGravity(0, 0, -9.81)

    # Load plane and robot URDF files
    planeId = physics.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 0]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    repo_dir = Path(__file__).parent.parent
    urdf_file = Path(repo_dir, 'URDF/hexapod.urdf')
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

    ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75,
            0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5]
    controller = HexapodController(ctrl)

    try:

        t = 0.0
        dt = 1. / 240.

        while True:

            # Step the simulation
            physics.stepSimulation()

            # Precomputes trajectories for all the servos (coxa, femur, tibia)
            # given the parameters
            joint_angles = controller.step(t)

            # Reshape to 6x3
            joint_angles = [
                [
                    joint_angles[0 + i * 3],
                    joint_angles[1 + i * 3],
                    joint_angles[2 + i * 3],
                ] for i in range(6)
            ]

            for i in range(6):

                joints = joint_mapping[i]
                angles = joint_angles[i]

                # Apply offsets
                offset = np.deg2rad(25)
                angles[1] -= offset
                angles[2] += offset

                angles[1] += np.pi / 2

                # Mirror
                if i > 2:
                    angles[1] *= -1

                # Move the joints to the target position
                p.setJointMotorControl2(robotID, joints['coxa_joint'], p.POSITION_CONTROL, targetPosition=angles[0])
                p.setJointMotorControl2(robotID, joints['femur_joint'], p.POSITION_CONTROL, targetPosition=angles[1])
                p.setJointMotorControl2(robotID, joints['tibia_joint'], p.POSITION_CONTROL, targetPosition=angles[2])

            time.sleep(dt)

            t += dt

    finally:

        # Disconnect from the simulation when done
        physics.disconnect()

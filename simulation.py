import pybullet as p
import pybullet_data
import time
import math


class Leg:

    def __init__(self, config, h=60):
        self.config = config
        self.h = h

    @staticmethod
    def _distance(position):
        """
        Euclidean distance from the leg's origin to the provided point.
        """
        return math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)

    def fk(self, angles):
        """
        Computes the forward kinematics (position of the end of the leg)
        given the joint angles.
        """

        coxa_angle, femur_angle, tibia_angle = angles

        # Convert angles from degrees to radians
        coxa_angle = math.radians(coxa_angle)
        femur_angle = math.radians(femur_angle)
        tibia_angle = math.radians(tibia_angle)

        f = self.config['femur']['length']
        t = self.config['tibia']['length']
        h = self.h

        # Position at the end of the first link (after coxa)
        x_coxa = math.cos(coxa_angle) * f
        y_coxa = math.sin(coxa_angle) * f

        # Position at the end of the second link (after femur)
        femur_total_angle = femur_angle
        z_femur = h + math.sin(femur_total_angle) * f
        x_femur = x_coxa + math.cos(femur_total_angle) * f

        # Position at the end of the third link (after tibia)
        tibia_total_angle = femur_total_angle - tibia_angle
        z_tibia = z_femur + math.sin(tibia_total_angle) * t
        x_tibia = x_femur + math.cos(tibia_total_angle) * t

        return x_tibia, y_coxa, z_tibia

    def ik(self, position):
        """
        Computes the inverse kinematics solution to reach the point.
        """

        x, y, z = position

        f = self.config['femur']['length']
        t = self.config['tibia']['length']
        h = self.h

        reachable = self._distance(position) <= (f + t)

        if not reachable:
            return 0, 0, 0

        alpha = math.atan2(y, x)

        d = math.sqrt(y ** 2 + (h - z) ** 2)
        b1 = math.asin(y / d)

        gamma = math.acos((t ** 2 + f ** 2 - d ** 2) / (2 * f * t))

        b2 = math.acos((f ** 2 + d ** 2 - t ** 2) / (2 * f * d))
        beta = b1 + b2

        return alpha, beta, gamma


if __name__ == '__main__':

    config = {
        "coxa": {
            "pin": 0,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_range": -90,
            "max_range": 90,
            "length": 42
        },
        "femur": {
            "pin": 1,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_range": -45,
            "max_range": 90,
            "length": 67.194
        },
        "tibia": {
            "pin": 2,
            "min_pulse": 500,
            "max_pulse": 2500,
            "min_range": -90,
            "max_range": 45,
            "length": 89.206
        }
    }

    # Create a Leg object that will help us compute joint values for a given target position
    leg = Leg(config)
    target = (50, 50, 100)
    coxa_angle, femur_angle, tibia_angle = leg.ik(target)

    # Map the angles to the correct ranges
    # TODO
    coxa_angle -= math.radians(45)
    femur_angle -= math.radians(90)

    # Connect to physics server
    physicsClient = p.connect(p.GUI)

    # Load additional data and set gravity
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Load plane and robot URDF files
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 0]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotID = p.loadURDF("hexapod.urdf", cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

    # List the revolute joints
    revolute_joints = []
    num_joints = p.getNumJoints(robotID)
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(robotID, joint_index)
        joint_type = joint_info[2]  # Joint type is the third element in joint_info tuple
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints.append(joint_index)
            print(f"Joint {joint_index} is revolute: {joint_info[1].decode('utf-8')}")

    # Take the coxa, femur and tibia joint indexes
    leg_index = 0
    coxa_joint = revolute_joints[0 + leg_index * 3]
    femur_joint = revolute_joints[1 + leg_index * 3]
    tibia_joint = revolute_joints[2 + leg_index * 3]

    # Add a static dot at the target position
    sphere_radius = 0.02
    sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1])  # Red dot
    p.createMultiBody(baseVisualShapeIndex=sphere_visual, basePosition=target)

    try:
        for i in range(10000):

            # Step the simulation
            p.stepSimulation()

            coxa_angle = 0
            femur_angle = 0
            tibia_angle = math.radians(90)

            # Move the joints to the target position
            # p.setJointMotorControl2(robotID, coxa_joint, p.POSITION_CONTROL, targetPosition=coxa_angle)
            p.setJointMotorControl2(robotID, femur_joint, p.POSITION_CONTROL, targetPosition=femur_angle)
            p.setJointMotorControl2(robotID, tibia_joint, p.POSITION_CONTROL, targetPosition=tibia_angle)

            time.sleep(1. / 240.)

    finally:
        # Disconnect from the simulation when done
        p.disconnect()
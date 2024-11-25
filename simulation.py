from pathlib import Path
import pybullet as p
import pybullet_data
import time
import math
import json


class Leg:

    def __init__(self, config, invert=False):
        self.config = config
        self.invert = invert

    def inverse_kinematics(self, target):
        """
        Computes the inverse kinematics solution to reach the target point.
        The solution is computed on the kinematic skeleton: it must be converted
        accounting for the actual servo configuration.
        """

        x, y, z = target

        c = self.config['coxa']['length']
        f = self.config['femur']['length']
        t = self.config['tibia']['length']

        alpha = math.atan2(y, x)

        r = math.sqrt(x ** 2 + y ** 2) - c
        d = math.sqrt(r ** 2 + z ** 2)

        if d < abs(f - t):
            raise ValueError(f"Target point {target} is too close to the leg's base.")

        if d > f + t:
            raise ValueError(f"Target point {target} is out of reach.")

        b1 = math.atan2(z, r)
        b2 = math.acos((f ** 2 + d ** 2 - t ** 2) / (2 * f * d))
        beta = b1 + b2

        gamma = math.acos((f ** 2 + t ** 2 - d ** 2) / (2 * f * t)) - math.pi

        return alpha, beta, gamma

    @classmethod
    def _map_range(cls, value, in_min, in_max, out_min, out_max):
        """
        Map the value from the input range to the output range.
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def translate(self, angles):
        """
        Translates the angles into servo ranges.
        """

        alpha, beta, gamma = angles

        femur_offset = math.radians(25)
        tibia_offset = math.radians(7.7)

        alpha = self._map_range(alpha, 0, math.pi, -math.pi / 2, math.pi / 2)
        beta = self._map_range(beta, -math.pi / 2, math.pi / 2, math.pi, 0) - femur_offset
        gamma = self._map_range(gamma, -math.pi, 0, math.pi / 2, -math.pi / 2) + femur_offset + tibia_offset

        return alpha, beta, gamma

    def reach(self, target):
        """
        Computes the inverse kinematics to reach the target point and
        maps the results to the joint ranges of the actual robot.
        """

        alpha, beta, gamma = self.inverse_kinematics(target)
        alpha, beta, gamma = self.translate((alpha, beta, gamma))

        c_min = math.radians(self.config['coxa']['range_min'])
        c_max = math.radians(self.config['coxa']['range_max'])
        f_min = math.radians(self.config['femur']['range_min'])
        f_max = math.radians(self.config['femur']['range_max'])
        t_min = math.radians(self.config['tibia']['range_min'])
        t_max = math.radians(self.config['tibia']['range_max'])

        if alpha < c_min or alpha > c_max:
            raise ValueError(f"Computed coxa value [{alpha}] out of range {c_min}, {c_max}")

        if beta < f_min or beta > f_max:
            raise ValueError(f"Computed femur value [{beta}] out of range {f_min}, {f_max}")

        if gamma < t_min or gamma > t_max:
            raise ValueError(f"Computed tibia value [{gamma}] out of range {t_min}, {t_max}")

        # TODO this will require further investigation
        if self.invert:
            alpha *= -1
            beta *= -1
            gamma *= -1

        return alpha, beta, gamma

    def forward_kinematics(self, angles):
        """
        Computes the forward kinematics based on joint angles (alpha, beta, gamma).
        Returns the 3D position of the end effector.
        """

        alpha, beta, gamma = angles

        c = self.config['coxa']['length']
        f = self.config['femur']['length']
        t = self.config['tibia']['length']

        # Position of the femur base relative to the body
        x_coxa = c * math.cos(alpha)
        y_coxa = c * math.sin(alpha)
        z_coxa = 0

        # Effective angle for the femur and tibia in the vertical plane
        beta_eff = beta
        gamma_eff = gamma

        # Position of the femur end (tibia base)
        x_femur = x_coxa + f * math.cos(beta_eff) * math.cos(alpha)
        y_femur = y_coxa + f * math.cos(beta_eff) * math.sin(alpha)
        z_femur = z_coxa + f * math.sin(beta_eff)

        # Position of the tibia end (foot)
        x_tibia = x_femur + t * math.cos(beta_eff + gamma_eff) * math.cos(alpha)
        y_tibia = y_femur + t * math.cos(beta_eff + gamma_eff) * math.sin(alpha)
        z_tibia = z_femur + t * math.sin(beta_eff + gamma_eff)

        return x_tibia, y_tibia, z_tibia


class Hexapod:

    def __init__(self, config, name='hexapod'):

        # Read the config if a Path is given
        if isinstance(config, Path):
            with open(config, 'r') as f:
                config = json.load(f)

        # Select the particular hexapod (a config file can hold more than one of them)
        config = config[name]

        self.legs = [Leg(config[f'leg_{i + 1}'], invert=(i>2)) for i in range(6)]
        self.coxa_joints = [
            (
                config[f'leg_{i + 1}']['joint']['x'],
                config[f'leg_{i + 1}']['joint']['y'],
                config[f'leg_{i + 1}']['joint']['z']
            ) for i in range(6)
        ]

        # Position and orientation of the main body
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0]

        self.current_angles = [[0, 0, 0] for _ in range(6)]  # Current angles for all legs
        self.target_angles = [[0, 0, 0] for _ in range(6)]  # Target angles for all legs

    def stand(self, height, x_offset=0, y_offset=120):
        """
        Computes the target angles to the stand position.
        All offsets are expressed in mm.
        """

        targets = [
            (x_offset, y_offset, -height) for _ in range(6)
        ]

        for i, (leg, target) in enumerate(zip(self.legs, targets)):
            self.target_angles[i] = [*leg.reach(target)]

    def update(self, speed=0.1, eps=1e-3):

        if not self.is_at_target():

            for i in range(6):

                # Interpolate each joint (Alpha, Beta, Gamma)
                for j in range(3):

                    current = self.current_angles[i][j]
                    target = self.target_angles[i][j]

                    if abs(current - target) > eps:
                        step = speed * (target - current)
                        self.current_angles[i][j] += step

                        # Clamp to prevent overshooting
                        if abs(self.current_angles[i][j] - target) < eps:
                            self.current_angles[i][j] = target

    def is_at_target(self, eps=1e-3):
        """
        Checks if all legs have reached their target angles.
        """

        for current, target in zip(self.current_angles, self.target_angles):
            if any(abs(c - t) > eps for c, t in zip(current, target)):
                return False
            return True

class PyBulletController:

    def __init__(self, hexapod, joint_mapping):
        self.hexapod = hexapod
        self.joint_mapping = joint_mapping

    def step(self, **kwargs):

        self.hexapod.update(**kwargs)

        for i, leg in enumerate(self.hexapod.legs):

            # Take the mapping for the respective leg
            leg = self.joint_mapping[f'leg_{i + 1}']
            coxa_joint = leg['coxa_joint']
            femur_joint = leg['femur_joint']
            tibia_joint = leg['tibia_joint']

            # Take the current angle for the respective leg
            coxa_angle, femur_angle, tibia_angle = self.hexapod.current_angles[i]

            # Move the joints to the target position
            p.setJointMotorControl2(robotID, coxa_joint, p.POSITION_CONTROL, targetPosition=coxa_angle)
            p.setJointMotorControl2(robotID, femur_joint, p.POSITION_CONTROL, targetPosition=femur_angle)
            p.setJointMotorControl2(robotID, tibia_joint, p.POSITION_CONTROL, targetPosition=tibia_angle)


if __name__ == '__main__':

    # --------------------------------- PyBullet --------------------------------- #

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

    # ---------------------------------- Hexapod --------------------------------- #

    config_path = Path('hexapod.json')
    hexapod = Hexapod(config_path)

    joint_mapping = {
        f'leg_{i + 1}': {
            'coxa_joint': revolute_joints[0 + i * 3],
            'femur_joint': revolute_joints[1 + i * 3],
            'tibia_joint': revolute_joints[2 + i * 3]
        } for i in range(6)
    }

    controller = PyBulletController(hexapod, joint_mapping)

    # ------------------------------ Simulation loop ----------------------------- #

    try:

        hexapod.stand(height=80)

        for i in range(10000):

            delta_time = 1. / 240.

            # Step the simulation
            p.stepSimulation()

            controller.step(speed=0.01)

            time.sleep(delta_time)

    finally:
        # Disconnect from the simulation when done
        p.disconnect()
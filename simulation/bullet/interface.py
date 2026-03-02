"""
PyBullet interface.
"""

import pybullet as p
import numpy as np


class PyBulletInterface:
    """PyBullet hexapod interface."""

    def __init__(self, config, robot_id, joints):
        """
        Initialize the interface.

        Args:
            config: configuration dictionary.
            joints: dictionary mapping leg_name to PyBullet joints (e.g. front_right: [j1, j2, j3]).
        """

        self.config = config

        # Enable joints to accept new values
        self.enabled = True

        # Ordered leg names
        self.leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]
        self.trim = {leg_name: config['hardware']['trim'][leg_name] for leg_name in self.leg_names}
        self.direction = {leg_name: config['hardware']['direction'][leg_name] for leg_name in self.leg_names}

        coxa_min, coxa_max = config['safety']['coxa_range']
        femur_min, femur_max = config['safety']['femur_range']
        tibia_min, tibia_max = config['safety']['tibia_range']
        self.servo_min = {leg_name: [coxa_min, femur_min, tibia_min] for leg_name in self.leg_names}
        self.servo_max = {leg_name: [coxa_max, femur_max, tibia_max] for leg_name in self.leg_names}

        # Legs are initially set to a random value (we don't know the initial state of the hexapod during boot)
        self.current_joint_values = {
            leg: np.array([
                np.random.uniform(coxa_min / 5, coxa_max / 5),
                np.random.uniform(femur_min / 5, femur_max / 5),
                np.random.uniform(tibia_min / 5, tibia_max / 5)
            ]) for leg in self.leg_names
        }

        # Pybullet objects
        self.robot_id = robot_id
        self.joints = joints

    @property
    def min_femur_kinematic_space(self):
        """
        Returns the angle in kinematic space corresponding to the
        minimum femur angle realizable in servo space.
        """
        return self.servo_space_to_kinematic_space(self.leg_names[0], 1, self.servo_min[self.leg_names[0]][1])

    @property
    def max_femur_kinematic_space(self):
        """
        Returns the angle in kinematic space corresponding to the
        maximum femur angle realizable in servo space.
        """
        return self.servo_space_to_kinematic_space(self.leg_names[0], 1, self.servo_max[self.leg_names[0]][1])

    @property
    def min_tibia_kinematic_space(self):
        """
        Returns the angle in kinematic space corresponding to the
        minimum tibia angle realizable in servo space.
        """
        return self.servo_space_to_kinematic_space(self.leg_names[0], 2, self.servo_min[self.leg_names[0]][2])

    @property
    def max_tibia_kinematic_space(self):
        """
        Returns the angle in kinematic space corresponding to the
        maximum tibia angle realizable in servo space.
        """
        return self.servo_space_to_kinematic_space(self.leg_names[0], 2, self.servo_max[self.leg_names[0]][2])

    def _get_joint_position(self, leg: str, joint: int):
        return p.getJointState(self.robot_id, self.joints[leg][joint])[0]

    def _set_joint_position(self, leg: str, joint: int, angle: float):

        # Convert the angle to servo space
        new_angle = self.kinematic_space_to_servo_space(leg, joint, angle)
        new_angle_rad = np.radians(new_angle)

        """
        Important: this teleports the joint to the exact position immediately. It is only suited
        for kinematic simulations. The setJointMotorControl2 mode implements a PD controller that
        gradually moves the joint toward the target position. The joints will require multiple simulation
        steps to reach the target position.
        """
        # p.resetJointState(self.robot_id, self.joints[leg_name][joint_index], new_angle_rad)

        """
        MG996R servos are rated for 11kgf.cm @ 6V.
        1kgf = g x 1kg = 9.8N
        1cm = 0.01m
        1kgf.cm = 9.8N x 0.01m = 0.098Nm
        11kgf.cm = 11 x 0.098Nm = 1.078Nm
        """
        force = 1.08

        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.joints[leg][joint],
            controlMode=p.POSITION_CONTROL,
            targetPosition=new_angle_rad,
            force=force,  # Maximum force the motor (MG996R) can apply
            # positionGain=0.8,
            # velocityGain=0.1
        )

    def update(self):
        """Update the simulation."""

        for leg_name in self.leg_names:
            angles = self.current_joint_values[leg_name]  # (coxa, femur, tibia) angles in degrees
            self._set_joint_position(leg_name, 0, angles[0])
            self._set_joint_position(leg_name, 1, angles[1])
            self._set_joint_position(leg_name, 2, angles[2])

    # Hardware interface

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def kinematic_space_to_servo_space(self, leg: str, joint: int, angle: float) -> float:
        """
        Convert angle from kinematic space to PyBullet joint space using config specification.

        Args:
            leg: Leg name (e.g. front_right, middle_left, ...)
            joint: Joint index (0/1/2)
            angle: Angle in degrees
        """

        angle *= self.direction[leg][joint]
        angle += self.trim[leg][joint]
        servo_min = self.servo_min[leg][joint]
        servo_max = self.servo_max[leg][joint]
        clipped_angle = np.clip(angle, servo_min, servo_max)
        return clipped_angle

    def servo_space_to_kinematic_space(self, leg: str, joint: int, angle: float) -> float:
        """
        Convert angle from Viser joint space back to kinematic space.

        Args:
            leg: Leg name (e.g. front_right, middle_left, ...)
            joint: Joint index (0/1/2)
            angle: Angle in degrees
        """

        angle -= self.trim[leg][joint]
        angle /= self.direction[leg][joint]
        return angle

    def set_joint(self, leg: str, joint: int, angle: float) -> bool:
        """Update only the selected leg/joint. Angle is expressed in kinematic space."""
        self.current_joint_values[leg][joint] = angle
        self.update()
        return True

    def set_leg(self, leg: str, angles: np.ndarray) -> bool:
        """Update only the selected leg. Angles are expressed in kinematic space."""
        self.current_joint_values[leg] = angles
        self.update()
        return True

    def set_all_legs(self, joint_values: dict) -> bool:
        """
        Set all legs to their respective angles. leg_angles is a dictionary
        mapping each leg (by name) to a list of angles. Angles are expressed
        in kinematic space.
        e.g. front_right: [90, 0, 0]
        """
        self.current_joint_values = joint_values
        self.update()
        return True

    def get_joint(self, leg: str, joint: int) -> float:
        """Get the selected leg/joint (kinematic space)."""
        return float(self.current_joint_values[leg][joint])

    def get_leg(self, leg: str) -> np.ndarray:
        """Get the selected leg (kinematic space)."""
        return self.current_joint_values[leg]

    def get_all_legs(self) -> dict:
        """Get the current leg angles (kinematic space)."""
        return self.current_joint_values

    def get_voltage(self) -> float:
        """Return mock voltage value."""
        return 0  # mock value

    def get_current(self) -> float:
        """Return mock current value."""
        return 0  # mock value

    def check(self) -> bool:
        """Check the robot is within the safety limits."""
        return True

    def set_led(pin, p: int, r: int, g: int, b: int) -> bool:
        """Set LED to color."""
        return True

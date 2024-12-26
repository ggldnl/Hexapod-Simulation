import numpy as np
from numpy import pi


class Controller :

    def __init__(self,
                 hexapod,
                 queue_size=100
                 ):
        """
        Hexapod controller, interpolates the current joint angles to the target ones and
        handles the gait sequence.

        Parameters:
            hexapod (Hexapod): Hexapod class used to compute the inverse kinematics.
            queue_size (int): Number of subsequent target configurations that can be precomputed
                and stored.
        """

        self.hexapod = hexapod

        self.current_angles = np.zeros((6, 3))  # Assuming 6 legs with 3 DoF each
        self.target_queue = []
        self.duration = 0  # Total time for interpolation
        self.elapsed = 0  # Time elapsed in the current interpolation
        # self.velocity = np.array([0, 0])  # x, y velocity for gait sequence

    # ----------------------------- Utility functions ---------------------------- #

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def is_done(self):
        return not self.target_queue

    def step(self, dt):
        """
        Update the joint angles by interpolating between current and target configurations.

        Parameters:
            dt (float): Time step in seconds.
        Returns:
            np.ndarray: Updated joint angles in servo configuration.
        """
        if not self.target_queue:
            return self.translate(self.current_angles)  # No target, return current state

        # Get the current target and duration
        target_angles, self.duration = self.target_queue[0]

        # Update elapsed time
        self.elapsed += dt
        progress = min(self.elapsed / self.duration, 1)  # Clamp to [0, 1]

        # Interpolate between current and target angles
        self.current_angles = (
            (1 - progress) * self.current_angles + progress * target_angles
        )

        # Check if the target is reached
        if progress == 1:
            self.target_queue.pop(0)  # Remove the completed target
            self.elapsed = 0  # Reset elapsed time for the next phase

        # Translate angles for the robot structure
        return self.translate(self.current_angles)

    def translate(self, angles):
        """
        Convert joint angles from the kinematic model to servo space.

        Parameters:
            angles (np.ndarray): Joint angles from the kinematic model (6x3).
        Returns:
            np.ndarray: Translated angles for servo motors.
        """

        translated_angles = angles.copy()

        # Translate angles for each leg
        for i, leg_angles in enumerate(translated_angles):
            leg_angles = [
                leg_angles[0],  # Coxa angle is the same
                self.map_range(leg_angles[1], pi / 2, -pi / 2, 0, pi),
                self.map_range(leg_angles[2], -pi / 2, pi / 2, 0, -pi)
            ]
            translated_angles[i] = leg_angles

        # Apply offsets to the angles
        offset = np.deg2rad(25)
        for i, leg_angles in enumerate(translated_angles):
            leg_angles[1] -= offset
            leg_angles[2] += offset

        # Mirror left legs
        for i, leg_angles in enumerate(translated_angles):
            if i > 2:
                leg_angles[1] *= -1

        return translated_angles

    # ------------------------------ Hexapod control ----------------------------- #

    def stand(self, duration, height=100, y_offset=120):
        """
        Make the robot stand. The height and distances are used to describe points
        in leg frames. These distances are the same for each leg.

        Parameters:
            height (float): Height at which the robot should stand.
            y_offset (float): Radial distance from the robot's body.
            duration (float): Time in seconds to interpolate to the target angles.
        """

        # Extend the leg
        targets_1 = [[y_offset, 0, 0] for _ in range(6)]
        angles_1 = self.hexapod.inverse_kinematics(targets_1)
        self.target_queue.append((angles_1, duration / 2))

        # Full lift
        targets_2 = [[y_offset, 0, -height] for _ in range(6)]
        angles_2 = self.hexapod.inverse_kinematics(targets_2)
        self.target_queue.append((angles_2, duration / 2))

    def set_body_pose(self, position, orientation, duration):
        """
        Set the target body pose.

        Parameters:
            position (list): Target body position (x, y, z).
            orientation (list): Target body orientation (roll, pitch, yaw).
            duration (float): Time in seconds to interpolate to the target angles.
        """

        self.hexapod.set_body_pose(position, orientation)
        target_angles = np.array([leg.joint_angles for leg in self.hexapod.legs])
        self.target_queue.append((target_angles, duration))

import numpy as np
from numpy import pi

from action import Action


class Controller :

    def __init__(self,
                 hexapod
                 ):
        """
        Hexapod controller, interpolates the current joint angles to the target ones and
        handles the gait sequence.

        Parameters:
            hexapod (Hexapod): Hexapod class used to compute the inverse kinematics.
        """

        self.hexapod = hexapod

        self.current_angles = np.zeros((6, 3))  # Assuming 6 legs with 3 DoF each
        self.current_action = None
        self.action_queue = []
        self.elapsed = 0  # Time elapsed in the current interpolation
        # self.velocity = np.array([0, 0])  # x, y velocity for gait sequence

    # ----------------------------- Utility functions ---------------------------- #

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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

    def step(self, dt):
        """
        Update the joint angles by interpolating between current and target configurations.

        Parameters:
            dt (float): Time step in seconds.
        Returns:
            np.ndarray: Updated joint angles in servo configuration.
        """
        if not self.current_action:
            # Get the next action if the current one is complete
            if self.action_queue:
                self.current_action = self.action_queue.pop(0)
                self.current_action.started = True
                self.elapsed = 0
            else:
                # No actions to execute
                return self.translate(self.current_angles)

        # Get the current target and duration
        target_angles, duration = self.current_action.current_target()
        if target_angles is None:
            # Action is complete
            self.current_action = None
            return self.translate(self.current_angles)

        # Update elapsed time
        self.elapsed += dt
        progress = min(self.elapsed / duration, 1)  # Clamp progress to [0, 1]

        # Interpolate between current and target angles
        self.current_angles = (
                (1 - progress) * self.current_angles + progress * target_angles
        )

        if progress == 1:
            # Target reached, advance to the next step in the action
            self.current_action.advance()
            self.elapsed = 0

        return self.translate(self.current_angles)

    def is_done(self):
        """
        Check if all actions in the queue are complete.
        """
        return not self.current_action and not self.action_queue

    def add_action(self, action):
        """
        Add an action to the queue.

        Parameters:
            action (Action): An Action object to add to the queue.
        """
        self.action_queue.append(action)

    # ------------------------------ Hexapod control ----------------------------- #

    def wait(self, duration):
        """
        Make the robot wait keeping the current configuration for the given amount of time.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        stand_action = Action(
            configurations=[
                np.array([leg.joint_angles for leg in self.hexapod])
            ],
            durations=[duration]
        )
        self.add_action(stand_action)

    def stand(self, duration, height=100, y_offset=120):
        """
        Make the robot stand. The height and distances are used to describe points
        in leg frames. These distances are the same for each leg.

        Parameters:
            height (float): Height at which the robot should stand.
            y_offset (float): Radial distance from the robot's body.
            duration (float): Time in seconds to interpolate to the target angles.
        """

        stand_action = Action(
            configurations=[

                # Extend the leg
                self.hexapod.inverse_kinematics([[y_offset, 0, 0] for _ in range(6)]),

                # Full lift
                self.hexapod.inverse_kinematics([[y_offset, 0, -height] for _ in range(6)])
            ],
            durations=[duration, duration]
        )
        self.add_action(stand_action)

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

        body_pose_action = Action(
            configurations=[
                target_angles
            ],
            durations=[duration]
        )
        self.add_action(body_pose_action)

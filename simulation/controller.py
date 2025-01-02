import numpy as np

from simulation.state import State
from simulation.action import Action


class Controller:

    def __init__(self, hexapod):

        self.hexapod = hexapod

        self.current_action = None
        self.action_queue = []
        self.elapsed = 0  # Time elapsed in the current interpolation
        # self.velocity = np.array([0, 0])  # x, y velocity for gait sequence

    # ----------------------------- Utility functions ---------------------------- #

    def step(self, dt):
        """
        Update the joint angles by interpolating between current and target states.

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
                return self.hexapod.state.joint_angles

        # Get the current target and duration
        target_state, duration = self.current_action.current_target()
        if target_state is None:
            # Action is complete
            self.current_action = None
            return self.hexapod.state.joint_angles

        # Update elapsed time
        self.elapsed += dt
        progress = min(self.elapsed / duration, 1)  # Clamp progress to [0, 1]

        # Interpolate between current and target state
        self.hexapod.state.interpolate(target_state, progress)

        if progress == 1:
            # Target reached, advance to the next step in the action
            self.current_action.advance()
            self.elapsed = 0

        return self.hexapod.state.joint_angles

    def is_done(self):
        """
        Check if all actions in the queue are complete.
        """
        return not self.current_action and not self.action_queue

    def add_action(self, action):
        """
        Add an action to the queue. An action consists of a state and a duration.
        The robot will try to reach the desired state in the specified amount of time.

        Parameters:
            action (Action): An Action object to add to the queue.
        """
        self.action_queue.append(action)

    # ---------------------------------- Actions --------------------------------- #

    # WARN ensure all the methods in the controller are consistent with respect to targets_in_body_frame
    #   e.g.
    #   1. the previous action set the legs_positions expressed in leg frame in the state;
    #   2. legs_positions could be missing in the current action, making the get_state method take them
    #       from the current state;
    #   3. the default value of targets_in_body_frame will be used (True);
    #   4. the value of targets_in_body_frame will not match: targets points are expressed in leg frame
    #       but targets_in_body_frame is True

    def get_state(self, legs_positions=None, body_position=None, body_orientation=None, convert_to_body_frame=False):

        # At least one of the three should be provided
        if all(param is None for param in (legs_positions, body_position, body_orientation)):
            raise ValueError('You should specify a target configuration to reach. None was given.')

        # TODO add logic to keep the parts of the state that are not specified unchanged

        if convert_to_body_frame:
            legs_positions = self.hexapod.transform_to_body_frame(legs_positions)

        joint_angles = self.hexapod.inverse_kinematics(
            legs_positions,
            body_position,
            body_orientation,
            targets_in_body_frame=True
        )

        # TODO add automatic error handling
        #  -> catch exception in hexapod class and return None if the point is unreachable
        if joint_angles is not None:
            return State(
                body_position=body_position,
                body_orientation=body_orientation,
                legs_positions=legs_positions,
                joint_angles=joint_angles
            )
        else:
            return self.hexapod.state

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
            states=[

                # Extend the leg
                self.get_state(
                    legs_positions=np.array([[y_offset, 0, 0] for _ in range(6)]),
                    convert_to_body_frame=True
                ),

                # Full lift
                self.get_state(
                    legs_positions=np.array([[y_offset, 0, 0] for _ in range(6)]),
                    body_position=np.array([0, 0, height]),
                    convert_to_body_frame=True
                )

            ],
            durations=[
                duration,
                duration
            ]
        )
        self.add_action(stand_action)

    def wait(self, duration):
        """
        Make the robot wait keeping the current configuration for the given amount of time.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        wait_action = Action(
            states=[
                self.hexapod.state
            ],
            durations=[duration]
        )
        self.add_action(wait_action)

    def reach(self, duration, legs_positions=None, body_position=None, body_orientation=None):
        """
        Reach a target configuration (body pose and end effectors positions).

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            legs_positions (np.ndarray): Target end-effector positions for each leg (6x3 matrix).
            body_position (np.ndarray): [x, y, z] position of the body in the world frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame. Default is [0, 0, 0].
        """

        reach_action = Action(
            states=[
                self.get_state(legs_positions, body_position, body_orientation)
            ],
            durations=[duration]
        )
        self.add_action(reach_action)

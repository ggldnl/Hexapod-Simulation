import numpy as np

from simulation.action import Action
from simulation.state import State


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
                return self.hexapod.get_joint_values()

        # Get the current target and duration
        target_state, duration = self.current_action.current_target()
        if target_state is None:
            # Action is complete
            self.current_action = None
            return self.hexapod.get_joint_values()

        # Update elapsed time
        self.elapsed += dt
        progress = min(self.elapsed / duration, 1)  # Clamp progress to [0, 1]

        # Interpolate between current and target state
        self.hexapod.state.interpolate(target_state, progress)

        if progress == 1:
            # Target reached, advance to the next step in the action
            print(f'Action \'{self.current_action.name}\' terminated in time {round(self.elapsed, 2)}s')
            self.current_action.advance()
            self.elapsed = 0

        return self.hexapod.get_joint_values()

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

    def stand(self, duration, height=100, y_offset=120):
        """
        Make the robot stand. The height and distances are used to describe points
        in leg frames. These distances are the same for each leg.

        Parameters:
            height (float): Height at which the robot should stand.
            y_offset (float): Radial distance from the robot's body.
            duration (float): Time in seconds to interpolate to the target angles.
        """

        assert duration > 0, f"The duration cannot be < 0."

        body_position_extend_phase = np.zeros(3)
        body_orientation_extend_phase = np.zeros(3)
        legs_positions_extend_phase = self.hexapod.translate_to_origin_frame(
            np.array([[y_offset, 0, 0] for _ in range(6)])
        )
        joint_values_extend_phase = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions_extend_phase,
            body_position_extend_phase,
            body_orientation_extend_phase
        )

        extend_phase = State(
            legs_positions_extend_phase,
            body_position_extend_phase,
            body_orientation_extend_phase,
            joint_values_extend_phase
        )

        body_position_lift_phase = np.array([0, 0, height])
        body_orientation_lift_phase = np.zeros(3)
        legs_positions_lift_phase = self.hexapod.translate_to_origin_frame(
            np.array([[y_offset, 0, 0] for _ in range(6)])
        )
        joint_values_lift_phase = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions_lift_phase,
            body_position_lift_phase,
            body_orientation_lift_phase
        )

        lift_phase = State(
            legs_positions_lift_phase,
            body_position_lift_phase,
            body_orientation_lift_phase,
            joint_values_lift_phase
        )

        stand_action = Action(
            states=[
                extend_phase,
                lift_phase
            ],
            durations=[
                duration,
                duration
            ],
            name='stand'
        )
        self.add_action(stand_action)

    def wait(self, duration):
        """
        Make the robot wait keeping the current configuration for the given amount of time.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
        """

        assert duration > 0, f"The duration cannot be < 0."

        # Get the last state of the last action of the queue
        current_state = self.hexapod.state
        if self.action_queue:
            current_state = self.action_queue[-1].states[-1]

        wait_action = Action(
            states=[
                current_state.copy()
            ],
            durations=[duration],
            name='wait'
        )
        self.add_action(wait_action)

    def set_body_pose(self, duration, body_position=None, body_orientation=None):
        """
        Set the body pose.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            body_position (np.ndarray): [x, y, z] position of the body in the world frame.
                Defaults to the value in the last state, if any.
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame.
                Defaults to the value in the last state, if any.
        """

        assert duration > 0, f"The duration cannot be < 0."
        if body_position is not None:
            assert body_position.shape == (3,), f"Invalid body position: {body_position}."
        if body_orientation is not None:
            assert body_orientation.shape == (3,), f"Invalid body orientation: {body_position}."


        # Get the last state of the last action of the queue
        last_state = self.hexapod.state
        if self.action_queue:
            last_state = self.action_queue[-1].states[-1]

        current_state = last_state.copy()

        if body_position is not None:
            current_state.body_position = body_position

        if body_orientation is not None:
            current_state.body_orientation = body_orientation

        current_state.joint_values = self.hexapod.inverse_kinematics_origin_frame(
            current_state.legs_positions,
            current_state.body_position,
            current_state.body_orientation
        )

        reach_action = Action(
            states=[
                current_state
            ],
            durations=[duration],
            name='set_body_pose'
        )
        self.add_action(reach_action)

    def set_legs_positions(self, duration, legs_positions):
        """
        Set the legs positions, specified in origin frame.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            legs_positions (np.ndarray): End-effector positions origin frame.
                Defaults to the value in the last state, if any.
        """

        assert duration > 0, f"The duration cannot be < 0."
        assert legs_positions.shape == (6, 3), f"Invalid leg position: {legs_positions}."

        # Get the last state of the last action of the queue
        last_state = self.hexapod.state
        if self.action_queue:
            last_state = self.action_queue[-1].states[-1]

        current_state = last_state.copy()
        current_state.joint_values = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions,
            current_state.body_position,
            current_state.body_orientation
        )

        reach_action = Action(
            states=[
                current_state
            ],
            durations=[duration],
            name='set_legs_positions'
        )
        self.add_action(reach_action)

    def set_leg_position(self, duration, leg_index, leg_position):
        """
        Set the position of the specified leg. The target is specified in origin frame.

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            leg_index (int): Leg index.
            leg_position (np.ndarray): End-effector positions origin frame.
                Defaults to the value in the last state, if any.
        """

        assert duration > 0, f"The duration cannot be < 0."
        assert 0 <= leg_index <= 5, f"Leg index out of range: {leg_index}."
        assert leg_position.shape == (3, ), f"Invalid leg position: {leg_position}."

        # Get the last state of the last action of the queue
        last_state = self.hexapod.state
        if self.action_queue:
            last_state = self.action_queue[-1].states[-1]

        current_state = last_state.copy()

        legs_positions = current_state.legs_positions
        legs_positions[leg_index] = leg_position

        current_state.joint_values = self.hexapod.inverse_kinematics_origin_frame(
            legs_positions,
            current_state.body_position,
            current_state.body_orientation
        )

        reach_action = Action(
            states=[
                current_state
            ],
            durations=[duration],
            name='set_leg_position'
        )
        self.add_action(reach_action)

    def reach(self, duration, legs_positions=None, body_position=None, body_orientation=None):
        """
        Reach a target configuration (body pose + legs positions).

        Parameters:
            duration (float): Time in seconds to interpolate to the target angles.
            body_position (np.ndarray): [x, y, z] position of the body in the world frame.
                Defaults to the value in the last state, if any.
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame.
                Defaults to the value in the last state, if any.
            legs_positions (np.ndarray): End-effector positions origin frame.
                Defaults to the value in the last state, if any.
        """

        assert duration > 0, f"The duration cannot be < 0."
        if legs_positions is not None:
            assert legs_positions.shape == (6, 3), f"Invalid body position: {body_position}."
        if body_position is not None:
            assert body_position.shape == (3,), f"Invalid body position: {body_position}."
        if body_orientation is not None:
            assert body_orientation.shape == (3,), f"Invalid body orientation: {body_position}."

        # Get the last state of the last action of the queue
        last_state = self.hexapod.state
        if self.action_queue:
            last_state = self.action_queue[-1].states[-1]

        current_state = last_state.copy()

        if legs_positions is not None:
            current_state.legs_positions = legs_positions

        if body_position is not None:
            current_state.body_position = body_position

        if body_orientation is not None:
            current_state.body_orientation = body_orientation

        current_state.joint_values = self.hexapod.inverse_kinematics_origin_frame(
            current_state.legs_positions,
            current_state.body_position,
            current_state.body_orientation
        )

        reach_action = Action(
            states=[
                current_state
            ],
            durations=[duration],
            name='reach'
        )
        self.add_action(reach_action)



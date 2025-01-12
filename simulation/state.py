class State:

    def __init__(self, legs_positions, body_position, body_orientation, joint_values):
        """
        Represents the state of the robot in a given instant. We will compute the state of the
        robot in some key points and interpolate between them. This lets us:
        1. interpolate in joint space, thus with less computational cost;
        2. retrieve the body pose and legs positions associated to a certain joint configuration
            at any time;

        Parameters:
            body_position (np.ndarray): Body position in origin frame.
            body_orientation (np.ndarray): Body orientation in origin frame.
            legs_positions (np.ndarray): Position of the end effectors of each leg in origin frame.
            joint_values (np.ndarray): Joint angles that realize the state.
        """

        self.body_position = body_position
        self.body_orientation = body_orientation
        self.legs_positions = legs_positions
        self.joint_values = joint_values

    def interpolate(self, target_state, progress):
        """
        Interpolate between the current state and another one.

        Parameters:
            target_state (State): The new state.
            progress (float): Interpolation progress (float in range [0, 1]).
        """

        self.joint_values = (
                (1 - progress) * self.joint_values + progress * target_state.joint_values
        )

    def copy(self):
        """
        Creates a copy of the current state.

        Returns:
            State: A new instance of State with identical values.
        """
        return State(
            self.legs_positions.copy(),
            self.body_position.copy(),
            self.body_orientation.copy(),
            self.joint_values.copy()
        )
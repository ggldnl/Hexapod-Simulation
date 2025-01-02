class State:

    def __init__(self, legs_positions, body_position, body_orientation, joint_angles):
        """
        Represents the state of the robot in a given instant. We will compute the state of the
        robot in some key points and interpolate between them. This lets us:
        1. interpolate in joint space, thus with less computational cost;
        2. have the current body pose and legs positions at any time during interpolation of the joints;

        Parameters:
            body_position (np.ndarray): Body position.
            body_orientation (np.ndarray): Body orientation.
            legs_positions (np.ndarray): Position of the end effectors of each leg.
            joint_angles (np.ndarray): Joint angles that realize the state.
        """

        self.body_position = body_position
        self.body_orientation = body_orientation
        self.legs_positions = legs_positions
        self.joint_angles = joint_angles

    def interpolate(self, target_state, progress):
        """
        Interpolate between the current state and another one.

        Parameters:
            target_state (State): The new state.
            progress (float): Interpolation progress (float in range [0, 1]).
        """

        self.body_position = (
            (1 - progress) * self.body_position + progress * target_state.body_position
        )

        self.body_orientation = (
                (1 - progress) * self.body_orientation + progress * target_state.body_orientation
        )

        self.legs_positions = (
                (1 - progress) * self.legs_positions + progress * target_state.legs_positions
        )

        self.joint_angles = (
                (1 - progress) * self.joint_angles + progress * target_state.joint_angles
        )

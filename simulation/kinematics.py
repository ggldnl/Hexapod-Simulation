import numpy as np


def rotation_matrix(angles):
    """
    Compute a rotation matrix from roll, pitch, and yaw angles.

    Parameters:
        angles (np.ndarray): 1x3 array of angles (roll, pitch, yaw).

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    roll, pitch, yaw = angles

    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return R_z @ R_y @ R_x


def transformation_matrix(rotation, translation):
    """
    Compute a 4x4 transformation matrix from rotation and translation.

    Parameters:
        rotation (np.ndarray): 3x3 rotation matrix.
        translation (np.ndarray): 1x3 translation vector.

    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def rotation_matrix_to_euler_angles(R):
    """
    Converts a 3x3 rotation matrix to Euler angles (XYZ convention).

    Parameters:
        R (np.ndarray): 3x3 numpy array representing the rotation matrix.

    Returns:
        tuple(float, float, float): Tuple of (roll, pitch, yaw) in radians.
    """
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"

    # Check for gimbal lock
    if np.abs(R[2, 0]) != 1:
        pitch = -np.arcsin(R[2, 0])  # beta
        roll = np.arctan2(R[2, 1], R[2, 2])  # alpha
        yaw = np.arctan2(R[1, 0], R[0, 0])  # gamma
    else:
        # Gimbal lock: pitch is ±90 degrees
        yaw = 0  # Can set yaw arbitrarily
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])

    return roll, pitch, yaw


class LegModel:

    def __init__(self, coxa, femur, tibia):
        """
        Leg kinematics.

        Parameters:
            coxa (float): Length of the coxa link.
            femur (float): Length of the femur link.
            tibia (float): Length of the tibia link.
        """

        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

    def __str__(self):
        return (f"Leg ("
                f"coxa={self.coxa}, "
                f"femur={self.femur}, "
                f"tibia={self.tibia}"
                f")")

    def forward_kinematics(self, angles):
        """
        Computes the position of the end effector in the leg's reference frame.

        Parameters:
            angles (np.array): List of joint angles in radians.

        Returns:
            np.ndarray: End effector position in the leg's reference frame.
        """

        coxa_position = np.array([
            self.coxa * np.cos(angles[0]),
            self.coxa * np.sin(angles[0]),
            0
        ])  # Coxa joint position

        femur_position = coxa_position + np.array([
            self.femur * np.cos(angles[0]) * np.cos(angles[1]),
            self.femur * np.sin(angles[0]) * np.cos(angles[1]),
            self.femur * np.sin(angles[1])
        ])  # Femur joint position

        tibia_position = femur_position + np.array([
            self.tibia * np.cos(angles[0]) * np.cos(angles[1] + angles[2]),
            self.tibia * np.sin(angles[0]) * np.cos(angles[1] + angles[2]),
            self.tibia * np.sin(angles[1] + angles[2])
        ])  # Tibia joint position (end effector)

        return tibia_position

    def inverse_kinematics(self, target):
        """
        Compute the joint angles to reach the target point (expressed in the leg's reference frame).

        Parameters:
            target (np.array): Cartesian point for the end effect to reach, expressed in leg frame.

        Returns:
            np.ndarray: Joint angles to reach the required end effector position.

        Raises:
            ValueError: If the required point is unreachable.
        """
        x, y, z = target

        c = self.coxa
        f = self.femur
        t = self.tibia

        # Coxa angle
        alpha = np.arctan2(y, x)
        r = np.sqrt(x ** 2 + y ** 2) - c

        # Compute 2D planar IK for femur and tibia
        d = np.sqrt(r ** 2 + z ** 2)  # Distance to the target
        if d > (f + t) or d < abs(f - t):
            raise ValueError(f"Target ({target[0]}, {target[1]}, {target[2]}) is out of reach (d={d}).")

        cos_angle2 = (f ** 2 + t ** 2 - d ** 2) / (2 * f * t)
        gamma = np.arccos(cos_angle2) - np.pi

        cos_angle1 = (f ** 2 + d ** 2 - t ** 2) / (2 * f * d)
        beta = np.arctan2(z, r) + np.arccos(cos_angle1)

        return alpha, beta, gamma


class HexapodModel:

    def __init__(self, legs, leg_frames):
        """
        Class containing the kinematic model for the Hexapod robot. This class is not intended to
        keep a state: it should only provide access to the formulas.

        Notation:
        1. origin_frame (origin_position + origin_orientation):
            Represents the hexapod robot's central reference point in relation to the world.
            This is the "global" frame that tracks the robot's overall position and orientation.
        2. body_frame (body_position + body_orientation):
            Represents the hexapod's body relative to the robot's base. This frame accounts
            for tilt, lean, or movement of the body relative to the base.
        3. leg frames:
            Represents the points where the legs are attached to the body.
            Each leg has its own frame.
        The forward and inverse kinematics solutions are expressed the origin_frame.

        Parameters:
            legs (list[LegModel]): List of legs.
            leg_frames (list[np.ndarray]): List of leg frames.
        """

        self.legs = legs
        self.leg_frames = leg_frames

    def __iter__(self):
        return iter(zip(self.legs, self.leg_frames))

    def forward_kinematics(self, joint_angles, body_position=None, body_orientation=None):
        """
        Computes the positions of all leg end effectors in the body frame.

        Parameters:
            joint_angles (np.ndarray): Joint angles for each leg (6x3 matrix).

        Returns:
            np.ndarray: End-effector positions in the body frame (6x3 matrix).
        """

        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        positions = []
        for leg, leg_frame, angles in zip(self.legs, self.leg_frames, joint_angles):
            end_effector_leg_frame = leg.forward_kinematics(angles)
            end_effector_body_frame = body_frame @ leg_frame @ np.array([*end_effector_leg_frame, 1])
            positions.append(end_effector_body_frame[:3])
        return np.array(positions)

    def inverse_kinematics_origin_frame(self, legs_positions, body_position=None, body_orientation=None):
        """
        Compute the leg joint angles to achieve the desired body pose and end-effector positions. The
        legs target points are expressed in the origin frame.

        Parameters:
            legs_positions (np.ndarray): Target end-effector positions for each leg (6x3 matrix) expressed in the origin frame.
            body_position (np.ndarray): [x, y, z] position of the body in the origin frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the origin frame. Default is [0, 0, 0].

        Returns:
            np.ndarray: Joint angles for each leg.
        """

        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        all_joint_angles = []
        for i, (leg, leg_frame, target_position) in enumerate(zip(self.legs, self.leg_frames, legs_positions)):

            try:

                # Compute the new leg frame after displacement and rotation of the body
                new_leg_frame = body_frame @ leg_frame
                new_leg_frame_inv = np.linalg.inv(new_leg_frame)

                # Express the same point in the new leg frame
                target_position_leg_frame = new_leg_frame_inv @ np.array([*target_position, 1])

                # Compute inverse kinematics
                joint_angles = leg.inverse_kinematics(target_position_leg_frame[:3])
                all_joint_angles.append(joint_angles)

            except ValueError as e:

                raise ValueError(f'Unable to reach target point for leg {i}: {e}')

        return np.array(all_joint_angles)

    def inverse_kinematics_leg_frame(self, legs_positions, body_position=None, body_orientation=None):
        """
        Compute the leg joint angles to achieve the desired body pose and end-effector positions. The
        legs target points are expressed in each leg's frame.

        Parameters:
            legs_positions (np.ndarray): Target end-effector positions for each leg (6x3 matrix).
            body_position (np.ndarray): [x, y, z] position of the body in the world frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame. Default is [0, 0, 0].

        Returns:
            np.ndarray: Joint angles for each leg.
        """

        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        new_body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        all_joint_angles = []
        for i, (leg, leg_frame, target_position) in enumerate(zip(self.legs, self.leg_frames, legs_positions)):

            try:

                # Compute the new leg frame after displacement and rotation of the body
                new_leg_frame = new_body_frame @ leg_frame
                new_leg_frame_inv = np.linalg.inv(new_leg_frame)

                # Express the same point in the new leg frame
                target_position_leg_frame = new_leg_frame_inv @ (leg_frame @ np.array([*target_position, 1]))

                # Compute inverse kinematics
                joint_angles = leg.inverse_kinematics(target_position_leg_frame[:3])
                all_joint_angles.append(joint_angles)

            except ValueError as e:

                raise ValueError(f'Unable to reach target point for leg {i}: {e}')

        return np.array(all_joint_angles)


    def translate_to_origin_frame(self, leg_positions, body_position=None, body_orientation=None):
        """
        Translates the positions of leg end-effectors from leg frames to the origin frame.

        Parameters:
            leg_positions (np.ndarray): End-effector positions in each leg's frame (6x3 matrix).
            body_position (np.ndarray): [x, y, z] position of the body in the origin frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the origin frame. Default is [0, 0, 0].

        Returns:
            np.ndarray: End-effector positions in the origin frame (6x3 matrix).
        """

        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        origin_frame_positions = []
        for leg_frame, leg_position in zip(self.leg_frames, leg_positions):

            # Apply transformations to move the position from the leg frame to the origin frame
            position_in_origin_frame = body_frame @ leg_frame @ np.array([*leg_position, 1])

            origin_frame_positions.append(position_in_origin_frame[:3])

        return np.array(origin_frame_positions)



if __name__ == '__main__':

    import argparse
    import json

    parser = argparse.ArgumentParser(description="Test kinematic model.")
    parser.add_argument("-c", "--config", type=str, default='simulation/config/config.json',
                        help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")
    parser.add_argument('-X', '--body-x', type=float, default=0, help="Body x coordinate")
    parser.add_argument('-Y', '--body-y', type=float, default=0, help="Body y coordinate")
    parser.add_argument('-Z', '--body-z', type=float, default=100, help="Body z coordinate")
    parser.add_argument('-r', '--roll', type=float, default=0, help="Roll angle (degrees)")
    parser.add_argument('-p', '--pitch', type=float, default=10, help="Pitch angle (degrees)")
    parser.add_argument('-w', '--yaw', type=float, default=10, help="Yaw angle (degrees)")

    args = parser.parse_args()

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a Hexapod object
    legs_config = config['hexapod']['legs']

    legs = []
    leg_frames = []
    current_angles = []

    for i, leg in enumerate(legs_config):
        current_coxa_angle = legs_config[leg]["coxa"]["cur_angle"]
        current_femur_angle = legs_config[leg]["femur"]["cur_angle"]
        current_tibia_angle = legs_config[leg]["tibia"]["cur_angle"]
        current_angles.append([current_coxa_angle, current_femur_angle, current_tibia_angle])

        coxa_length = legs_config[leg]["coxa"]["length"]
        femur_length = legs_config[leg]["femur"]["length"]
        tibia_length = legs_config[leg]["tibia"]["length"]

        position = legs_config[leg]["frame"]["position"]
        orientation = legs_config[leg]["frame"]["orientation"]

        legs.append(LegModel(coxa_length, femur_length, tibia_length))
        leg_frames.append(transformation_matrix(rotation_matrix(orientation), position))

    hexapod = HexapodModel(legs, leg_frames)

    # Target body pose and leg targets
    body_position = np.array([args.body_x, args.body_y, args.body_z])
    body_orientation = np.array([np.deg2rad(args.roll), np.deg2rad(args.pitch), np.deg2rad(args.yaw)])
    leg_positions_origin_frame = np.array([
        [150, 150, 0],
        [180, 0, 0],
        [150, -150, 0],
        [-150, -150, 0],
        [-180, 0, 0],
        [-150, 150, 0]
    ])

    print(f'\nBody position:\n{np.round(body_position, 2)}')
    print(f'\nBody orientation:\n{np.round(body_orientation, 2)}')
    print(f'\nLeg positions (origin frame):\n{np.round(leg_positions_origin_frame, 2)}')

    joint_angles = hexapod.inverse_kinematics_origin_frame(leg_positions_origin_frame, body_position, body_orientation)
    print(f'\nJoint angles:\n{np.round(joint_angles, 2)}')

    computed_legs_positions = hexapod.forward_kinematics(joint_angles, body_position, body_orientation)
    print(f'\nForward kinematics:\n{np.round(computed_legs_positions, 2)}')

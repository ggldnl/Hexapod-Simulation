import numpy as np


def rotation_x(theta):
    return np.array([
        [1, 0,              0,             0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta),  np.cos(theta), 0],
        [0, 0,              0,             1]
    ])

def rotation_y(theta):
    return np.array([
        [np.cos(theta),  0, np.sin(theta), 0],
        [0,              1, 0,             0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0,              0, 0,             1]
    ])

def rotation_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

def rotation_matrix(angles):
    roll, pitch, yaw = angles
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)

def transformation_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation[:3, :3]
    matrix[:3, 3] = translation
    return matrix


class LegModel:

    def __init__(self, coxa, femur, tibia, frame=None):
        """
        Leg kinematics.

        Parameters:
            coxa (float): Length of the coxa link.
            femur (float): Length of the femur link.
            tibia (float): Length of the tibia link.
            frame (np.array): Transformation matrix that describes how the leg is attached to the body.
                This won't be used in this class, it's just a way to tie together a leg and the respective
                transformation matrix. By default, the frame will be the transformation matrix obtained
                by placing the leg at the origin with orientation [0, 0, 0].
        """

        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

        if frame is None:
            frame = transformation_matrix(rotation_matrix([0, 0, 0]), [0, 0, 0])
        self.frame = frame


    def __str__(self):
        return (f"Leg ("
                f"coxa={self.coxa}, "
                f"femur={self.femur}, "
                f"tibia={self.tibia}"
                f")")

    def coxa_T(self, alpha):
        """
        Compute the transformation matrix for the coxa joint.

        Parameters:
            alpha (float): Coxa angle in radians.
        """
        return np.array([
            [np.cos(alpha), -np.sin(alpha), 0, self.coxa],
            [np.sin(alpha), np.cos(alpha), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def femur_T(self, beta):
        """
        Compute the transformation matrix for the femur joint.

        Parameters:
            beta (float): Femur angle in radians.
        """
        return np.array([
            [np.cos(beta), -np.sin(beta), 0, self.femur],
            [0, 0, 1, 0],
            [-np.sin(beta), -np.cos(beta), 0, 0],
            [0, 0, 0, 1]
        ])

    def tibia_T(self, gamma):
        """
        Compute the transformation matrix for the tibia joint.

        Parameters:
            gamma (float): Tibia angle in radians.
        """
        return np.array([
            [np.cos(gamma), -np.sin(gamma), 0, self.tibia],
            [0, 0, 1, 0],
            [-np.sin(gamma), -np.cos(gamma), 0, 0],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, angles):
        """
        Computes the position of the end effector in the leg's reference frame.

        Parameters:
            angles (np.array): List of joint angles in radians.
        """
        end_effector = np.array([0, 0, 0, 1])  # Homogeneous coordinates
        transformation = self.coxa_T(angles[0]) @ self.femur_T(angles[1]) @ self.tibia_T(angles[2])
        result = transformation @ end_effector
        return result[:3]

    def inverse_kinematics(self, target):
        """
        Compute the joint angles to reach the target point (expressed in the leg's reference frame).

        Parameters:
            target (np.array): Cartesian point for the end effect to reach, expressed in leg frame.
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
        if d > (f + t):
            raise ValueError(f"Target ({target[0]}, {target[1]}, {target[2]}) is out of reach (d={d}).")

        cos_angle2 = (f ** 2 + t ** 2 - d ** 2) / (2 * f * t)
        gamma = np.arccos(cos_angle2) - np.pi

        cos_angle1 = (f ** 2 + d ** 2 - t ** 2) / (2 * f * d)
        beta = np.arctan2(z, r) + np.arccos(cos_angle1)

        return alpha, beta, gamma


class HexapodModel:

    def __init__(self, legs):
        """
        Hexapod kinematics.

        Parameters:
            legs (list[LegModel]): List of legs.
        """

        self.legs = legs

    def __iter__(self):
        return iter(self.legs)

    def forward_kinematics(self, joint_angles, body_position=None, body_orientation=None):
        """
        Compute the global end-effector positions for the given joint angles and body pose.

        Parameters:
            joint_angles (np.ndarray): A list of joint angle tuples (alpha, beta, gamma) for each leg.
            body_position (np.ndarray): Body position [x, y, z]. Default is [0, 0, 0].
            body_orientation (np.ndarray): Body orientation [roll, pitch, yaw]. Default is [0, 0, 0].
        """

        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)
        all_end_effector_positions = []

        for leg, angles in zip(self.legs, joint_angles):

            # Compute the end-effector position in the leg's local frame
            end_effector_local = leg.forward_kinematics(angles)

            # Transform the local position to the global frame
            end_effector_in_leg_frame = np.append(end_effector_local, 1)
            end_effector_in_body_frame = leg.frame @ end_effector_in_leg_frame
            end_effector_global = body_frame @ end_effector_in_body_frame

            all_end_effector_positions.append(end_effector_global[:3])

        return np.array(all_end_effector_positions)

    def inverse_kinematics(self, legs_positions, body_position=None, body_orientation=None, targets_in_body_frame=True):
        """
        Compute the leg joint angles to achieve the desired body pose and end-effector positions.

        Parameters:
            legs_positions (np.ndarray): Target end-effector positions for each leg (6x3 matrix).
            body_position (np.ndarray): [x, y, z] position of the body in the world frame. Default is [0, 0, 0].
            body_orientation (np.ndarray): [roll, pitch, yaw] orientation of the body in the world frame. Default is [0, 0, 0].
            targets_in_body_frame (bool): If True, the targets are expressed in body frames, otherwise they are expressed in leg frame.
                Setting the targets in leg frame could be useful during gait.

        Returns:
            np.ndarray: Joint angles for each leg.
        """
        if body_position is None:
            body_position = np.zeros(3)

        if body_orientation is None:
            body_orientation = np.zeros(3)

        new_body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

        all_joint_angles = []
        for i, (leg, target_position) in enumerate(zip(self.legs, legs_positions)):

            try:

                # Compute the new leg frame after displacement and rotation of the body
                new_leg_frane = new_body_frame @ leg.frame
                new_leg_frame_inv = np.linalg.inv(new_leg_frane)

                if targets_in_body_frame:

                    # Express the same point in the new leg frame
                    target_position_leg_frame = new_leg_frame_inv @ np.array([*target_position, 1])

                else:  # Points are in leg frames

                    target_position_leg_frame = new_leg_frame_inv @ (leg.frame @ np.array([*target_position, 1]))

                # Compute inverse kinematics
                joint_angles = leg.inverse_kinematics(target_position_leg_frame[:3])
                all_joint_angles.append(joint_angles)

            except ValueError as e:

                raise ValueError(f'Unable to reach target point for leg {i}: {e}')


        return np.array(all_joint_angles)
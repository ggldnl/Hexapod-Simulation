import plotly.graph_objects as go
from abc import ABC
import numpy as np

from simulation.kinematics import HexapodModel, LegModel
from simulation.kinematics import transformation_matrix, rotation_matrix
from simulation.state import State


class RobotInterface(HexapodModel, ABC):
    """
    Wrapper to the kinematic model that also Keeps an internal state
    (body position, body orientation and legs positions) and handles
    exceptions.
    """

    def __init__(self, config):

        # Misc
        self.config = config

        body_config = self.config["body"]

        legs = []
        current_angles = []
        for i, leg in enumerate(self.config["legs"]):

            current_coxa_angle = self.config["legs"][leg]["coxa"]["cur_angle"]
            current_femur_angle = self.config["legs"][leg]["femur"]["cur_angle"]
            current_tibia_angle = self.config["legs"][leg]["tibia"]["cur_angle"]
            current_angles.append([current_coxa_angle, current_femur_angle, current_tibia_angle])

            coxa_length = self.config["legs"][leg]["coxa"]["length"]
            femur_length = self.config["legs"][leg]["femur"]["length"]
            tibia_length = self.config["legs"][leg]["tibia"]["length"]

            position = self.config["legs"][leg]["frame"]["position"]
            orientation = self.config["legs"][leg]["frame"]["orientation"]
            leg_frame = transformation_matrix(rotation_matrix(orientation), position)

            legs.append(LegModel(coxa_length, femur_length, tibia_length, leg_frame))

        super().__init__(legs=legs)

        # Robot state
        body_position = np.array(body_config["frame"]["position"])
        body_orientation = np.array(body_config["frame"]["orientation"])
        current_angles = np.array(current_angles)
        legs_positions = self.forward_kinematics(current_angles, body_position, body_orientation)

        # Robot state
        self.state = State(
            legs_positions,
            body_position,
            body_orientation,
            current_angles
        )

    def transform_to_body_frame(self, target_leg_positions):
        """
        Transforms target end-effector positions from their respective leg frames to the body frame.

        Parameters:
            target_leg_positions (np.ndarray): Target positions for each leg in their respective leg frames (6x3 matrix).
        """

        # Create the current body frame transformation matrix
        body_frame = transformation_matrix(rotation_matrix(self.state.body_orientation), self.state.body_position)

        transformed_positions = []
        for leg, target_position in zip(self.legs, target_leg_positions):
            # Transform target position from the leg frame to the body frame
            target_position_body_frame = body_frame @ (leg.frame @ np.array([*target_position, 1]))
            transformed_positions.append(target_position_body_frame[:3])

        return np.array(transformed_positions)


class VirtualRobot(RobotInterface):
    """
    Represents the robot in a simulation environment. The robot is described in its parts with a URDF
    but this has different joint ranges with respect both to the kinematic model and the actual robot.
    This class applies a transformation to the computed joint values to mimic the real movement.
    """

    def __init__(self, config):
        super().__init__(config)

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def translate(self, angles):
        """
        Transform the angles in way that the robot moves consistently in a simulated environment.

        Parameters:
            angles (np.ndarray): Joint angles.
        """
        translated_angles = angles.copy()

        # Translate angles for each leg
        for i, leg_angles in enumerate(translated_angles):
            leg_angles = [
                leg_angles[0],  # Coxa angle is the same
                self.map_range(leg_angles[1], np.pi / 2, -np.pi / 2, 0, np.pi),
                self.map_range(leg_angles[2], -np.pi / 2, np.pi / 2, 0, -np.pi)
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

    def inverse_kinematics(self, legs_positions, body_position=None, body_orientation=None, targets_in_body_frame=True):
        # All the target points are supposed to be in body frame
        joint_angles = super().inverse_kinematics(
            legs_positions,
            body_position,
            body_orientation,
            targets_in_body_frame=True
        )
        translated_angles = self.translate(joint_angles)
        return translated_angles


class Robot(RobotInterface):
    """
    Represents the real robot. This means we have to translate the joint angles
    computed by the kinematic model into the servo space, check if the conversion
    is valid (the angles do not violate physical constraints) and translate the
    angles as pulse widths.
    """

    def __init__(self, config):
        super().__init__(config)

        # Servo ranges
        self.min_angles = []
        self.max_angles = []
        self.min_pulses = []
        self.max_pulses = []

        for i, leg in enumerate(self.config["legs"]):
            coxa_min_angle = self.config["legs"][leg]["coxa"]["min_angle"]
            femur_min_angle = self.config["legs"][leg]["femur"]["min_angle"]
            tibia_min_angle = self.config["legs"][leg]["tibia"]["min_angle"]
            self.min_angles.append([coxa_min_angle, femur_min_angle, tibia_min_angle])

            coxa_max_angle = self.config["legs"][leg]["coxa"]["max_angle"]
            femur_max_angle = self.config["legs"][leg]["femur"]["max_angle"]
            tibia_max_angle = self.config["legs"][leg]["tibia"]["max_angle"]
            self.max_angles.append([coxa_max_angle, femur_max_angle, tibia_max_angle])

            coxa_min_pulse = self.config["legs"][leg]["coxa"]["min_pulse"]
            femur_min_pulse = self.config["legs"][leg]["femur"]["min_pulse"]
            tibia_min_pulse = self.config["legs"][leg]["tibia"]["min_pulse"]
            self.min_pulses.append([coxa_min_pulse, femur_min_pulse, tibia_min_pulse])

            coxa_max_pulse = self.config["legs"][leg]["coxa"]["max_pulse"]
            femur_max_pulse = self.config["legs"][leg]["femur"]["max_pulse"]
            tibia_max_pulse = self.config["legs"][leg]["tibia"]["max_pulse"]
            self.max_pulses.append([coxa_max_pulse, femur_max_pulse, tibia_max_pulse])

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @classmethod
    def angle_to_pulse(cls, angle, min_pulse, max_pulse):
        # return int(round(cls.map_range(angle, -np.pi/2, np.pi/2, min_pulse, max_pulse)))
        # TODO fix this
        return angle

    def translate(self, angles):
        """
        Translate the angles into the servo space.

        Parameters:
            angles (np.ndarray): Joint angles.
        """
        translated_angles = angles.copy()

        # Translate angles for each leg
        for i, leg_angles in enumerate(translated_angles):
            leg_angles = [
                leg_angles[0],  # Coxa angle is the same
                self.map_range(leg_angles[1], np.pi / 2, -np.pi / 2, 0, np.pi),
                self.map_range(leg_angles[2], -np.pi / 2, np.pi / 2, 0, -np.pi)
            ]
            translated_angles[i] = leg_angles

        # Apply offsets to the angles
        offset = np.deg2rad(25)
        for i, leg_angles in enumerate(translated_angles):
            leg_angles[1] -= offset
            leg_angles[2] += offset

        return np.array(translated_angles)

    def check(self, angles):
        """
        Check if the angles are all in the valid range.

        Parameters:
            angles (np.ndarray): Joint angles.
        """

        for leg_angles, min_leg_angles, max_leg_angles in zip(angles, self.min_angles, self.max_angles):
            for angle, min_angle, max_angle in zip(leg_angles, min_leg_angles, max_leg_angles):
                if not min_angle <= angle <= max_angle:
                    raise ValueError(f'Angle {angle} out of range ({min_angle}, {max_angle})')

        return angles

    def to_pulse(self, angles):
        """
        Converts the angles to pulse widths.

        Parameters:
            angles (np.ndarray): Joint angles.
        """
        joint_pulses = []
        for leg_angles, min_leg_pulses, max_leg_pulses in zip(angles, self.min_pulses, self.max_pulses):
            leg_pulses = []
            for angle, min_pulse, max_pulse in zip(leg_angles, min_leg_pulses, max_leg_pulses):
                leg_pulses.append(self.angle_to_pulse(angle, min_pulse, max_pulse))
            joint_pulses.append(leg_pulses)
        return np.array(joint_pulses)

    def inverse_kinematics(self, legs_positions, body_position=None, body_orientation=None, targets_in_body_frame=True):
        # All the target points are supposed to be in body frame
        joint_angles = super().inverse_kinematics(
            legs_positions,
            body_position,
            body_orientation,
            targets_in_body_frame=True
        )
        translated_angles = self.translate(joint_angles)
        translated_angles = self.check(translated_angles)
        pulses = self.to_pulse(translated_angles)
        return pulses


def draw(hexapod):
    """
    Draws the hexapod using Plotly and labels each leg with its index.
    """

    fig = go.Figure()

    body_frame = transformation_matrix(rotation_matrix(hexapod.state.body_orientation), hexapod.state.body_position)

    # Plot body points and connect them
    body_points = []
    for leg in hexapod.legs:
        new_leg_frame = body_frame @ leg.frame
        x_off, y_off, z_off = new_leg_frame[:3, 3]
        body_points.append((x_off, y_off, z_off))

    body_points.append(body_points[0])  # Close the body loop
    body_points = np.array(body_points)

    # Add body line and points
    fig.add_trace(go.Scatter3d(
        x=body_points[:, 0],
        y=body_points[:, 1],
        z=body_points[:, 2],
        mode='lines+markers',
        line=dict(color='blue'),
        marker=dict(size=5, color='blue'),
        name='Body'
    ))

    # Plot each leg with labels
    for index, (leg, leg_angles) in enumerate(zip(hexapod.legs, hexapod.state.joint_angles)):

        # Joint positions
        c = leg.coxa
        f = leg.femur
        t = leg.tibia

        alpha, beta, gamma = leg_angles

        coxa_pos = np.array([0, 0, 0])
        femur_pos = np.array([c * np.cos(alpha), c * np.sin(alpha), 0])
        tibia_pos = femur_pos + np.array([
            f * np.cos(alpha) * np.cos(beta),
            f * np.sin(alpha) * np.cos(beta),
            f * np.sin(beta)
        ])
        end_effector_pos = tibia_pos + np.array([
            t * np.cos(alpha) * np.cos(beta + gamma),
            t * np.sin(alpha) * np.cos(beta + gamma),
            t * np.sin(beta + gamma)
        ])

        # Apply leg frame transformation
        positions = [coxa_pos, femur_pos, tibia_pos, end_effector_pos]
        positions = [body_frame @ leg.frame @ np.array([*pos, 1]) for pos in positions]
        positions = np.array([pos[:3] for pos in positions])

        # Add leg line and points
        fig.add_trace(go.Scatter3d(
            x=positions[:, 0],
            y=positions[:, 1],
            z=positions[:, 2],
            mode='lines+markers',
            line=dict(color='red'),
            marker=dict(size=5, color='red'),
            name=f'Leg {index}'
        ))

        # Add leg label at end effector position
        fig.add_trace(go.Scatter3d(
            x=[positions[-1, 0]],
            y=[positions[-1, 1]],
            z=[positions[-1, 2]],
            mode='text',
            text=[f'Leg {index}'],
            textposition='top center',
            showlegend=False
        ))

    fig.update_layout(
        scene=dict(
            aspectmode='data',
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z"
        ),
        title="Hexapod Visualization"
    )

    fig.show()


if __name__ == '__main__':

    import argparse
    import json

    parser = argparse.ArgumentParser(description="Code to be run on the Hexapod")
    parser.add_argument("-c", "--config", type=str, default='simulation/config/config.json',
                        help="Path to the robot's configuration file")
    parser.add_argument('-n', '--name', type=str, default='hexapod', help="Name of the robot in the config")

    args = parser.parse_args()

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a Hexapod object
    hexapod = RobotInterface(config[args.name])

    """
    angles = [45, 0, -45, -135, 180, 135]
    angles = [np.deg2rad(a) for a in angles]
    radius = 150
    legs_positions = np.array([
        [round(radius * np.cos(angle), 2), round(radius * np.sin(angle), 2), 0]
        for angle in angles
    ])

    body_position = np.array([0, 0, 60])
    body_orientation = np.array([0, 0, np.deg2rad(10)])

    joint_values = hexapod.inverse_kinematics(
        legs_positions,
        body_position,
        body_orientation,
        targets_in_body_frame=True
    )

    hexapod.state = State(
        legs_positions=legs_positions,
        body_position=body_position,
        body_orientation=body_orientation,
        joint_angles=joint_values
    )
    """

    """
    legs_positions = np.array([[120, 0, 0] for _ in range(6)])
    body_position = np.array([0, 0, 60])
    body_orientation = np.array([0, 0, np.deg2rad(10)])

    joint_values = hexapod.inverse_kinematics(
        legs_positions,
        body_position,
        body_orientation,
        targets_in_body_frame=False
    )

    hexapod.state = State(
        legs_positions=legs_positions,
        body_position=body_position,
        body_orientation=body_orientation,
        joint_angles=joint_values
    )
    """

    draw(hexapod)

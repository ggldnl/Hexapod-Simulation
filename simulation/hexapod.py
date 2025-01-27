import sys
sys.path.append('../Hexapod-Controller/controller')

import plotly.graph_objects as go
import numpy as np

from kinematics import HexapodModel, LegModel
from kinematics import transformation_matrix, rotation_matrix
from state import State


class VirtualHexapod(HexapodModel):
    """
    Represents the robot in a simulation environment. The robot is described by a URDF
    but this has different joint ranges with respect both to the pyre kinematic model and the actual robot.
    This class applies a transformation to the computed joint values to mimic the real movement.
    """

    def __init__(self, config):

        # Misc
        self.config = config

        legs_config = self.config["legs"]

        legs = []
        leg_frames = []
        joint_values = []
        for i, leg in enumerate(legs_config):

            current_coxa_angle = legs_config[leg]["coxa"]["cur_angle"]
            current_femur_angle = legs_config[leg]["femur"]["cur_angle"]
            current_tibia_angle = legs_config[leg]["tibia"]["cur_angle"]
            joint_values.append([current_coxa_angle, current_femur_angle, current_tibia_angle])

            coxa_length = legs_config[leg]["coxa"]["length"]
            femur_length = legs_config[leg]["femur"]["length"]
            tibia_length = legs_config[leg]["tibia"]["length"]

            position = legs_config[leg]["frame"]["position"]
            orientation = legs_config[leg]["frame"]["orientation"]
            leg_frame = transformation_matrix(rotation_matrix(orientation), position)

            legs.append(LegModel(coxa_length, femur_length, tibia_length))
            leg_frames.append(leg_frame)

        super().__init__(legs=legs, leg_frames=leg_frames)

        # Robot state
        body_config = self.config["body"]
        body_position = body_config["frame"]["position"]
        body_orientation = body_config["frame"]["orientation"]
        joint_values = np.array(joint_values)
        legs_positions = self.forward_kinematics(joint_values, body_position, body_orientation)

        self.state = State(legs_positions, body_position, body_orientation, joint_values)

    def get_joint_values(self):
        joint_angles = self.state.joint_angles
        return self.translate(joint_angles)

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

def draw(hexapod, body_position=None, body_orientation=None):
    """
    Draws the hexapod using Plotly and labels each leg with its index.
    """

    fig = go.Figure()

    if body_position is None:
        body_position = np.zeros(3)

    if body_orientation is None:
        body_orientation = np.zeros(3)

    body_frame = transformation_matrix(rotation_matrix(body_orientation), body_position)

    # Plot body points and connect them
    body_points = []
    for leg, leg_frame in zip(hexapod.legs, hexapod.leg_frames):
        new_leg_frame = body_frame @ leg_frame
        x_off, y_off, z_off = new_leg_frame[:3, 3]
        body_points.append((x_off, y_off, z_off))

    print(f'Body points:\n{np.round(body_points, 2)}')

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
    for index, (leg, leg_frame, leg_angles) in enumerate(zip(hexapod.legs, hexapod.leg_frames, hexapod.joint_values)):

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
        positions = [body_frame @ leg_frame @ np.array([*pos, 1]) for pos in positions]
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
    parser.add_argument('-X', '--body-x', type=float, default=0, help="Body x coordinate")
    parser.add_argument('-Y', '--body-y', type=float, default=0, help="Body y coordinate")
    parser.add_argument('-Z', '--body-z', type=float, default=100, help="Body z coordinate")
    parser.add_argument('-r', '--roll', type=float, default=0, help="Roll angle (degrees)")
    parser.add_argument('-p', '--pitch', type=float, default=10, help="Pitch angle (degrees)")
    parser.add_argument('-w', '--yaw', type=float, default=10, help="Yaw angle (degrees)")
    parser.add_argument('-x', '--leg-x', type=float, default=120, help="Leg x coordinate (leg frame)")
    parser.add_argument('-y', '--leg-y', type=float, default=0, help="Leg y coordinate (leg frame)")
    parser.add_argument('-z', '--leg-z', type=float, default=0, help="Leg z coordinate (leg frame)")

    args = parser.parse_args()

    # Read the JSON
    with open(args.config) as f:
        config = json.load(f)

    # Create a Hexapod object
    hexapod = VirtualHexapod(config[args.name])

    # We need to disable the translation as we are dealing with the kinematic model and not the
    # simulated robot
    hexapod.translate = lambda x: x

    # Set target points in leg frames
    legs_positions = np.array([[args.leg_x, args.leg_y, args.leg_z] for _ in range(6)])
    body_position = np.array([args.body_x, args.body_y, args.body_z])
    body_orientation = np.array([np.deg2rad(args.roll), np.deg2rad(args.pitch), np.deg2rad(args.yaw)])

    print(f'\nBody position:\n{np.round(body_position, 2)}')
    print(f'\nBody orientation:\n{np.round(body_orientation, 2)}')
    print(f'\nLeg positions (leg frame):\n{np.round(legs_positions, 2)}')

    joint_angles = hexapod.inverse_kinematics_leg_frame(
        legs_positions,
        body_position,
        body_orientation
    )
    print(f'\nJoint angles:\n{np.round(joint_angles, 2)}')

    # Draw the hexapod
    hexapod.joint_values = joint_angles  # We put the joint values inside just to draw it
    draw(hexapod, body_position=body_position, body_orientation=body_orientation)

    computed_legs_positions = hexapod.forward_kinematics(joint_angles, body_position, body_orientation)
    print(f'\nForward kinematics:\n{np.round(computed_legs_positions, 2)}')

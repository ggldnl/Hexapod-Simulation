import plotly.graph_objects as go
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

class Leg:

    def __init__(self, config, frame):
        """
        Initialize the Leg class.
        """

        self.config = config
        self.frame = frame

        # Joint angles [coxa_angle, femur_angle, tibia_angle] (in radians)
        self.joint_angles = [0.0, 0.0, 0.0]

    def __str__(self):
        return (f'Leg ( '
                f'coxa={self.config["coxa"]["length"]}, '
                f'femur={self.config["femur"]["length"]}, '
                f'tibia={self.config["tibia"]["length"]} | '
                f'alpha={round(self.joint_angles[0], 2)}, '
                f'beta={round(self.joint_angles[1], 2)}, '
                f'gamma={round(self.joint_angles[2], 2)})'
            )

    def inverse_kinematics(self, target):
        """
        Compute the joint angles to reach the target point (expressed in the leg's reference frame)
        """
        x, y, z = target

        c = self.config['coxa']['length']
        f = self.config['femur']['length']
        t = self.config['tibia']['length']

        # Coxa angle
        alpha = np.arctan2(y, x)
        r = np.sqrt(x ** 2 + y ** 2) - c

        # Compute 2D planar IK for femur and tibia
        d = np.sqrt(r ** 2 + z ** 2)  # Distance to the target
        if d > (f + t):
            raise ValueError("Target is out of reach!")

        cos_angle2 = (f ** 2 + t ** 2 - d ** 2) / (2 * f * t)
        gamma = np.arccos(cos_angle2) - np.pi

        cos_angle1 = (f ** 2 + d ** 2 - t ** 2) / (2 * f * d)
        beta = np.arctan2(z, r) + np.arccos(cos_angle1)

        self.joint_angles = [alpha, beta, gamma]

    def forward_kinematics(self):
        """
        Computes the position of the end effector in the leg's reference frame.
        """

        c = self.config['coxa']['length']
        f = self.config['femur']['length']
        t = self.config['tibia']['length']

        alpha, beta, gamma = self.joint_angles

        coxa_x = c * np.cos(alpha)
        coxa_y = c * np.sin(alpha)
        coxa_z = 0

        femur_x = coxa_x + f * np.cos(alpha) * np.cos(beta)
        femur_y = coxa_y + f * np.sin(alpha) * np.cos(beta)
        femur_z = coxa_z + f * np.sin(beta)

        tibia_x = femur_x + t * np.cos(alpha) * np.cos(beta + gamma)
        tibia_y = femur_y + t * np.sin(alpha) * np.cos(beta + gamma)
        tibia_z = femur_z + t * np.sin(beta + gamma)

        return tibia_x, tibia_y, tibia_z

    def plot(self):
        """
        Plot the leg using Plotly.
        """

        c = self.config['coxa']['length']
        f = self.config['femur']['length']
        t = self.config['tibia']['length']

        alpha, beta, gamma = self.joint_angles

        origin_x = 0
        origin_y = 0
        origin_z = 0

        coxa_x = c * np.cos(alpha)
        coxa_y = c * np.sin(alpha)
        coxa_z = 0

        femur_x = coxa_x + f * np.cos(alpha) * np.cos(beta)
        femur_y = coxa_y + f * np.sin(alpha) * np.cos(beta)
        femur_z = coxa_z + f * np.sin(beta)

        tibia_x = femur_x + t * np.cos(alpha) * np.cos(beta + gamma)
        tibia_y = femur_y + t * np.sin(alpha) * np.cos(beta + gamma)
        tibia_z = femur_z + t * np.sin(beta + gamma)

        x_coords = [origin_x, coxa_x, femur_x, tibia_x]
        y_coords = [origin_y, coxa_y, femur_y, tibia_y]
        z_coords = [origin_z, coxa_z, femur_z, tibia_z]

        # Create 3D scatter and line plot
        fig = go.Figure()

        # Add leg segments
        fig.add_trace(go.Scatter3d(
            x=x_coords, y=y_coords, z=z_coords,
            mode='lines+markers',
            marker=dict(size=6, color='blue'),
            line=dict(color='red', width=4),
            name='Hexapod Leg'
        ))

        # Set equal aspect ratio
        fig.update_layout(
            title="Hexapod Leg with Joint Angles",
            scene=dict(
                xaxis_title="X-axis",
                yaxis_title="Y-axis",
                zaxis_title="Z-axis",
                aspectmode="cube",
                xaxis=dict(range=[-100, 100]),
                yaxis=dict(range=[-100, 100]),
                zaxis=dict(range=[-100, 100]),
            ),
            margin=dict(l=0, r=0, b=0, t=40),
        )

        fig.show()


class Hexapod:

    def __init__(self, config, silent_fail=True):
        self.config = config
        self.silent_fail = silent_fail

        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0]
        self.body_frame = transformation_matrix(rotation_matrix(self.orientation), self.position)

        self.legs = []
        for leg in config:

            x_off = config[leg]['T']['x']
            y_off = config[leg]['T']['y']
            z_off = config[leg]['T']['z']
            position = [x_off, y_off, z_off]

            roll = config[leg]['T']['roll']
            pitch = config[leg]['T']['pitch']
            yaw = config[leg]['T']['yaw']
            orientation = [roll, pitch, yaw]

            leg_frame = transformation_matrix(rotation_matrix(orientation), position)
            self.legs.append(Leg(config[leg], leg_frame))

    def __iter__(self):
        return iter(self.legs)

    def set_body_pose(self, position, orientation):
        """
        Set a new body pose and update the legs such that they keep the
        end effector in the same position as before.
        """

        new_body_frame = transformation_matrix(rotation_matrix(orientation), position)

        # Compute the new joint values to reach the same point
        for leg in self.legs:

            # Compute the current end-effector position in the global frame
            ee_position_global = self.body_frame @ leg.frame @ np.array([*leg.forward_kinematics(), 1])

            # Update the leg's frame based on the new body frame
            leg.frame = new_body_frame @ leg.frame

            # Transform the end effector to the new leg frame
            ee_position_leg_frame = np.linalg.inv(self.body_frame) @ np.linalg.inv(leg.frame) @ ee_position_global
            ee_position_leg_frame = ee_position_leg_frame[:3]

            # Compute the new joint angles
            leg.inverse_kinematics(ee_position_leg_frame)

        # Update body pose
        self.position = position
        self.orientation = orientation
        self.body_frame = new_body_frame

    def forward_kinematics(self):
        """
        Returns the current end effector positions expressed in the respective leg frame.
        """

        leg_ee_positions = []
        for leg in self.legs:
            leg_ee_positions.append(leg.forward_kinematics())

        return leg_ee_positions

    def inverse_kinematics(self, targets):
        """
        Compute the joint angles, for each leg, to reach the target point.
        This keeps the body pose unchanged.
        """

        for leg, target in zip(self.legs, targets):
            leg.inverse_kinematics(target)

    def plot(self, additional_geometries=None):
        """
        Plot the Hexapod body and legs using Plotly.
        """

        fig = go.Figure()

        if additional_geometries is None:
            additional_geometries = []

        # Plot the body attachment points
        body_points = np.array([leg.frame[:3, 3] for leg in self.legs])
        body_points = np.vstack((body_points, body_points[0]))  # Close the loop

        fig.add_trace(go.Scatter3d(
            x=body_points[:, 0], y=body_points[:, 1], z=body_points[:, 2],
            mode='lines+markers',
            name='Body',
            marker=dict(size=5, color='blue'),
            line=dict(width=4, color='blue')
        ))

        # Plot each leg
        for i, leg in enumerate(self.legs):

            # Position of the attachment point in world frame
            attachment_p = leg.frame[:3, 3]

            c = leg.config['coxa']['length']
            f = leg.config['femur']['length']
            t = leg.config['tibia']['length']

            alpha, beta, gamma = leg.joint_angles

            coxa_x = c * np.cos(alpha)
            coxa_y = c * np.sin(alpha)
            coxa_z = 0
            coxa_p = np.array([coxa_x, coxa_y, coxa_z, 1])
            coxa_p = leg.frame @ coxa_p

            femur_x = coxa_x + f * np.cos(alpha) * np.cos(beta)
            femur_y = coxa_y + f * np.sin(alpha) * np.cos(beta)
            femur_z = coxa_z + f * np.sin(beta)
            femur_p = np.array([femur_x, femur_y, femur_z, 1])
            femur_p = leg.frame @ femur_p

            tibia_x = femur_x + t * np.cos(alpha) * np.cos(beta + gamma)
            tibia_y = femur_y + t * np.sin(alpha) * np.cos(beta + gamma)
            tibia_z = femur_z + t * np.sin(beta + gamma)
            tibia_p = np.array([tibia_x, tibia_y, tibia_z, 1])
            tibia_p = leg.frame @ tibia_p

            # Position of the coxa endpoint in world frame
            fig.add_trace(go.Scatter3d(
                x=[attachment_p[0], coxa_p[0], femur_p[0], tibia_p[0]],
                y=[attachment_p[1], coxa_p[1], femur_p[1], tibia_p[1]],
                z=[attachment_p[2], coxa_p[2], femur_p[2], tibia_p[2]],
                mode='lines+markers',
                name=f'leg_{i}',
                marker=dict(size=4, color='red'),
                line=dict(width=3, color='red')
            ))

        # Plot additional geometries
        for geom in additional_geometries:
            if isinstance(geom, np.ndarray):
                if geom.shape[1] == 3:  # Assume 3D points
                    fig.add_trace(go.Scatter3d(
                        x=geom[:, 0],
                        y=geom[:, 1],
                        z=geom[:, 2],
                        mode='markers',
                        name='Additional Points',
                        marker=dict(size=5, color='green')
                    ))
                elif geom.shape[0] == 2 and geom.shape[1] == 3:  # Line represented by two points
                    fig.add_trace(go.Scatter3d(
                        x=geom[:, 0],
                        y=geom[:, 1],
                        z=geom[:, 2],
                        mode='lines',
                        name='Additional Line',
                        line=dict(width=2, color='purple')
                    ))
                elif geom.ndim == 3 and geom.shape[2] == 3:  # Multiple lines
                    for line in geom:
                        fig.add_trace(go.Scatter3d(
                            x=line[:, 0],
                            y=line[:, 1],
                            z=line[:, 2],
                            mode='lines',
                            name='Additional Lines',
                            line=dict(width=2, color='purple')
                        ))
                else:
                    raise ValueError(f"Unsupported geometry shape: {geom.shape}")
            else:
                raise TypeError(f"Unsupported geometry type: {type(geom)}")

        # Set plot layout
        fig.update_layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z',
                aspectmode='data',
            ),
            title='Hexapod Visualization',
        )

        # Show the plot
        fig.show()


if __name__ == '__main__':

    import json
    from pathlib import Path

    # Read the config
    path = Path('hexapod.json')
    with open(path) as f:
        config = json.load(f)

    # Create a Hexapod object
    name = 'hexapod'
    hexapod = Hexapod(config[name])

    # Set the target points each leg has to reach (in leg frames)
    targets = [
        [100, 0, -50],
        [100, 0, -50],
        [100, 0, -50],
        [100, 0, -50],
        [100, 0, -50],
        [100, 0, -50],
    ]
    hexapod.inverse_kinematics(targets)
    hexapod.plot()

    # Change body pose
    hexapod.set_body_pose(
        [0, 0, 0],
        [np.deg2rad(10), np.deg2rad(10), np.deg2rad(10)]
    )

    # Plot the robot
    hexapod.plot()

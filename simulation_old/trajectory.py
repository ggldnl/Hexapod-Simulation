import matplotlib.pyplot as plt
import numpy as np
import argparse


def generate_trajectory(A, B, duty_cycle, arc_height, array_dim=100):
    """
    Generate a 3D trajectory for the leg's end effector based on the given parameters
    in cartesian space. This only serves to show how the gait will work. The controller
    will interpolate in joint space and only the two points (and relative IK) will be
    computed at start.

    Parameters:
        A (tuple): Start point (x, y, z) of the trajectory.
        B (tuple): End point (x, y, z) of the trajectory.
        duty_cycle (float): Fraction of time for the stance phase (0 to 1).
        arc_height (float): Maximum height of the arc during the swing phase.
        array_dim (int): Total number of points for the trajectory.

    Returns:
        np.ndarray: Array of (x, y, z) points for the trajectory.
    """
    assert 0 <= duty_cycle <= 1, "Duty cycle must be between 0 and 1"
    assert arc_height >= 0, "Arc height must be non-negative"

    # Split points for swing and stance phases
    swing_points = int((1 - duty_cycle) * array_dim)
    stance_points = array_dim - swing_points

    # Swing phase: Generate a parabolic arc in the z-axis
    swing_x = np.linspace(A[0], B[0], swing_points)
    swing_y = np.linspace(A[1], B[1], swing_points)
    swing_z = np.linspace(A[2], B[2], swing_points)
    swing_z += arc_height * (1 - (2 * (np.arange(swing_points) / swing_points - 0.5)) ** 2)

    # Stance phase: Straight line interpolation
    stance_x = np.linspace(B[0], A[0], stance_points)
    stance_y = np.linspace(B[1], A[1], stance_points)
    stance_z = np.linspace(B[2], A[2], stance_points)

    # Combine the phases
    x = np.concatenate((swing_x, stance_x))
    y = np.concatenate((swing_y, stance_y))
    z = np.concatenate((swing_z, stance_z))

    return np.column_stack((x, y, z))


def draw(trajectory, duty_cycle):
    """
    Plot the 3D trajectory with different colors for swing and stance phases.

    Parameters:
        trajectory (np.ndarray): 3D points for the trajectory.
        duty_cycle (float): Fraction of time for the stance phase.
    """
    array_dim = len(trajectory)
    swing_end = int((1 - duty_cycle) * array_dim)

    # Extract swing and stance trajectories
    swing_trajectory = trajectory[:swing_end]
    stance_trajectory = trajectory[swing_end:]

    # Plot the 3D trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot swing phase in red
    ax.plot(
        swing_trajectory[:, 0],
        swing_trajectory[:, 1],
        swing_trajectory[:, 2],
        color='red', label='Swing Phase'
    )

    # Plot stance phase in blue
    ax.plot(
        stance_trajectory[:, 0],
        stance_trajectory[:, 1],
        stance_trajectory[:, 2],
        color='blue', label='Stance Phase'
    )

    ax.set_title("3D Trajectory of End Effector")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Generate and visualize a sample leg trajectory.")
    parser.add_argument("-a", "--A", type=float, nargs=3, required=True, help="Start point (x, y, z) of the trajectory.")
    parser.add_argument("-b", "--B", type=float, nargs=3, required=True, help="End point (x, y, z) of the trajectory.")
    parser.add_argument("-c", "--duty_cycle", type=float, required=True, help="Fraction of time for the stance phase (0 to 1).")
    parser.add_argument("-h", "--arc_height", type=float, required=True, help="Maximum height of the arc during the swing phase.")
    parser.add_argument("-d", "--array_dim", type=int, default=100, help="Total number of points for the trajectory.")

    args = parser.parse_args()

    A = tuple(args.A)
    B = tuple(args.B)
    duty_cycle = args.duty_cycle
    arc_height = args.arc_height
    array_dim = args.array_dim

    trajectory = generate_trajectory(A, B, duty_cycle, arc_height, array_dim)
    draw(trajectory, duty_cycle)

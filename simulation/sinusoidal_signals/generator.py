import numpy as np


def control_signal(amplitude, phase, duty_cycle, offset, array_dim=100):
    """
    Create a smooth periodic function with amplitude, phase, and duty cycle
    """

    assert 0 <= amplitude <= 1
    assert 0 <= phase <= 1
    assert 0 <= duty_cycle <= 1
    assert -1 <= offset <= 1

    # Compute uptime based on the duty cycle
    up_time = int(array_dim * duty_cycle)

    # Create a square function
    temp = np.array([amplitude if i < up_time else -amplitude for i in range(array_dim)])

    # Apply kernel smoothing (kernel size is based 1/10 of the total size of the array)
    kernel_size = array_dim // 10
    sigma = kernel_size / 3

    # Create a gaussian kernel
    kernel = np.exp(-np.square(np.arange(-kernel_size, kernel_size + 1)) / (2 * sigma ** 2))
    kernel /= (sigma * np.sqrt(np.pi))
    kernel /= kernel.sum()

    # Apply kernel smoothing to the square function
    command = np.convolve(temp, kernel, mode='same')

    # Apply phase shift
    start = int(array_dim * phase)
    final_command = np.roll(command, -start)

    # Apply offset
    final_command += offset

    return final_command


class OpenLoopGaitGenerator:

    def __init__(self, params, array_dim=100):
        self.params = params
        self.array_dim = array_dim
        self.trajectories = self._compute_trajectories(params, array_dim)

    def step(self, dt):
        k = int(np.floor(dt * self.array_dim)) % self.array_dim
        return self.trajectories[:, :, k]

    @classmethod
    def _compute_trajectories(cls, params, array_dim):

        # Params has the following structure:
        # - one element for each leg
        # - for each leg, one element for each joint
        # - for each joint, one element for each parameter (3 i.e. amplitude, phase and duty_cycle)
        # We need to produce a single control input for each joint:
        # - one element for each leg
        # - for each leg, one control input per joint
        # - each control input is an array of size array_dim

        num_legs = len(params)
        joints_per_leg = len(params[0])

        trajectories = np.zeros((num_legs, joints_per_leg, array_dim))

        for leg in range(num_legs):
            for joint in range(joints_per_leg):
                trajectories[leg, joint] = control_signal(*params[leg][joint], array_dim)

        return trajectories


if __name__ == '__main__':

    from gaits import Gaits

    import plotly.graph_objects as go
    import argparse

    parser = argparse.ArgumentParser(description='Plot a control signal.')
    parser.add_argument('--array_dim', type=int, default=100, help='Array dimension')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step')
    parser.add_argument('--gait', type=str, default="TRI_GAIT", help='Gait type')
    args = parser.parse_args()

    gait = Gaits[args.gait]
    gait_params = gait.data
    print(f'Selected gait: {gait.label}')

    open_loop_gait_generator = OpenLoopGaitGenerator(gait_params, args.array_dim)
    trajectories = open_loop_gait_generator.trajectories

    leg = 0
    first_joint_trajectories = trajectories[leg][0]
    second_joint_trajectories = trajectories[leg][1]
    third_joint_trajectories = trajectories[leg][2]

    # Generate time steps
    time = np.linspace(0, 1, args.array_dim)

    # Create figure using Plotly
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time, y=first_joint_trajectories, mode='lines', name='First joint trajectories'))
    fig.add_trace(go.Scatter(x=time, y=second_joint_trajectories, mode='lines', name='Second joint trajectories'))
    fig.add_trace(go.Scatter(x=time, y=third_joint_trajectories, mode='lines', name='Third joint trajectories'))

    fig.update_layout(
        title='Generated Control Signal',
        xaxis_title='Time (normalized)',
        yaxis_title='Amplitude'
    )

    fig.show()

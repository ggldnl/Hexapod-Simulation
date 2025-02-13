import numpy as np


def control_signal(amplitude, phase, duty_cycle, scale, array_dim=100):
    """
    Create a smooth periodic function with amplitude, phase, and duty cycle.
    """

    assert 0 <= amplitude <= 1
    assert 0 <= phase <= 1
    assert 0 <= duty_cycle <= 1

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

    # Scale the signal
    final_command *= scale

    return final_command


if __name__ == '__main__':

    import plotly.graph_objects as go
    import argparse

    parser = argparse.ArgumentParser(description='Plot a control signal.')
    parser.add_argument('--array_dim', type=int, default=100, help='Array dimension')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step')
    parser.add_argument('--amplitude', type=float, default=0.8, help='Signal amplitude')
    parser.add_argument('--phase', type=float, default=0.2, help='Phase shift')
    parser.add_argument('--duty_cycle', type=float, default=0.5, help='Duty cycle')
    args = parser.parse_args()

    trajectories = control_signal(args.amplitude, args.phase, args.duty_cycle, args.array_dim)

    # Time steps
    time = np.linspace(0, 1, args.array_dim)

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time, y=trajectories, mode='lines', name='Control Signal'))

    fig.update_layout(
        title='Generated Control Signal',
        xaxis_title='Time (normalized)',
        yaxis_title='Amplitude'
    )

    fig.show()

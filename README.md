# Hexapod Simulation

This repository includes the robot's URDF and python code to simulate the gait of the hexapod in a PyBullet environment. 

For a complete overview of the project, refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/ggldnl/Hexapod-Simulation.git
    ```
2. Create a conda environment:
    ```bash
    mamba env create -f environment.yml
    mamba activate hexapod
    ```

## Usage

1. Run the simulation script:
   ```bash
   python pybullet/simulation.py
   ```

## TODO

- [ ] Add documentation on how to interpret simulation results.
- [ ] Provide videos of the simulation.

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

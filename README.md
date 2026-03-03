# Hexapod Simulation

This repository includes the code to simulate the Hexapod. 
It includes code to visualize and interact with the robot in Viser and code to simulate it in a PyBullet environment. 

<table>
  <tr>
    <td><img src="media/tripod.gif" alt="Tripod gait"></td>
    <td><img src="media/wave.gif" alt="Wave gait"></td>
  </tr>
  <tr>
    <td><img src="media/look_around.gif" alt="Looking around"></td>
    <td><img src="media/ripple.gif" alt="Ripple gait"></td>
  </tr>
</table>

For a complete overview of the project, refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

## 🛠️ Setup

Before you start, ensure `mamba` is properly installed on the machine you are using for the simulation.

1. Clone the repository:

   ```bash
    git clone --recursive https://github.com/ggldnl/Hexapod-Simulation.git
    ```
   
   The [Hexapod-Controller](https://github.com/ggldnl/Hexapod-Controller.git) and [Hexapod-Hardware]((https://github.com/ggldnl/Hexapod-Hardware.git)) repositories are configured as submodules and are automatically downloaded. If for some reason you need to update the submodules in the future:

    ```bash
    # Update the submodules if something changes in the future
    git submodule update --recursive --remote
    ```

2. Create a mamba environment:

    ```bash
    cd Hexapod-Simulation
    mamba env create -f environment.yml
    mamba activate hexapod-sim
    ```
   
   The Hexapod-Controller code will be automatically installed as a package. After this, from within the `hexapod-sim` mamba environment, you will be able to do something like this:

   ```python
   from controller import HexapodController
   ```
   
## 🚀 Delpoy

- `viser/main.py` will open a Viser-based Hexapod demo, showing the Hexapod moving forward and performing some adjustments along the way (body height, yaw, speed, ...).  

   ```bash
   python simulation/viser/main.py
   ```

- `bullet/main.py` will open a PyBullet Hexapod simulation, showing how the robot behaves when physics is involved. 
I used the actual stall torque the servos are rated for to simulate the motors.

   ```bash
   python simulation/bullet/main.py
   ```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.

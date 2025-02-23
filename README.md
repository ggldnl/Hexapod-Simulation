# Hexapod Simulation

This repository includes the code to simulate the hexapod in a PyBullet environment. 

For a complete overview of the project, refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

## 🛠️ Build and deployment

1. Clone the repository:

   ```bash
    git clone --recursive https://github.com/ggldnl/Hexapod-Simulation.git
    ```
2. Create a conda environment:

    ```bash
    mamba env create -f environment.yml
    mamba activate hexapod-sim
    ```

## 🚀 Delpoy

- `showcase_animations.py` will show the hexapod performing a set of predefined actions.

   ```bash
   python simulation/showcase_animations.py
   ```
  
    [![actions](https://img.youtube.com/vi/msuydRaIWuU/0.jpg)](https://www.youtube.com/watch?v=msuydRaIWuU)

- `showcase_gaits.py` will show the hexapod move with different gait strategies.

   ```bash
   python simulation/showcase_gaits.py
   ```

- `showcase_sinusoidal_signal_gaits.py` will show the hexapod move with open-loop gaits based on sinusoidal signals as described in the paper "Robots that can adapt like animals" by Cully et al., Nature, 2015.
  In this approach, each joint is controlled by a periodic signal defined by:
  - amplitude (range of motion);
  - phase (relative timing of the movement in a period);
  - duty cycle (proportion of time de movement lasts);

  Since no high-level controller is involved, I set the range of motion of each joint to [-90, 90] degs. The signals are centered at 0 and have values in range [-1, 1] and they get mapped to the joint range scaled by the amplitude. There is no way of changing the center of the signal. This is a problem if we need the output joint values to have a different range (e.g. legs not parallel -> coxas will need different ranges to produce coordinated motion). For this reason I introduced another parameter for the signal generation:
  - vertical shift (center of the signal);

  In my opinion, while there are undoubtedly advantages to using this approach (as described in the paper), this control strategy is difficult to use and adapt and I think other approaches might be worth exploring.

   ```bash
     python simulation/showcase_sinusoidal_signal_gaits.py
   ```
  
  [![actions](https://img.youtube.com/vi/W5BeiLtARxg/0.jpg)](https://www.youtube.com/watch?v=W5BeiLtARxg)

  Note: motion feels a little jerky but this is definitely my fault and can probably be solved by better tuning the gait parameters. I have no plan to continue working on this kind of gait strategy. 

You can even run the `hexapod.py` file to plot the kinematic structure of the robot.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.

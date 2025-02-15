# Hexapod Simulation

This repository includes the code to simulate the hexapod in a PyBullet environment. 

For a complete overview of the project, refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

## 🛠️ Build and deployment

1. Clone the repository:

   ```bash
    git clone https://github.com/ggldnl/Hexapod-Simulation.git
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

- `showcase_sinusoidal_signal_gaits.py` will show the hexapod move with open-loop gaits based on sinusoidal signals as described in the paper "Robots that can adapt like animals" by Cully et al., Nature, 2015.
  In this approach, each joint is controlled by a periodic signal defined by:
  - amplitude (range of motion);
  - phase (relative timing of the movement in a period);
  - duty cycle (proportion of time de movement lasts);
  Since no high-level controller is involved, I supposed the range of motion of each joint is [-90, 90] degs. In this formulation, each signal is centered at 0. This means the signal in range [-1, 1] is mapped to [-90, 90] with no way of changing the center of the interval. This is a problem if we need the output joint value to have a different range (e.g. legs not parallel -> coxas will need different ranges to produce coordinated motion). For this reason I introduced another parameter for the signal generation:
  - vertical shift (center of the signal);

  In my opinion, while there are undoubtedly advantages to using this approach (as described in the paper), this control strategy is difficult to use and adapt and I think other approaches might worth exploring.

   ```bash
     python simulation/showcase_sinusoidal_signal_gaits.py
   ```
  
  [![actions](https://img.youtube.com/vi/W5BeiLtARxg/0.jpg)](https://www.youtube.com/watch?v=W5BeiLtARxg)

  Note: motion feels a little jerky but this is definitely my fault and can probably be solved by better tuning the gait parameters. I have no plan to continue working on this kind of gait strategy. 

- `showcase_genetics.py` will show the hexapod learn a gait strategy with a genetic algorithm;

   ```bash
   python simulation/showcase_genetics.py
   ```

You can even run the `hexapod.py` file to plot the kinematic structure of the robot.

## TODO

- [ ] Develop genetic algorithm to learn gait patterns.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.

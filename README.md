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

- `showcase_animations.py` will show the hexapod performing a set of predefined actions;

   ```bash
   python simulation/showcase_animations.py
   ```
  
    [![actions](https://img.youtube.com/vi/msuydRaIWuU/0.jpg)](https://www.youtube.com/watch?v=msuydRaIWuU)

- `showcase_gaits.py` will show the hexapod move with different predefined open-loop gaits (e.g. tri-gait, ripple-gait, ...);

   ```bash
   python simulation/showcase_gaits.py
   ```

- `showcase_genetics.py` will show the hexapod learn a gait strategy with a genetic algorithm;

   ```bash
   python simulation/showcase_genetics.py
   ```

You can even run the `hexapod.py` file to plot the kinematic structure of the robot.

## TODO

- [ ] Develop genetic algorithm to learn gait patterns.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.

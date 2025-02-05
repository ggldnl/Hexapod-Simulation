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

Run the main simulation script; this will just show the hexapod performing a set of predefined actions:

```bash
python simulation/main.py
```

## TODO

- [ ] Develop genetic algorithm to learn gait patterns.
- [ ] Provide a way to select gait strategy from terminal.
- [ ] Provide videos of the simulation.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod).

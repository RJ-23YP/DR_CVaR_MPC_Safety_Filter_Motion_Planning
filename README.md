# Distributionally Robust Conditional Value at Risk-based Safety Filtering for Motion Planning in Uncertain Environments

This repository has been forked from the original implementation of the ICRA 2024 research paper titled "Distributionally Robust CVaR-Based Safety Filtering for Motion Planning in Uncertain Environments." The goal is to create a custom implementation and compare it's results against the official implementation. 

Link to the paper: https://arxiv.org/pdf/2309.08821

Link to the original implementation of the paper: https://github.com/TSummersLab/dr-cvar-safety_filtering

## Installation

The instructions to install the conda environment and the relevant packages are provided in the original repo. They can be easily reproduced for any OS provided that Conda is already installed.  

## My Implementation
I have created the 'my_backend' folder to start with my custom implementation of the paper. These are the files I have created and their purpose:
- dynamics.py: Implements 2D double and single integrator models for ego and obstacle dynamics.
- ref_traj_generation.py: Generates obstacle-free reference trajectories using MPC to reach goal states.
- safety_filter.py: Implements an MPC-based safety filter that enforces DR-CVaR safe halfspace constraints.

## Running the Scripts
The following scripts are useful for running the experiments:
- `drone_simulations.py`: runs the simulation
- `experiment_setup.py`: setup for experiment scenarios (intersection, head-on, multi-obstacles, ...)
- `multi_exp_sim.py`: runs Monte Carlo simulations and plots the results
- `paper_figures.py`: generates the halfspace comparison plots and the halfspace compute time plots


### Running simulations and generating experiment data:
- To plot paper supporting figures (non-experiment figures), run `paper_figures.py`.
- To set up a new type of experiment, edit the `experiment_setup.py` file
- To run a single simulation, edit the setting of the `reach_avoid` function in `drone_simulations.py` and run the script
- To run Monte carlo simulations, edit the toggles and ranges at the top of `multi_exp_sim.py` and run the script
- To plot experiment data
  - For single experiments, set the plotting variables in `drone_simulations.py` to `True`
  - For multiple experiments, set the plotting variable in `multi_exp_sim.py` to `True`

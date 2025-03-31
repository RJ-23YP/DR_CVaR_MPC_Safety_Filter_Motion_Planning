# Distributionally Robust Conditional Value at Risk 

This repository has been forked from the original implementation of the ICRA 2024 research paper titled "Distributionally Robust CVaR-Based Safety Filtering for Motion Planning in Uncertain Environments." The goal is to create a custom implementation and compare it's results against the official implementation. 

Link to the Paper: https://arxiv.org/pdf/2309.08821

Link to the original implementation of the Paper: https://github.com/TSummersLab/dr-cvar-safety_filtering

## Installation

The instructions to install the conda environment and the relevant packages are provided in the original repo. They can be easily reproduced for any OS provided that Conda is already installed.  

## Architecture
We highlight the following files in `backend`:
- `dynamic_vehicles.py`: defines classes for dynamic vehicles; i.e. vehicles with both geometry and a dynamics models
- `dynamics.py`: defines classes for the vehicle dynamics
- `geometry_vehicles.py`: defines classes for geometric vehicles
- `ref_traj_generation.py`: defines classes to generate reference trajectories for the ego vehicle
- `safe_halfspaces.py`: defines classes to obtain safe halfspaces
- `safety_filters.py`: defines classes for MPC-based safety filters
- `simulation_functions.py`: mostly includes plotting functions (and some to keep `drone_simulations.py` cleaner)

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

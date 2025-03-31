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

## Simulations and Experiments
I have used the following scripts to run some experiments to get an idea of the expected results:
- drone_simulations.py: Runs full 2D drone simulation experiments with DR-CVaR-based MPC safety filtering and plotting.
- experiment_setup.py: Defines multiple dynamic motion planning scenarios and sets up experiment configurations for ego and obstacle vehicles.

Once the Conda environment is activated we can run the following command to execute the experiments:

``python drone_simulations.py``


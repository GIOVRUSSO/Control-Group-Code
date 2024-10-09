# Data-driven architecture for the locomotion of a humanoid robot using A* and Crowdsourcing
# Building the project
The build procedure has been tested on Ubuntu 22.04.1 LTS. 

## Install the system dependencies
The project depends on some system libraries to build the external dependencies. To install them, run the following command:
```bash 
sudo apt-get update && sudo apt-get install cmake libgl1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build zlib1g-dev clang-12
```

## Setup the conda environment
In order to easily install the python dependencies, a conda environment is provided in the `spec-file.txt` file. To create the conda environment, run the following command:
```bash
conda create --name mujoco --file spec-file.txt
```
where `<env_name>` is the name of the conda environment you want to create. To activate the environment, run:
```bash
conda activate mujoco
```

## Install the external dependencies
To install the external dependencies, run the following command inside the conda environment you just created:
```bash
python3 setup.py install
```
This will download, build and install all of the required dependencies.

# Running the project
To run the crowdsourcing in a test environment, simply run the main python module:
```bash
python3 -m data_driven_legged_locomotion
```
We provide a fully modular architecture for defining a new environment as a collection of obstacles, however for the time being changes in the environment should be made by manually editing the  `__main__.py` file. We will provide a more user-friendly way to define the environment in the future.

# Project folders
- `data_driven_legged_locomotion/`: The main python package.
- `data_driven_legged_locomotion/common/`: The common classes and functions used throughout the project. These include generic classes for the crowdsourcing logic, the services, probability distributions, state spaces, etc.
- `data_driven_legged_locomotion/agents/`: The agents that can be used to solve the task, as well as the services that provide the behaviors to the crowdsourcing algorithm.
- `data_driven_legged_locomotion/envs/`: The environments for the crowdsourcing task.
- `data_driven_legged_locomotion/maps/`: The maps from which a cost function is generated from a given environment.
- `data_driven_legged_locomotion/utils/`: Utility functions and classes used throughout the project but not directly related to the crowdsourcing task.

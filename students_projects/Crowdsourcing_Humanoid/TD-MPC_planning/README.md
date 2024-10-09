# data-driven-legged-locomotion
# Building the project
The build procedure has been tested on Ubuntu 22.04.1 LTS. 

# Install dependencies
The dependencies are contained in the `requirements.txt` file. To install them, run:
```bash
pip3 install -r requirements.txt
```

# Running the project
To run the crowdsourcing in a test environment, simply run the main python module:
```bash
python3 -m data_driven_legged_locomotion
```
We provide a fully modular architecture for defining a new environment as a collection of obstacles, however for the time being changes in the environment should be made by manually editing the `__main__.py` file. We will provide a more user-friendly way to define the environment in the future.

# Project folders
- `data_driven_legged_locomotion/`: The main python package.
- `data_driven_legged_locomotion/common/`: The common classes and functions used throughout the project. These include generic classes for the crowdsourcing logic, the services, probability distributions, state spaces, etc.
- `data_driven_legged_locomotion/agents/`: The agents that can be used to solve the task, as well as the services that provide the behaviours to the crowdsourcing algorithm.
- `data_driven_legged_locomotion/envs/`: The environments for the crowdsourcing task.
- `data_driven_legged_locomotion/utils/`: Utility functions and classes used throughout the project but not directly related to the crowdsourcing task.

# Acknowledgements
This project makes use of the code from TD-MPC2 repository. 
# Robust Decision-Making Via Free Energy Minimization
### Introduction
This repository collects the supporting code for the manuscript **Robust Decision-Making Via Free Energy Minimization**. The manuscript discusses the challenge of ensuring the robust performance of autonomous agents amidst environmental and training ambiguities. We introduce DR-FREE, an energy-based computational model that embeds robustness directly into decision-making via free energy minimization. By leveraging a distributionally robust extension of the free energy principle, DR-FREE yields policies that are both optimal and resilient against uncertainty, as demonstrated through real rover experiments.
### Contents
The following list of directories can be found in the repository, reproducing the simulation and experimental results in the manuscript.
- Experiments:
  - The folder contains our DRFREE implementation for the Robotarium experiments.
  - The folder also contains:
    - Code for ambiguity unaware agent.
    - Gaussian Process (GP) models and the code to train GP models.
    - Code to reconstruct the cost using trajectory data.  
- Belief Update Benchmark:
  - This folder contains the code files required to replicate the belief update benchmarking results. 
- Assets
  - contains all the plots of the manuscript, the data from the experiments used to generate these plots, and the Robotarium movie. 

### Results
We present the simulation and experimental results given in the manuscript.

*Robotarium:*
- In-silico results:

  ![Screenshot 2025-02-28 105328](https://github.com/user-attachments/assets/deea8cb1-bfe5-4427-a46b-b7f25b9a1af4)

Figure. At every training stage, we compare DR-FREE with a free-energy minimizing agent that, while making optimal decisions, does not account for ambiguity. With identical starting positions across experiments, DR-FREE consistently guides the robot to complete its task, whereas the ambiguity-unaware agent fails.

  ![Screenshot 2025-02-28 140146](https://github.com/user-attachments/assets/41696956-ca99-4943-b0fe-c6c7371c90e8)

Figure. (left panel) The nonconvex state cost for the navigation task. (right panel) Reconstructed cost using the belief updating algorithm.

- Experimental results:

The following videos show robotarium robot performing the task:
  - When the control policy is obtained using DR-FREE Algoritm.

https://github.com/user-attachments/assets/41724483-9ade-4a6b-89b8-fd4c7bf963fa




### Authors and Contributors 
Author of the code and simulations: *Hozefa Jesawada* (hjesawada@unisa.it)

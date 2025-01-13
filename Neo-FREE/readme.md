# Neo-FREE: Policy Composition Through Thousand Brains And Free Energy Optimization

This repository contains the Python code developed for the proceeding ***Neo-FREE: Policy Composition Through Thousand Brains And Free Energy Optimization***.

The application involves the navigation of a unicycle robot from the Robotarium [[1]](#1) in an environment with obstacles. Specifically:
* The folder *Robotarium/simulations* contains the code for the simulated environment. The outputs are:
  - The heat map of the cost function used in the simulations:

    ![robot_cost_map](https://github.com/user-attachments/assets/2d4f1bce-e916-41cd-b80e-b71b62410942)

  - The trajectories of the robot controlled by Neo-FREE:

    ![trajectories](https://github.com/user-attachments/assets/bae2c803-e801-40c5-9951-c99d23500778)

* The folder *Robotarium/real_experiment* contains the code developed for the real-hardware experiment, which can be reproduced by uploading the file on the Robotarium webpage. The resulting video is shown here:

  https://github.com/user-attachments/assets/cb9e6789-ee19-4578-86c9-79c5fd861c47



The Python simulator for Robotarium is available at https://github.com/robotarium/robotarium_python_simulator.

The following dependencies are required for the simulations:
- NumPy (http://www.numpy.org)
- matplotlib (http://matplotlib.org/index.html)
- CVXPY (https://www.cvxpy.org)
- SciPy (https://scipy.org).




## References
<a id="1">[1]</a> 
S. Wilson et al., "The Robotarium: Globally Impactful Opportunities, Challenges, and Lessons Learned in Remote-Access, Distributed Control of Multirobot Systems," in IEEE Control Systems Magazine, vol. 40, no. 1, pp. 26-44, Feb. 2020

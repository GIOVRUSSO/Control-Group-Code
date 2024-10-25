# Rover Navigation in Unstructured Environment

## Overview

This project is focusing on the development of an autonomous rover navigation system for unstructured environments by exploiting Data Driven techniques. The system integrates several ROS-based tools and algorithms for localization, obstacle detection, path planning, and control. It leverages multiple sensors, dynamic models, and simulation environments using Gazebo.

### Key Features:
- **Autonomous Navigation**: Waypoint-based navigation with path adjustments in response to dynamic obstacles.
- **Sensor Fusion**: Localization using GPS, IMU, and LiDAR sensors, integrating data with an EKF-based fusion system. #TODO key files for this function.
- **Multi-Robot Simulation**: Integration of multiple Husky rovers in a Gazebo environment with real-time obstacle detection. #TODO key files for this features
- **Teleoperation**: Manual control interface for teleoperating the rover using a keyboard.
- **Extraction of passive dynamics from teleoperation data**: teleoperation facility can be used for generating trajectories which can be taken into account as example data or passive dynamics by the control algorithm for the planning phase.

## Project Structure


### Main Components: #TODO add brief description for move_base + official link document

1. **`config`**:
   - Contains configuration files for controlling the rover, managing localization, and planning navigation.

2. **`husky_description`**:
   - Husky rover description package where sensors are enabled and namespace and `tf_prefix` are configured.

3. **`launch`**:
   - Contains launch files to start different components:
     - **Dual EKF NavSat**: For GPS, IMU, and odometry data fusion with EKF.
     - **Move Base**: For local path planning and low-level control.
     - **State Manager Node**: Estimates the robot’s state (position and pose).
     - **Waypoint Follower Node**: Responsible for trajectory following. It represents the interface from the KLC planner and move_base.
     - **World files**: Launch different Gazebo environments (10 configurations).

4. **`multi_husky_rovers`**:
   - Files for integrating multiple Husky rovers in Gazebo with utility nodes for remapping topics.

5. **`rviz`**:
   - Configuration for RViz visualization.

6. **`plants`**:
   - Contains dynamic models used for controlling the rover, including:
     - **Linear Model**: Linear dynamic model of the rover.
     - **Uniform Plant**: For passive dynamics.
     - **Trajectory-Based Plant**: Extracts passive dynamics from recorded trajectories.

7. **`point_cloud_perception`**:
   - Implements point cloud processing:
     - **Clustering**: Groups point clouds to detect obstacles.
     - **Segmentation**: Separates the ground from obstacles.
     - **Voxel Filtering**: Reduces the resolution of the point cloud.

8. **Control Loops**:
   - **`control_loop.py`**: Main control loop for the KLC algorithm.
   - **`main_control_loop.py`**: KLC with receding horizon and MPC as low-level controller.
   - **`online_control_loop.py`**: Online KLC, replanning when significant changes in the environment occur.

9. **Controllers**:
   - **`klc_controller.py`**: Classical KLC controller.
   - **`klc_controller_online.py`**: KLC controller that plans up to two steps ahead.
   - **`mpc_controller_x_x.py`**: MPC controller files for 9 different environments.
   - **`fullKx_x.py`**: 9 files for loading waypoints generated through KLC and followed due to mpc_controller.

10. **Obstacle Management**:
    - **`obstacle_cluster_manager.py`**: Manages obstacle clusters detected by point cloud clustering.
    - **`obstacle_manager.py`**: Manages detected obstacles using LiDAR data, publishing obstacle centroids for RViz and tracking changes in the environment.

11. **State Estimation**:
    - **`state_manager_node.py`**: ROS node which rovides 2D position and full pose estimation.

12. **Utilities**:
    - **`utility.py`**: Contains utility functions used across the project.

13. **Simulation**:
    - **`worlds/`**: Contains `.world` and `.sdf` files for different Gazebo simulation environments.

14. **Trajectory and Dataset Management**:
    - **`data_combiner.ipynb`**: A notebook used to merge recorded trajectories into a dataset.
    - **`log_position_xxxxxxx_xxxxxx`**: Numpy arrays storing sample trajectories generated through teleoperation.
15. **Teleoperation user interface**
    - **`keyboard_husky.py`**: it allows to control the rover using the keyboard. In particular, you just need to press W,A,S,D respectively for going forawd, turning to left, going backward, turning right.
16. **Multi-robot integration**
    - **`multi_husky_rovers`**: folder where are contained launch files configured for spawning, localizing and controlling multiple Husky rovers in Gazebo. Each rover is equipped with GPS,IMU, LiDAR VLP-16 and odometry. All sensors are simulated but is easy to add gaussian noise (through urdf) in order to emulate more realistic scenarios.
17. **Performance evaluation**
    -**`evaluate_performance.py`**: aUse it to assess the performance of the control algorithms, focusing on trajectory tracking accuracy and obstacle avoidance efficiency.

## Setup Instructions


```markdown
# Setup Instructions for ROS Navigation Project

## Step 1: Install Python Packages for ROS
Open your terminal and run the following commands:

```bash
sudo apt install python3-catkin-tools python3-osrf-pycommon
sudo apt install python3-wstool
```

---

## Step 2: Install ROS-Related Packages
Execute the following commands to install necessary ROS packages:

1. Download and execute the installation script:
   ```bash
   wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
   chmod +x ./ros_install_noetic.sh
   ./ros_install_noetic.sh
   ```

2. Install the primary ROS packages:
   ```bash
   sudo apt install ros-$ROS_DISTRO-catkin python3-catkin-tools
   sudo apt install ros-noetic-moveit
   sudo apt install ros-noetic-rqt-controller-manager
   sudo apt install ros-noetic-rqt-joint-trajectory-controller
   sudo apt install ros-noetic-rqt-multiplot
   sudo apt install husky_gazebo
   sudo apt-get install ros-noetic-husky-simulator
   ```

---

## Step 3: Create Catkin Workspace
Set up your Catkin workspace:

1. Navigate to your home directory:
   ```bash
   cd ~/
   ```

2. Create the Catkin workspace and source directory:
   ```bash
   mkdir --parents catkin_ws/src
   ```

3. Move into the Catkin workspace:
   ```bash
   cd catkin_ws
   ```

4. Build the workspace:
   ```bash
   catkin_make
   ```

---

## Step 4: Run the Code
### Configuration: Preregisterd waypoints generated with KLC + MPC
To run the code with preregistered waypoints and mpc as low_level contoller, follow these steps:

1. Update the `ROS_PACKAGE_PATH`:
   ```bash
   export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH  # Change the path accordingly
   ```

2. Launch the desired world in Gazebo:
   ```bash
   roslaunch husky_mpc_datadriven world1_1.launch
   ```

3. Execute your main Python script:
   ```bash
   python3 fullK1_1.py
   ```
### Configuration: KLC + Move_base
To run the code with KLC as planner and move_base follow these steps:

1. Update the `ROS_PACKAGE_PATH`:
   ```bash
   export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH  # Change the path accordingly
   ```
2. Go in the file multi_husky_world.launch and optionally change the environment choosing between that available in the worlds folder

3. Go in the file multi_husky_nav.launch and optionally change the number of agents that you want in the environment.

4. Launch the desired world in Gazebo:
   ```bash
   roslaunch husky_mpc_datadriven multi_husky_world.launch
   ```

5. Execute your main Python script:
   ```bash
   python3 control_loop.py
   ```
### Configuration: KLC Online + Move_base
To run the code with KLC Online as planner and move_base follow these steps:

1. Update the `ROS_PACKAGE_PATH`:
   ```bash
   export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH  # Change the path accordingly
   ```
2. Go in the file multi_husky_world.launch and optionally change the environment choosing between that available in the worlds folder

3. Go in the file multi_husky_nav.launch and optionally change the number of agents that you want in the environment.

4. Launch the desired world in Gazebo:
   ```bash
   roslaunch husky_mpc_datadriven multi_husky_world.launch
   ```

5. Execute your main Python script:
   ```bash
   python3 online_control_loop.py
   ```

### Configuration: Teleoperation
To run the code with KLC Online as planner and move_base follow these steps:

1. Update the `ROS_PACKAGE_PATH`:
   ```bash
   export ROS_PACKAGE_PATH=/home/sx/catkin_ws/src/:$ROS_PACKAGE_PATH  # Change the path accordingly
   ```
2. Go in the file multi_husky_world.launch and optionally change the environment choosing between that available in the worlds folder

3. Go in the file multi_husky_nav.launch and optionally change the number of agents that you want in the environment.

4. Launch the desired world in Gazebo:
   ```bash
   roslaunch husky_mpc_datadriven multi_husky_world.launch
   ```
5. Open the file husky_keyboard.py and set the proper topic for sending velocity command to the rover.

6. Execute your main Python script:
   ```bash
   python3 keyboard_husky.py

---

## Step 5: Optional Record (Not Needed for Setup)
The following commands are not necessary for setup but are left here as a record:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Additional package installations (if needed):
```bash
sudo apt install python3-osrf-pycommon
sudo apt install ros-$ROS_DISTRO-moveit
sudo apt install ros-$ROS_DISTRO-rqt-controller-manager
sudo apt install ros-$ROS_DISTRO-rqt-joint-trajectory-controller
sudo apt install ros-$ROS_DISTRO-rqt-multiplot
```


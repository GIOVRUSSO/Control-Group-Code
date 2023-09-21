# ***Guide to install and execute the husky_mpc_datadriven*** 

### **Prerequisites**
To execute all the code and the simulations, hardware and not, you have to install on your PC the ***Ubuntu 20.04 LTS*** version.\
In any case, the algorithm works optimally with any simulator, just access the state information and you can implement the control modifing the ROS/Gazebo part.

## **Ros and Gazebo installation**
Following the guide below you will install the correct version of ROS and Gazebo for the Ubuntu version cited above.

1. wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x 
./ros_install_noetic.sh && ./ros_install_noetic.sh
 
2. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
3. source ~/.bashrc
 
4. sudo apt install ros-$ROS_DISTRO-catkin 
5. python3-catkin-tools python3-osrf-pycommon
6. sudo apt install python3-wstool
7. sudo apt install ros-$ROS_DISTRO-moveit
8. sudo apt install ros-$ROS_DISTRO-rqt-controller-manager 
9. ros-$ROS_DISTRO-rqt-joint-trajectory-controller 
10. ros-$ROS_DISTRO-rqt-multiplot


11. cd
12. mkdir -p ~/catkin_ws/src
13. cd ~/catkin_ws/
14. catkin config --extend /opt/ros/$ROS_DISTRO/ --cmake-args -DCMAKE_BUILD_TYPE=Release
15. catkin init
16. catkin build
 
17. echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
18. source ~/.bashrc
 
19. sudo apt-get install ros-<distro>-husky-simulator
20. export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
21. roslaunch husky_gazebo husky_empty_world.launch 
<hr>

## **Configuration of the code**
The configuration of the code is easy, follow the steps below.\
The controller main is contained in the **main_control_loop.py** file: 
1. Considering the figure, you can choose the controller changing the **import** in this two way:
    - from klc_controller import *
    - from klc_controller_online import *<br>
2. To change the plant you have to modify a flag, at **line 9**:
    - Plant **0** is the passive dynamics corresponding to the uniform distribution;
    - Plant **1** is the passive dynamics corresponding to the linearized data plant;
    - Plant **2** is the last passive dynamics corresponding to the Uniform + MPC executed trajectory.
3. The modifications have to be applied also a the **line 10** like this:
    - klc = ControllerKLC(target, mode)
    - klc = ControllerKLCOnline(target, mode)

![main_control_loop.py](img\change_controller.png) 
    
At the end of configuration it is also usefull to know the files related to the fine-tuning of the algorithms:
- for the KLC algotithm the designed file is **klc_controller** or **klc_controller_online**;
![main_control_loop.py](img\tuning_klc.png) 


- instead, for the MPC algorithm the file is **mpc_controller**
![main_control_loop.py](img\tuning_mpc.png) 
![main_control_loop.py](img\tuning_mpc_bounds.png) 
<hr>

## Execution of the code 
1. Open Gazebo simulator with your world: 
    ```bash
    roslaunch  husky_mpc_datadriven  collision_avoidance_real_husky.launch x:=-4 y:=-4 z:=0 yaw:=0
    ```
2. Open the **main_control_loop.py** class;
3. Choose the target for the control problem;
5. Start the **main_control_loop.py** with simple command:\
    ```bash
    python3 main_control_loop.py
    ```
6. Find the results into the src directory. 

### PS: The guide is also for the hardware branch version included into the repository, there aren't differences.
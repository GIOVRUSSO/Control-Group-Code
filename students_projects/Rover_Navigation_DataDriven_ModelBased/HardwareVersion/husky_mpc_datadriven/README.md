# Husky Mixed Control

## Preliminaries
If you want to run the developed work in the same way shown you need to install Ubuntu in version 20.04LTS for full compatibility with ROS.\
In any case, the algorithm works optimally with any simulator, just access the state information and you can implement the control.

## Execution of the code 
1. Open Gazebo simulator with your world: 
    ```bash
    roslaunch  husky_mpc_datadriven  collision_avoidance_real_husky.launch x:=-4 y:=-4 z:=0 yaw:=0
    ```
2. Open the **main_control_loop.py** class;
3. Choose one of the two version of the algorithm:\
    - ControllerKLC
    - ControllerKLCOnline
4. Choose the target for the control problem;
5. Start the **main_control_loop.py** with simple command:\
    ```bash
    python3 main_control_loop.py
    ```
6. Find the results into the src directory.
# Human-Robot Interaction with Force Feedback in Collaborative Tasks

Master thesis, M.Eng. Mechatronics and Cyber-physical Systems,
Deggendorf Institute of Technology, Campus Cham (2025).
Author: Sijin Mathew | Supervisor: Prof. Dr. Dmitrii Dobriborsci

## Overview
Simulation of a UR3e cobot with force/torque feedback, MoveIt2
motion planning, admittance control and proximity-based safety
(slow-down and stop zones) in Gazebo.

## Features
- UR3e + Robotiq 2F-140 gripper in Gazebo
- MoveIt2 planning (UR3e and gripper groups) in RViz2
- F/T sensor on `/wrench`, visualised in RViz2
- Admittance controller (adjustable damping and stiffness)
- Pick and place using IFRA LinkAttacher
- Human-proximity monitor (warning and stop zones)

## Requirements
- Ubuntu 22.04, ROS 2 Humble, Gazebo, MoveIt2

## Build
    cd ~/ur_ws
    colcon build --symlink-install
    source install/setup.bash

## Run
    ros2 launch ur_sim spawn_ur3e_gripper_moveit.launch.py

    python3 src/ur_sim/ur_sim/admittance_controller.py
    ros2 param set /admittance_controller damping 15.0
    ros2 param set /admittance_controller stiffness 50.0

## Third-party packages
Universal_Robots_ROS2_Description, Universal_Robots_ROS2_Gazebo_Simulation,
ros2_robotiq_gripper, IFRA_LinkAttacher (see their original licences).

## Limitations
Simulation only; not validated on a real UR3e.

## Licence
(e.g. MIT for your own code. The thesis text is not included.)

This directory contains the source code used during the diploma thesis: **Force Control and Human--Robot Interaction for the Franka Research 
3 Manipulator** The source code is divided into three main categories: - `src_ROS/`: ROS 2 based code, launch workflows, MoveIt-related 
files, and node-based experiments. - `src_gazebo/`: Gazebo / simulation-related files used for testing the robot before real hardware 
execution. - `src_non_ROS/`: standalone C++ programs that communicate directly with the real FR3 using `libfranka`, without ROS. This 
separation was used because the thesis involved different experimental environments: simulation, ROS 2 integration, and direct real-time
robot control.
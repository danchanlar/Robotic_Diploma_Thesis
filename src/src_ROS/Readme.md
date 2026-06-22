This folder contains the ROS 2 related source code and configuration files used in the thesis. The purpose of this folder is to organize the
 \parts of the project that depend on ROS 2, MoveIt 2, RViz, `ros2_control`, and Franka ROS 2 packages. Files in this folder are used for:
 - launching the FR3 robot model in ROS 2, 
- connecting the FR3 with MoveIt 2, 
- testing fake hardware before real robot execution, 
- visualizing the robot in RViz,
 - checking ROS 2 controllers and joint states, 
- preparing higher-level robot workflows. 
This code is mainly useful for visualization, planning, debugging, and integration with the ROS 2 ecosystem. The real-time force-control 
experiments were mainly executed through the non-ROS `libfranka` programs.
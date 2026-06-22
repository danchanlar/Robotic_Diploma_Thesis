This folder contains standalone C++ programs that communicate directly with the Franka Research 3 using `libfranka`, without ROS. The purpose of this folder is to implement and test low-level real-time control methods for the FR3. Files in this folder are used for: 
- direct connection to the real FR3 through the Franka Control Interface,
- reading the robot state, - reading external force and torque estimates,
- controlling the Franka Hand, - executing Cartesian motion,
- implementing impedance control,
- implementing admittance control,
- implementing hybrid force/position control,
- logging experimental data to CSV files.
  
This folder is the most important part of the repository for the real robot force-control experiments. Direct `libfranka` communication was used because it provides access to the 1 kHz control loop, robot model quantities, Jacobian, Coriolis terms, joint torques, and external wrench estimates. Before running any program from this folder on the real robot, the user must verify: - the Ethernet connection, - the Desk state, - FCI activation, - `echo_robot_state`, - emergency stop availability, - safe robot configuration.

This folder contains the Gazebo simulation files used during the thesis.

The purpose of this folder is to test the FR3 setup in a safe simulated environment before running experiments on the real robot.

Files in this folder are used for:

- Gazebo / Ignition Gazebo simulation,
- testing robot scenes and object placement,
- checking table, wall, or obstacle positions,
- validating approximate robot reachability,
- testing simulation workflows before real hardware execution,
- reducing the risk of unsafe behavior on the physical FR3.

Gazebo was used as an intermediate validation step. It does not perfectly reproduce real contact forces, friction, or sensor behavior, but it is useful for debugging geometry, launch files, and general motion workflows.

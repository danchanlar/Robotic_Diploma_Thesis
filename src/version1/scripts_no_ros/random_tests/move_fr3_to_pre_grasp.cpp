#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>

double distance3d(double x1, double y1, double z1,
                  double x2, double y2, double z2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dz = z2 - z1;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char** argv) {
  const char* robot_ip = "172.16.0.2";

  if (argc == 2) {
    robot_ip = argv[1];
  }

  try {
    franka::Robot robot(robot_ip);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::RobotState initial_state = robot.readOnce();

    std::array<double, 16> start_pose = initial_state.O_T_EE;
    std::array<double, 16> goal_pose = start_pose;

    // pre_grasp target from grasp_target.yaml
    const double target_x = 0.45;
    const double target_y = 0.15;
    const double target_z = 0.25;

    const double start_x = start_pose[12];
    const double start_y = start_pose[13];
    const double start_z = start_pose[14];

    const double dist = distance3d(start_x, start_y, start_z,
                                   target_x, target_y, target_z);

    std::cout << "Current EE position:\n";
    std::cout << "  x=" << start_x << " y=" << start_y << " z=" << start_z << "\n";

    std::cout << "Target pre_grasp position:\n";
    std::cout << "  x=" << target_x << " y=" << target_y << " z=" << target_z << "\n";

    std::cout << "Distance to target: " << dist << " m\n";

    // Safety check: do not allow a big unexpected Cartesian move.
    if (dist > 0.30) {
      std::cerr << "ABORT: target is more than 0.30 m away from current pose.\n";
      std::cerr << "Move the robot closer first or choose a nearer pre_grasp.\n";
      return 1;
    }

    if (target_z < 0.15) {
      std::cerr << "ABORT: target_z is too low for first pre_grasp test.\n";
      return 1;
    }

    goal_pose[12] = target_x;
    goal_pose[13] = target_y;
    goal_pose[14] = target_z;

    // Slow motion. Minimum 8 sec, longer if distance is larger.
    const double max_speed = 0.025;  // m/s
    const double duration = std::max(8.0, dist / max_speed);

    std::cout << "Moving to pre_grasp slowly.\n";
    std::cout << "Duration: " << duration << " seconds.\n";
    std::cout << "Orientation is kept the same as current gripper orientation.\n";
    std::cout << "Keep emergency stop ready.\n";

    double time = 0.0;

    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      double t = std::min(time / duration, 1.0);

      // Smooth cubic interpolation.
      double s = 3.0 * t * t - 2.0 * t * t * t;

      std::array<double, 16> commanded_pose = start_pose;

      // Interpolate only translation. Keep orientation from current pose.
      commanded_pose[12] = start_pose[12] + s * (goal_pose[12] - start_pose[12]);
      commanded_pose[13] = start_pose[13] + s * (goal_pose[13] - start_pose[13]);
      commanded_pose[14] = start_pose[14] + s * (goal_pose[14] - start_pose[14]);

      if (time >= duration) {
        std::cout << "Reached pre_grasp.\n";
        return franka::MotionFinished(franka::CartesianPose(goal_pose));
      }

      return franka::CartesianPose(commanded_pose);
    });

    std::cout << "Done.\n";
    return 0;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception: " << e.what() << "\n";
    return 1;
  }
}

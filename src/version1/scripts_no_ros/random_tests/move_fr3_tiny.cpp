#include <array>
#include <cmath>
#include <iostream>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
    return 1;
  }
  const char* robot_ip = argv[1];
  try {
    franka::Robot robot(robot_ip);
    // Conservative collision/contact thresholds.
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
    std::array<double, 7> q_start = initial_state.q;
    std::array<double, 7> q_goal = q_start;
    // Tiny safe movement: move only joint 7 by +0.02 rad.
    // 0.02 rad is about 1.15 degrees.
    q_goal[6] += 0.02;
    std::cout << "Starting tiny motion on joint 7 by +0.02 rad.\n";
    std::cout << "Keep emergency stop ready.\n";
    double time = 0.0;
    const double duration = 5.0;
    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      double t = std::min(time / duration, 1.0);
      // Smooth cubic interpolation: 0 -> 1.
      double s = 3.0 * t * t - 2.0 * t * t * t;
      std::array<double, 7> q_cmd{};
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] = q_start[i] + s * (q_goal[i] - q_start[i]);
      }
      if (time >= duration) {
        std::cout << "Motion finished.\n";
        return franka::MotionFinished(franka::JointPositions(q_goal));
      }
      return franka::JointPositions(q_cmd);
    });
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception: " << e.what() << "\n";
    return 1;
  }
}

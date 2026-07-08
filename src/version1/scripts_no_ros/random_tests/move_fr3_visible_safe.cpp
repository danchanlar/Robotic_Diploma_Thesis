#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
    return 1;
  }

  const char* robot_ip = argv[1];

  try {
    franka::Robot robot(robot_ip);

    // Sxetika syntiritika thresholds gia prwth dokimh.
    // An to robot akoumphsei kati h dei megales dynameis, tha kanei stop/reflex.
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

    // Pio emfanhs alla akoma mikrh kinhsh:
    // joint 7 +0.12 rad sto meso kai epistrofh sto arxiko.
    // 0.12 rad ~= 6.9 degrees.
    const double amplitude = 0.12;
    const double duration = 10.0;

    std::cout << "Starting visible safe motion.\n";
    std::cout << "Moving only joint 7 up to +0.12 rad and returning to start.\n";
    std::cout << "Duration: " << duration << " seconds.\n";
    std::cout << "Keep emergency stop ready.\n";

    double time = 0.0;

    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      double t = std::min(time / duration, 1.0);

      // Smooth out-and-back profile:
      // offset = 0 at start, +amplitude at middle, 0 at end.
      // This avoids abrupt start/stop.
      double offset = amplitude * std::pow(std::sin(M_PI * t), 2.0);

      std::array<double, 7> q_cmd = q_start;
      q_cmd[6] = q_start[6] + offset;

      if (time >= duration) {
        std::cout << "Motion finished. Returning to initial joint position.\n";
        return franka::MotionFinished(franka::JointPositions(q_start));
      }

      return franka::JointPositions(q_cmd);
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

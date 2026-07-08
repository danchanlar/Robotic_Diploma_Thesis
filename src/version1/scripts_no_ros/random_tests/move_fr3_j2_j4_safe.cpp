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

    // Conservative collision/contact thresholds.
    // An kati paei lathos h to robot akoumphsei kati, tha kanei stop/reflex.
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

    // Προσοχή:
    // Στον κώδικα οι πίνακες είναι 0-based:
    // q[0] = joint 1
    // q[1] = joint 2
    // q[2] = joint 3
    // q[3] = joint 4
    // ...
    //
    // Εδώ κινείται μόνο η άρθρωση 2 και η άρθρωση 4.
    const double joint2_amplitude = 0.07;   // περίπου 4 μοίρες
    const double joint4_amplitude = -0.10;  // περίπου -5.7 μοίρες

    const double duration = 12.0;  // αργή και ομαλή κίνηση

    std::cout << "Starting safe joint-space trajectory.\n";
    std::cout << "Moving joint 2 by +0.07 rad and joint 4 by -0.10 rad.\n";
    std::cout << "Then returning to the initial position.\n";
    std::cout << "Duration: " << duration << " seconds.\n";
    std::cout << "Keep emergency stop ready.\n";

    double time = 0.0;

    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      double t = std::min(time / duration, 1.0);

      // Smooth out-and-back profile:
      // 0 στην αρχή, 1 στη μέση, 0 στο τέλος.
      // Έτσι δεν έχουμε απότομη εκκίνηση/στάση.
      double s = std::pow(std::sin(M_PI * t), 2.0);

      std::array<double, 7> q_cmd = q_start;

      // joint 2
      q_cmd[1] = q_start[1] + joint2_amplitude * s;

      // joint 4
      q_cmd[3] = q_start[3] + joint4_amplitude * s;

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

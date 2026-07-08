#include <array>
#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

void printArray7(const std::string& name, const std::array<double, 7>& a) {
  std::cout << name << " = [ ";
  for (size_t i = 0; i < 7; i++) {
    std::cout << std::setw(9) << std::fixed << std::setprecision(4) << a[i];
    if (i < 6) std::cout << ", ";
  }
  std::cout << " ]" << std::endl;
}

void printArray6(const std::string& name, const std::array<double, 6>& a) {
  std::cout << name << " = [ ";
  for (size_t i = 0; i < 6; i++) {
    std::cout << std::setw(9) << std::fixed << std::setprecision(4) << a[i];
    if (i < 5) std::cout << ", ";
  }
  std::cout << " ]" << std::endl;
}

int main(int argc, char** argv) {
  try {
    std::string robot_ip = "172.16.0.2";

    if (argc >= 2) {
      robot_ip = argv[1];
    }

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;

    franka::Robot robot(robot_ip);

    std::cout << "Reading robot state..." << std::endl;
    std::cout << "Press Ctrl+C to stop.\n" << std::endl;

    while (true) {
      franka::RobotState state = robot.readOnce();

      std::cout << "----------------------------------------" << std::endl;

      // Measured joint positions, rad
      printArray7("q [rad]                 ", state.q);

      // Measured joint velocities, rad/s
      printArray7("dq [rad/s]              ", state.dq);

      // Measured joint torques, Nm
      printArray7("tau_J [Nm]              ", state.tau_J);

      // Desired joint torques without gravity, Nm
      printArray7("tau_J_d [Nm]            ", state.tau_J_d);

      // Estimated external joint torques, Nm
      printArray7("tau_ext_hat_filtered[Nm]", state.tau_ext_hat_filtered);

      // External wrench in base frame:
      // [Fx, Fy, Fz, Mx, My, Mz]
      printArray6("O_F_ext_hat_K [N,Nm]    ", state.O_F_ext_hat_K);

      // External wrench in stiffness frame:
      // [Fx, Fy, Fz, Mx, My, Mz]
      printArray6("K_F_ext_hat_K [N,Nm]    ", state.K_F_ext_hat_K);

      // Contact/collision indicators
      printArray7("joint_contact           ", state.joint_contact);
      printArray7("joint_collision         ", state.joint_collision);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

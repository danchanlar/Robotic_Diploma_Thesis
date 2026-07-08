#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/control_types.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct LogSample {
  double t;

  double x;
  double y;
  double z;

  double xd;
  double yd;
  double zd;

  double ex;
  double ey;
  double ez;

  double Fx;
  double Fy;
  double Fz;
  double Mx;
  double My;
  double Mz;

  double tau_ext_1;
  double tau_ext_2;
  double tau_ext_3;
  double tau_ext_4;
  double tau_ext_5;
  double tau_ext_6;
  double tau_ext_7;

  double tau_cmd_1;
  double tau_cmd_2;
  double tau_cmd_3;
  double tau_cmd_4;
  double tau_cmd_5;
  double tau_cmd_6;
  double tau_cmd_7;

  double cartesian_contact_x;
  double cartesian_contact_y;
  double cartesian_contact_z;
  double cartesian_contact_rx;
  double cartesian_contact_ry;
  double cartesian_contact_rz;
};

struct TestConfig {
  std::string mode;
  double translational_stiffness;
  double rotational_stiffness;
  double nullspace_stiffness;
  double duration_sec;
  double move_start_sec;
  double move_end_sec;
  Eigen::Vector3d motion_offset;
  std::string csv_path;
};

Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

double smooth01(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 0.5 - 0.5 * std::cos(kPi * s);
}

TestConfig makeConfig(const std::string& mode) {
  TestConfig cfg;
  cfg.mode = mode;
  cfg.duration_sec = 24.0;
  cfg.move_start_sec = 3.0;
  cfg.move_end_sec = 9.0;

  // Small relative motion used as a safe task.
  // The robot moves the Cartesian equilibrium point by 3 cm in +x.
  cfg.motion_offset = Eigen::Vector3d(0.03, 0.0, 0.0);

  if (mode == "hard" || mode == "stiff") {
    cfg.mode = "hard";
    cfg.translational_stiffness = 500.0;  // N/m
    cfg.rotational_stiffness = 35.0;      // Nm/rad
    cfg.nullspace_stiffness = 20.0;
    cfg.csv_path = "impedance_task_hard_log.csv";
  } else {
    cfg.mode = "soft";
    cfg.translational_stiffness = 120.0;  // N/m
    cfg.rotational_stiffness = 8.0;       // Nm/rad
    cfg.nullspace_stiffness = 8.0;
    cfg.csv_path = "impedance_task_soft_log.csv";
  }

  return cfg;
}

void writeCsv(const std::string& path, const std::vector<LogSample>& samples) {
  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Could not open CSV file for writing: " << path << std::endl;
    return;
  }

  file << "t,"
       << "x,y,z,"
       << "xd,yd,zd,"
       << "ex,ey,ez,"
       << "Fx,Fy,Fz,Mx,My,Mz,"
       << "tau_ext_1,tau_ext_2,tau_ext_3,tau_ext_4,tau_ext_5,tau_ext_6,tau_ext_7,"
       << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7,"
       << "cartesian_contact_x,cartesian_contact_y,cartesian_contact_z,"
       << "cartesian_contact_rx,cartesian_contact_ry,cartesian_contact_rz\n";

  file << std::fixed << std::setprecision(8);

  for (const auto& s : samples) {
    file << s.t << ","
         << s.x << "," << s.y << "," << s.z << ","
         << s.xd << "," << s.yd << "," << s.zd << ","
         << s.ex << "," << s.ey << "," << s.ez << ","
         << s.Fx << "," << s.Fy << "," << s.Fz << ","
         << s.Mx << "," << s.My << "," << s.Mz << ","
         << s.tau_ext_1 << "," << s.tau_ext_2 << "," << s.tau_ext_3 << ","
         << s.tau_ext_4 << "," << s.tau_ext_5 << "," << s.tau_ext_6 << ","
         << s.tau_ext_7 << ","
         << s.tau_cmd_1 << "," << s.tau_cmd_2 << "," << s.tau_cmd_3 << ","
         << s.tau_cmd_4 << "," << s.tau_cmd_5 << "," << s.tau_cmd_6 << ","
         << s.tau_cmd_7 << ","
         << s.cartesian_contact_x << "," << s.cartesian_contact_y << ","
         << s.cartesian_contact_z << "," << s.cartesian_contact_rx << ","
         << s.cartesian_contact_ry << "," << s.cartesian_contact_rz << "\n";
  }
}

void printSummary(const std::vector<LogSample>& samples) {
  if (samples.empty()) {
    std::cout << "No log samples recorded." << std::endl;
    return;
  }

  double max_position_error = 0.0;
  double max_force_norm = 0.0;
  double max_abs_fz = 0.0;

  for (const auto& s : samples) {
    double e_norm = std::sqrt(s.ex * s.ex + s.ey * s.ey + s.ez * s.ez);
    double f_norm = std::sqrt(s.Fx * s.Fx + s.Fy * s.Fy + s.Fz * s.Fz);

    max_position_error = std::max(max_position_error, e_norm);
    max_force_norm = std::max(max_force_norm, f_norm);
    max_abs_fz = std::max(max_abs_fz, std::abs(s.Fz));
  }

  std::cout << "\n========== Impedance Test Summary ==========\n";
  std::cout << "Samples recorded: " << samples.size() << "\n";
  std::cout << "Max position error norm: " << max_position_error << " m\n";
  std::cout << "Max external force norm: " << max_force_norm << " N\n";
  std::cout << "Max |Fz|: " << max_abs_fz << " N\n";
  std::cout << "===========================================\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string robot_ip = "172.16.0.2";
    std::string mode = "soft";

    if (argc >= 2) {
      robot_ip = argv[1];
    }

    if (argc >= 3) {
      mode = argv[2];
    }

    TestConfig cfg = makeConfig(mode);

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;
    std::cout << "Impedance mode: " << cfg.mode << std::endl;
    std::cout << "Translational stiffness: " << cfg.translational_stiffness << " N/m" << std::endl;
    std::cout << "Rotational stiffness: " << cfg.rotational_stiffness << " Nm/rad" << std::endl;
    std::cout << "Duration: " << cfg.duration_sec << " sec" << std::endl;
    std::cout << "CSV output: " << cfg.csv_path << std::endl;

    franka::Robot robot(robot_ip);

    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    Eigen::Matrix4d initial_T = arrayToEigen(initial_state.O_T_EE);
    Eigen::Vector3d position_initial = initial_T.block<3, 1>(0, 3);
    Eigen::Quaterniond orientation_d(initial_T.block<3, 3>(0, 0));
    orientation_d.normalize();

    Eigen::Vector3d position_target = position_initial + cfg.motion_offset;

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Matrix<double, 7, 1> q_d_nullspace = q_initial;

    std::cout << "\nInitial EE position: "
              << "x=" << position_initial.x()
              << " y=" << position_initial.y()
              << " z=" << position_initial.z() << std::endl;

    std::cout << "Target equilibrium after small task motion: "
              << "x=" << position_target.x()
              << " y=" << position_target.y()
              << " z=" << position_target.z() << std::endl;

    std::cout << "\nWhat this test does:\n";
    std::cout << "  0-3 sec: holds the current pose with Cartesian impedance.\n";
    std::cout << "  3-9 sec: moves the impedance equilibrium point by 3 cm in +x.\n";
    std::cout << "  9-24 sec: holds the new pose. Gently push the end-effector by hand.\n";
    std::cout << "\nRun once with 'soft' and once with 'hard'.\n";
    std::cout << "The soft run should show larger position deflection for similar external force.\n";
    std::cout << "\nKeep one hand near the emergency stop. Push gently only.\n";
    std::cout << "Press ENTER to start..." << std::endl;
    std::cin.ignore();

    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> damping;

    stiffness.setZero();
    damping.setZero();

    stiffness.topLeftCorner<3, 3>() =
        cfg.translational_stiffness * Eigen::Matrix3d::Identity();

    stiffness.bottomRightCorner<3, 3>() =
        cfg.rotational_stiffness * Eigen::Matrix3d::Identity();

    // Critical damping approximation.
    damping.topLeftCorner<3, 3>() =
        2.0 * std::sqrt(cfg.translational_stiffness) * Eigen::Matrix3d::Identity();

    damping.bottomRightCorner<3, 3>() =
        2.0 * std::sqrt(cfg.rotational_stiffness) * Eigen::Matrix3d::Identity();

    const size_t max_samples = static_cast<size_t>(cfg.duration_sec * 100.0) + 100;
    std::vector<LogSample> samples;
    samples.resize(max_samples);
    size_t sample_count = 0;

    double elapsed_time = 0.0;
    double last_log_time = -1.0;
    double last_print_time = -1.0;

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_callback =
            [&](const franka::RobotState& robot_state,
                franka::Duration period) -> franka::Torques {
      elapsed_time += period.toSec();

      Eigen::Vector3d position_d = position_initial;

      if (elapsed_time > cfg.move_start_sec) {
        double s = (elapsed_time - cfg.move_start_sec) /
                   (cfg.move_end_sec - cfg.move_start_sec);
        position_d = position_initial + smooth01(s) * cfg.motion_offset;
      }

      if (elapsed_time >= cfg.move_end_sec) {
        position_d = position_target;
      }

      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      Eigen::Matrix4d current_T = arrayToEigen(robot_state.O_T_EE);
      Eigen::Vector3d position = current_T.block<3, 1>(0, 3);
      Eigen::Matrix3d rotation = current_T.block<3, 3>(0, 0);
      Eigen::Quaterniond orientation(rotation);
      orientation.normalize();

      Eigen::Matrix<double, 6, 1> error;
      error.head<3>() = position - position_d;

      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() *= -1.0;
      }

      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail<3>() << error_quaternion.x(),
                          error_quaternion.y(),
                          error_quaternion.z();

      error.tail<3>() = -rotation * error.tail<3>();

      Eigen::Matrix<double, 7, 1> tau_task =
          jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));

      Eigen::Matrix<double, 7, 1> tau_nullspace =
          cfg.nullspace_stiffness * (q_d_nullspace - q)
          - 2.0 * std::sqrt(cfg.nullspace_stiffness) * dq;

      Eigen::Matrix<double, 7, 1> tau_d =
          tau_task + 0.15 * tau_nullspace + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;

      if (elapsed_time - last_log_time >= 0.01 && sample_count < samples.size()) {
        last_log_time = elapsed_time;

        LogSample s{};
        s.t = elapsed_time;

        s.x = position.x();
        s.y = position.y();
        s.z = position.z();

        s.xd = position_d.x();
        s.yd = position_d.y();
        s.zd = position_d.z();

        s.ex = error(0);
        s.ey = error(1);
        s.ez = error(2);

        s.Fx = robot_state.O_F_ext_hat_K[0];
        s.Fy = robot_state.O_F_ext_hat_K[1];
        s.Fz = robot_state.O_F_ext_hat_K[2];
        s.Mx = robot_state.O_F_ext_hat_K[3];
        s.My = robot_state.O_F_ext_hat_K[4];
        s.Mz = robot_state.O_F_ext_hat_K[5];

        s.tau_ext_1 = robot_state.tau_ext_hat_filtered[0];
        s.tau_ext_2 = robot_state.tau_ext_hat_filtered[1];
        s.tau_ext_3 = robot_state.tau_ext_hat_filtered[2];
        s.tau_ext_4 = robot_state.tau_ext_hat_filtered[3];
        s.tau_ext_5 = robot_state.tau_ext_hat_filtered[4];
        s.tau_ext_6 = robot_state.tau_ext_hat_filtered[5];
        s.tau_ext_7 = robot_state.tau_ext_hat_filtered[6];

        s.tau_cmd_1 = tau_d_array[0];
        s.tau_cmd_2 = tau_d_array[1];
        s.tau_cmd_3 = tau_d_array[2];
        s.tau_cmd_4 = tau_d_array[3];
        s.tau_cmd_5 = tau_d_array[4];
        s.tau_cmd_6 = tau_d_array[5];
        s.tau_cmd_7 = tau_d_array[6];

        s.cartesian_contact_x = robot_state.cartesian_contact[0];
        s.cartesian_contact_y = robot_state.cartesian_contact[1];
        s.cartesian_contact_z = robot_state.cartesian_contact[2];
        s.cartesian_contact_rx = robot_state.cartesian_contact[3];
        s.cartesian_contact_ry = robot_state.cartesian_contact[4];
        s.cartesian_contact_rz = robot_state.cartesian_contact[5];

        samples[sample_count++] = s;
      }

      if (elapsed_time - last_print_time >= 0.5) {
        last_print_time = elapsed_time;

        double e_norm = error.head<3>().norm();
        const auto& F = robot_state.O_F_ext_hat_K;

        std::cout << std::fixed << std::setprecision(4)
                  << "t=" << elapsed_time
                  << "  pos=[" << position.x() << ", " << position.y() << ", " << position.z() << "]"
                  << "  pos_d=[" << position_d.x() << ", " << position_d.y() << ", " << position_d.z() << "]"
                  << "  |e_pos|=" << e_norm
                  << "  F=[" << F[0] << ", " << F[1] << ", " << F[2] << "]"
                  << std::endl;
      }

      franka::Torques output(tau_d_array);

      if (elapsed_time >= cfg.duration_sec) {
        return franka::MotionFinished(output);
      }

      return output;
    };

    robot.control(impedance_callback, true);

    samples.resize(sample_count);
    writeCsv(cfg.csv_path, samples);
    printSummary(samples);

    std::cout << "\nCSV written to: " << cfg.csv_path << std::endl;
    std::cout << "\nRecommended proof:\n";
    std::cout << "  1. Run soft mode and gently push during the final hold phase.\n";
    std::cout << "  2. Run hard mode and push similarly.\n";
    std::cout << "  3. Compare max position error in the two CSV files.\n";
    std::cout << "     Soft should deflect more than hard for similar external force.\n";

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

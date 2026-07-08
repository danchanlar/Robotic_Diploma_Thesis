#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct LogSample {
  double t;
  double x, y, z;
  double x_nom, y_nom, z_nom;
  double x_cmd, y_cmd, z_cmd;
  double adm_x, adm_y, adm_z;
  double adm_vx, adm_vy, adm_vz;
  double ex_cmd, ey_cmd, ez_cmd;
  double Fx, Fy, Fz, Mx, My, Mz;
  double Ff_x, Ff_y, Ff_z;
};

struct TestConfig {
  std::string mode;

  // Admittance model:
  // M*xdd + D*xd + K*x = F_ext
  double virtual_mass;
  double virtual_damping;
  double virtual_stiffness;

  double duration_sec;
  double move_start_sec;
  double move_end_sec;
  double interaction_end_sec;

  Eigen::Vector3d motion_offset;

  // Force processing.
  double force_deadband_N;
  double force_filter_alpha;
  double max_force_for_admittance_N;
  double force_sign;

  // Safety/rate limits for the admittance state.
  double max_admittance_offset_m;
  double max_admittance_velocity_mps;
  double max_admittance_acceleration_mps2;

  std::string csv_path;
};

std::array<double, 16> eigenToArray(const Eigen::Matrix4d& T) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
  return out;
}

Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

double smooth01(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 0.5 - 0.5 * std::cos(kPi * s);
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d& v, double max_norm) {
  double n = v.norm();
  if (n > max_norm && n > 1e-12) {
    return v * (max_norm / n);
  }
  return v;
}

Eigen::Vector3d applyDeadband(const Eigen::Vector3d& f, double deadband) {
  Eigen::Vector3d out = f;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(out(i)) < deadband) {
      out(i) = 0.0;
    } else {
      out(i) -= std::copysign(deadband, out(i));
    }
  }
  return out;
}

TestConfig makeConfig(const std::string& mode) {
  TestConfig cfg;
  cfg.mode = mode;

  cfg.duration_sec = 30.0;
  cfg.move_start_sec = 3.0;
  cfg.move_end_sec = 9.0;
  cfg.interaction_end_sec = 22.0;

  // Very small nominal task motion.
  cfg.motion_offset = Eigen::Vector3d(0.03, 0.0, 0.0);

  // IMPORTANT:
  // These values are intentionally conservative because CartesianPose
  // commands in libfranka must be very smooth.
  cfg.force_deadband_N = 2.0;
  cfg.force_filter_alpha = 0.01;
  cfg.max_force_for_admittance_N = 4.0;
  cfg.max_admittance_offset_m = 0.015;          // 15 mm
  cfg.max_admittance_velocity_mps = 0.015;      // 15 mm/s
  cfg.max_admittance_acceleration_mps2 = 0.04;  // 40 mm/s^2
  cfg.force_sign = 1.0;

  if (mode == "hard" || mode == "stiff") {
    cfg.mode = "hard";
    cfg.virtual_mass = 12.0;
    cfg.virtual_damping = 220.0;
    cfg.virtual_stiffness = 500.0;
    cfg.csv_path = "admittance_task_hard_log.csv";
  } else {
    cfg.mode = "soft";
    cfg.virtual_mass = 10.0;
    cfg.virtual_damping = 130.0;
    cfg.virtual_stiffness = 120.0;
    cfg.csv_path = "admittance_task_soft_log.csv";
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
       << "x_nom,y_nom,z_nom,"
       << "x_cmd,y_cmd,z_cmd,"
       << "adm_x,adm_y,adm_z,"
       << "adm_vx,adm_vy,adm_vz,"
       << "ex_cmd,ey_cmd,ez_cmd,"
       << "Fx,Fy,Fz,Mx,My,Mz,"
       << "Ff_x,Ff_y,Ff_z\n";

  file << std::fixed << std::setprecision(8);

  for (const auto& s : samples) {
    file << s.t << ","
         << s.x << "," << s.y << "," << s.z << ","
         << s.x_nom << "," << s.y_nom << "," << s.z_nom << ","
         << s.x_cmd << "," << s.y_cmd << "," << s.z_cmd << ","
         << s.adm_x << "," << s.adm_y << "," << s.adm_z << ","
         << s.adm_vx << "," << s.adm_vy << "," << s.adm_vz << ","
         << s.ex_cmd << "," << s.ey_cmd << "," << s.ez_cmd << ","
         << s.Fx << "," << s.Fy << "," << s.Fz << ","
         << s.Mx << "," << s.My << "," << s.Mz << ","
         << s.Ff_x << "," << s.Ff_y << "," << s.Ff_z << "\n";
  }

  std::cout << "CSV written to: " << path << std::endl;
}

void printSummary(const std::vector<LogSample>& samples) {
  if (samples.empty()) {
    std::cout << "No log samples recorded." << std::endl;
    return;
  }

  double max_adm_mm = 0.0;
  double max_v_mm_s = 0.0;
  double max_force = 0.0;

  for (const auto& s : samples) {
    double adm = std::sqrt(s.adm_x*s.adm_x + s.adm_y*s.adm_y + s.adm_z*s.adm_z) * 1000.0;
    double vel = std::sqrt(s.adm_vx*s.adm_vx + s.adm_vy*s.adm_vy + s.adm_vz*s.adm_vz) * 1000.0;
    double force = std::sqrt(s.Fx*s.Fx + s.Fy*s.Fy + s.Fz*s.Fz);
    max_adm_mm = std::max(max_adm_mm, adm);
    max_v_mm_s = std::max(max_v_mm_s, vel);
    max_force = std::max(max_force, force);
  }

  std::cout << "\n========== Safe Admittance Summary ==========\n";
  std::cout << "Samples: " << samples.size() << "\n";
  std::cout << "Max admittance offset: " << max_adm_mm << " mm\n";
  std::cout << "Max admittance velocity: " << max_v_mm_s << " mm/s\n";
  std::cout << "Max external force norm: " << max_force << " N\n";
  std::cout << "============================================\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<LogSample> samples;
  std::string csv_path = "admittance_task_log.csv";

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
    csv_path = cfg.csv_path;

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;
    std::cout << "Admittance mode: " << cfg.mode << std::endl;
    std::cout << "Virtual mass: " << cfg.virtual_mass << " kg" << std::endl;
    std::cout << "Virtual damping: " << cfg.virtual_damping << " N*s/m" << std::endl;
    std::cout << "Virtual stiffness: " << cfg.virtual_stiffness << " N/m" << std::endl;
    std::cout << "Max force used by admittance: " << cfg.max_force_for_admittance_N << " N" << std::endl;
    std::cout << "Max admittance offset: " << cfg.max_admittance_offset_m * 1000.0 << " mm" << std::endl;
    std::cout << "Max admittance velocity: " << cfg.max_admittance_velocity_mps * 1000.0 << " mm/s" << std::endl;
    std::cout << "Max admittance acceleration: " << cfg.max_admittance_acceleration_mps2 * 1000.0 << " mm/s^2" << std::endl;
    std::cout << "Duration: " << cfg.duration_sec << " sec" << std::endl;
    std::cout << "Interaction phase ends at: " << cfg.interaction_end_sec << " sec" << std::endl;
    std::cout << "CSV output: " << cfg.csv_path << std::endl;

    franka::Robot robot(robot_ip);

    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Matrix4d initial_T = arrayToEigen(initial_state.O_T_EE);
    Eigen::Vector3d position_initial = initial_T.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation_initial = initial_T.block<3, 3>(0, 0);
    Eigen::Quaterniond orientation_initial(rotation_initial);
    orientation_initial.normalize();

    Eigen::Vector3d position_target = position_initial + cfg.motion_offset;

    std::cout << "\nInitial EE position: "
              << "x=" << position_initial.x()
              << " y=" << position_initial.y()
              << " z=" << position_initial.z() << std::endl;

    std::cout << "Nominal target after small task motion: "
              << "x=" << position_target.x()
              << " y=" << position_target.y()
              << " z=" << position_target.z() << std::endl;

    std::cout << "\nTiming:\n";
    std::cout << "  0-3 sec: hold\n";
    std::cout << "  3-9 sec: nominal 3 cm task motion\n";
    std::cout << "  9-22 sec: interaction phase. Push VERY gently here.\n";
    std::cout << "  22-30 sec: settle phase. Stop pushing completely.\n";
    std::cout << "\nRecommended push force: small, about 2-4 N. Do not force it to the 15 mm limit.\n";
    std::cout << "Press ENTER to start..." << std::endl;
    std::cin.ignore();

    const size_t max_samples = static_cast<size_t>(cfg.duration_sec * 100.0) + 100;
    samples.resize(max_samples);
    size_t sample_count = 0;

    double elapsed_time = 0.0;
    double last_log_time = -1.0;
    double last_print_time = -1.0;

    Eigen::Vector3d adm_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d adm_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_filtered = Eigen::Vector3d::Zero();

    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration period)
            -> franka::CartesianPose {
          const double dt = period.toSec();
          elapsed_time += dt;

          Eigen::Vector3d position_nom = position_initial;
          if (elapsed_time > cfg.move_start_sec) {
            double s = (elapsed_time - cfg.move_start_sec) /
                       (cfg.move_end_sec - cfg.move_start_sec);
            position_nom = position_initial + smooth01(s) * cfg.motion_offset;
          }
          if (elapsed_time >= cfg.move_end_sec) {
            position_nom = position_target;
          }

          Eigen::Matrix4d current_T = arrayToEigen(robot_state.O_T_EE);
          Eigen::Vector3d position = current_T.block<3, 1>(0, 3);

          Eigen::Vector3d force_raw(
              robot_state.O_F_ext_hat_K[0],
              robot_state.O_F_ext_hat_K[1],
              robot_state.O_F_ext_hat_K[2]);

          force_filtered =
              (1.0 - cfg.force_filter_alpha) * force_filtered
              + cfg.force_filter_alpha * force_raw;

          Eigen::Vector3d force_for_admittance =
              cfg.force_sign * applyDeadband(force_filtered, cfg.force_deadband_N);

          force_for_admittance =
              clampNorm(force_for_admittance, cfg.max_force_for_admittance_N);

          // During settle phase, ignore external force so that the command becomes smooth.
          if (elapsed_time >= cfg.interaction_end_sec) {
            force_for_admittance.setZero();
          }

          // M*xdd + D*xd + K*x = F
          Eigen::Vector3d adm_acceleration =
              (force_for_admittance
               - cfg.virtual_damping * adm_velocity
               - cfg.virtual_stiffness * adm_offset)
              / cfg.virtual_mass;

          adm_acceleration =
              clampNorm(adm_acceleration, cfg.max_admittance_acceleration_mps2);

          adm_velocity += adm_acceleration * dt;
          adm_velocity = clampNorm(adm_velocity, cfg.max_admittance_velocity_mps);

          adm_offset += adm_velocity * dt;
          adm_offset = clampNorm(adm_offset, cfg.max_admittance_offset_m);

          // Extra final damping near the end.
          if (elapsed_time >= cfg.duration_sec - 3.0) {
            adm_velocity *= 0.85;
            adm_offset *= 0.990;
          }

          Eigen::Vector3d position_cmd = position_nom + adm_offset;

          Eigen::Matrix4d command_T = Eigen::Matrix4d::Identity();
          command_T.block<3, 3>(0, 0) = orientation_initial.toRotationMatrix();
          command_T.block<3, 1>(0, 3) = position_cmd;

          franka::CartesianPose output_pose(eigenToArray(command_T));

          if (elapsed_time - last_log_time >= 0.01 && sample_count < samples.size()) {
            last_log_time = elapsed_time;

            LogSample s{};
            s.t = elapsed_time;

            s.x = position.x();
            s.y = position.y();
            s.z = position.z();

            s.x_nom = position_nom.x();
            s.y_nom = position_nom.y();
            s.z_nom = position_nom.z();

            s.x_cmd = position_cmd.x();
            s.y_cmd = position_cmd.y();
            s.z_cmd = position_cmd.z();

            s.adm_x = adm_offset.x();
            s.adm_y = adm_offset.y();
            s.adm_z = adm_offset.z();

            s.adm_vx = adm_velocity.x();
            s.adm_vy = adm_velocity.y();
            s.adm_vz = adm_velocity.z();

            s.ex_cmd = position.x() - position_cmd.x();
            s.ey_cmd = position.y() - position_cmd.y();
            s.ez_cmd = position.z() - position_cmd.z();

            s.Fx = robot_state.O_F_ext_hat_K[0];
            s.Fy = robot_state.O_F_ext_hat_K[1];
            s.Fz = robot_state.O_F_ext_hat_K[2];
            s.Mx = robot_state.O_F_ext_hat_K[3];
            s.My = robot_state.O_F_ext_hat_K[4];
            s.Mz = robot_state.O_F_ext_hat_K[5];

            s.Ff_x = force_filtered.x();
            s.Ff_y = force_filtered.y();
            s.Ff_z = force_filtered.z();

            samples[sample_count++] = s;
          }

          if (elapsed_time - last_print_time >= 0.5) {
            last_print_time = elapsed_time;

            double adm_norm_mm = adm_offset.norm() * 1000.0;
            double adm_vel_mm_s = adm_velocity.norm() * 1000.0;
            double force_norm = force_raw.norm();

            std::cout << std::fixed << std::setprecision(4)
                      << "t=" << elapsed_time
                      << "  pos=[" << position.x() << ", " << position.y() << ", " << position.z() << "]"
                      << "  cmd=[" << position_cmd.x() << ", " << position_cmd.y() << ", " << position_cmd.z() << "]"
                      << "  adm_offset=" << adm_norm_mm << " mm"
                      << "  adm_vel=" << adm_vel_mm_s << " mm/s"
                      << "  |F|=" << force_norm << " N";

            if (elapsed_time >= cfg.interaction_end_sec) {
              std::cout << "  SETTLING";
            }

            std::cout << std::endl;
          }

          if (elapsed_time >= cfg.duration_sec) {
            return franka::MotionFinished(output_pose);
          }

          return output_pose;
        },
        franka::ControllerMode::kCartesianImpedance,
        true);

    samples.resize(sample_count);
    writeCsv(csv_path, samples);
    printSummary(samples);

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;

    size_t real_count = 0;
    while (real_count < samples.size() && samples[real_count].t > 0.0) {
      real_count++;
    }
    samples.resize(real_count);

    if (!samples.empty()) {
      std::cerr << "Writing partial CSV despite exception..." << std::endl;
      writeCsv(csv_path, samples);
      printSummary(samples);
    }

    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;

    size_t real_count = 0;
    while (real_count < samples.size() && samples[real_count].t > 0.0) {
      real_count++;
    }
    samples.resize(real_count);

    if (!samples.empty()) {
      std::cerr << "Writing partial CSV despite exception..." << std::endl;
      writeCsv(csv_path, samples);
      printSummary(samples);
    }

    return -1;
  }

  return 0;
}

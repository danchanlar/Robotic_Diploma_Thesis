#include <array>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int) {
  g_stop_requested = 1;
}

struct Options {
  std::string robot_ip = "172.16.0.2";
  std::string mode = "hold";  // hold, sine_x, sine_y, sine_z
  double duration = 25.0;
  double amplitude = 0.020;
  double frequency = 0.05;

  double trans_stiffness = 250.0;
  double trans_damping = 2.0 * std::sqrt(250.0);
  double orient_stiffness = 20.0;
  double orient_damping = 2.0 * std::sqrt(20.0);
  double nullspace_stiffness = 5.0;
  double nullspace_damping = 2.0 * std::sqrt(5.0);

  double max_force = 25.0;
  double max_external_torque_norm = 12.0;
  double max_ee_speed = 0.25;
  double disturbance_threshold = 4.0;

  int log_decimation = 5;
  std::string output = "/tmp/fr3_exp4_human_push_disturbance.csv";
  bool acknowledged_risk = false;
};

void printUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " --robot-ip 172.16.0.2 --output log.csv --i-understand-real-robot-risk [options]\n\n"
            << "Options:\n"
            << "  --mode hold|sine_x|sine_y|sine_z       Reference type. Default: hold\n"
            << "  --duration SECONDS                     Experiment duration. Default: 25\n"
            << "  --amplitude METERS                     Sine reference amplitude. Default: 0.020\n"
            << "  --frequency HZ                         Sine reference frequency. Default: 0.05\n"
            << "  --trans-stiffness N_PER_M              Translational stiffness. Default: 250\n"
            << "  --trans-damping NS_PER_M               Translational damping. Default: critical-like\n"
            << "  --orient-stiffness NM_PER_RAD          Orientation stiffness. Default: 20\n"
            << "  --orient-damping NMS_PER_RAD           Orientation damping. Default: critical-like\n"
            << "  --nullspace-stiffness NM_PER_RAD       Nullspace posture stiffness. Default: 5\n"
            << "  --max-force N                          Safety limit on estimated external wrench force norm\n"
            << "  --max-ext-torque-norm NM               Safety limit on external joint torque norm\n"
            << "  --max-ee-speed M_PER_S                 Safety limit on end-effector speed\n"
            << "  --disturbance-threshold N              Force norm above which disturbance_flag=1\n"
            << "  --log-decimation N                     Log every N control cycles. Default: 5\n";
}

bool parseDouble(const std::string& text, double& value) {
  try {
    size_t idx = 0;
    value = std::stod(text, &idx);
    return idx == text.size();
  } catch (...) {
    return false;
  }
}

bool parseInt(const std::string& text, int& value) {
  try {
    size_t idx = 0;
    value = std::stoi(text, &idx);
    return idx == text.size();
  } catch (...) {
    return false;
  }
}

Options parseOptions(int argc, char** argv) {
  Options opt;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need_value = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value after " + name);
      }
      return argv[++i];
    };

    if (a == "--help" || a == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (a == "--robot-ip") {
      opt.robot_ip = need_value(a);
    } else if (a == "--mode") {
      opt.mode = need_value(a);
    } else if (a == "--duration") {
      if (!parseDouble(need_value(a), opt.duration)) throw std::runtime_error("Invalid --duration");
    } else if (a == "--amplitude") {
      if (!parseDouble(need_value(a), opt.amplitude)) throw std::runtime_error("Invalid --amplitude");
    } else if (a == "--frequency") {
      if (!parseDouble(need_value(a), opt.frequency)) throw std::runtime_error("Invalid --frequency");
    } else if (a == "--trans-stiffness") {
      if (!parseDouble(need_value(a), opt.trans_stiffness)) throw std::runtime_error("Invalid --trans-stiffness");
    } else if (a == "--trans-damping") {
      if (!parseDouble(need_value(a), opt.trans_damping)) throw std::runtime_error("Invalid --trans-damping");
    } else if (a == "--orient-stiffness") {
      if (!parseDouble(need_value(a), opt.orient_stiffness)) throw std::runtime_error("Invalid --orient-stiffness");
    } else if (a == "--orient-damping") {
      if (!parseDouble(need_value(a), opt.orient_damping)) throw std::runtime_error("Invalid --orient-damping");
    } else if (a == "--nullspace-stiffness") {
      if (!parseDouble(need_value(a), opt.nullspace_stiffness)) throw std::runtime_error("Invalid --nullspace-stiffness");
    } else if (a == "--nullspace-damping") {
      if (!parseDouble(need_value(a), opt.nullspace_damping)) throw std::runtime_error("Invalid --nullspace-damping");
    } else if (a == "--max-force") {
      if (!parseDouble(need_value(a), opt.max_force)) throw std::runtime_error("Invalid --max-force");
    } else if (a == "--max-ext-torque-norm") {
      if (!parseDouble(need_value(a), opt.max_external_torque_norm)) throw std::runtime_error("Invalid --max-ext-torque-norm");
    } else if (a == "--max-ee-speed") {
      if (!parseDouble(need_value(a), opt.max_ee_speed)) throw std::runtime_error("Invalid --max-ee-speed");
    } else if (a == "--disturbance-threshold") {
      if (!parseDouble(need_value(a), opt.disturbance_threshold)) throw std::runtime_error("Invalid --disturbance-threshold");
    } else if (a == "--log-decimation") {
      if (!parseInt(need_value(a), opt.log_decimation)) throw std::runtime_error("Invalid --log-decimation");
    } else if (a == "--output") {
      opt.output = need_value(a);
    } else if (a == "--i-understand-real-robot-risk") {
      opt.acknowledged_risk = true;
    } else {
      throw std::runtime_error("Unknown argument: " + a);
    }
  }

  if (!opt.acknowledged_risk) {
    throw std::runtime_error("Refusing to run on real robot without --i-understand-real-robot-risk");
  }
  if (opt.mode != "hold" && opt.mode != "sine_x" && opt.mode != "sine_y" && opt.mode != "sine_z") {
    throw std::runtime_error("--mode must be one of: hold, sine_x, sine_y, sine_z");
  }
  if (opt.duration <= 0.0 || opt.duration > 120.0) {
    throw std::runtime_error("--duration must be in (0, 120] s");
  }
  if (opt.amplitude < 0.0 || opt.amplitude > 0.050) {
    throw std::runtime_error("--amplitude must be in [0, 0.050] m");
  }
  if (opt.frequency < 0.0 || opt.frequency > 0.20) {
    throw std::runtime_error("--frequency must be in [0, 0.20] Hz");
  }
  if (opt.trans_stiffness <= 0.0 || opt.trans_stiffness > 1000.0) {
    throw std::runtime_error("--trans-stiffness must be in (0, 1000] N/m");
  }
  if (opt.trans_damping <= 0.0 || opt.trans_damping > 150.0) {
    throw std::runtime_error("--trans-damping must be in (0, 150] Ns/m");
  }
  if (opt.orient_stiffness <= 0.0 || opt.orient_stiffness > 80.0) {
    throw std::runtime_error("--orient-stiffness must be in (0, 80] Nm/rad");
  }
  if (opt.orient_damping <= 0.0 || opt.orient_damping > 50.0) {
    throw std::runtime_error("--orient-damping must be in (0, 50] Nms/rad");
  }
  if (opt.max_force <= 0.0 || opt.max_force > 60.0) {
    throw std::runtime_error("--max-force must be in (0, 60] N");
  }
  if (opt.max_external_torque_norm <= 0.0 || opt.max_external_torque_norm > 30.0) {
    throw std::runtime_error("--max-ext-torque-norm must be in (0, 30] Nm");
  }
  if (opt.max_ee_speed <= 0.0 || opt.max_ee_speed > 0.50) {
    throw std::runtime_error("--max-ee-speed must be in (0, 0.50] m/s");
  }
  if (opt.log_decimation <= 0 || opt.log_decimation > 1000) {
    throw std::runtime_error("--log-decimation must be in [1, 1000]");
  }

  return opt;
}

Eigen::Matrix<double, 7, 1> saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  constexpr double kMaxDeltaTau = 1.0;  // Nm per control cycle
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  for (size_t i = 0; i < 7; ++i) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::clamp(difference, -kMaxDeltaTau, kMaxDeltaTau);
  }
  return tau_d_saturated;
}

void writeHeader(std::ofstream& log) {
  log << "time,mode,disturbance_flag,"
      << "x,y,z,xd,yd,zd,ex,ey,ez,err_norm,"
      << "vx,vy,vz,vxd,vyd,vzd,ee_speed,"
      << "fx_ext,fy_ext,fz_ext,mx_ext,my_ext,mz_ext,force_norm,wrench_norm,tau_ext_norm,"
      << "fx_cmd,fy_cmd,fz_cmd,mx_cmd,my_cmd,mz_cmd,";
  for (int i = 1; i <= 7; ++i) log << "tau_J" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "tau_ext" << i << ",";
  for (int i = 1; i <= 7; ++i) {
    log << "tau_cmd" << i;
    if (i < 7) log << ",";
  }
  log << "\n";
}

std::array<double, 16> robotPoseToArray(const Eigen::Affine3d& pose) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix<double, 4, 4>>(out.data()) = pose.matrix();
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  Options opt;
  try {
    opt = parseOptions(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    printUsage(argv[0]);
    return 1;
  }

  std::ofstream log(opt.output);
  if (!log) {
    std::cerr << "Could not open output file: " << opt.output << "\n";
    return 1;
  }
  writeHeader(log);

  try {
    franka::Robot robot(opt.robot_ip);
    robot.automaticErrorRecovery();
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(
        Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d0 = initial_transform.translation();
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Matrix<double, 7, 1> q_d_nullspace = q_initial;

    std::cout << "Experiment 4: Human push and disturbance rejection\n";
    std::cout << "Initial position: [" << position_d0.transpose() << "] m\n";
    std::cout << "Mode: " << opt.mode << "\n";
    std::cout << "Translational stiffness: " << opt.trans_stiffness << " N/m\n";
    std::cout << "Translational damping: " << opt.trans_damping << " Ns/m\n";
    std::cout << "Duration: " << opt.duration << " s\n";
    std::cout << "During the experiment, apply only small gentle pushes. Keep the emergency stop ready.\n";

    const Eigen::Matrix<double, 6, 6> stiffness = [&]() {
      Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
      K.topLeftCorner(3, 3) = opt.trans_stiffness * Eigen::Matrix3d::Identity();
      K.bottomRightCorner(3, 3) = opt.orient_stiffness * Eigen::Matrix3d::Identity();
      return K;
    }();

    const Eigen::Matrix<double, 6, 6> damping = [&]() {
      Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
      D.topLeftCorner(3, 3) = opt.trans_damping * Eigen::Matrix3d::Identity();
      D.bottomRightCorner(3, 3) = opt.orient_damping * Eigen::Matrix3d::Identity();
      return D;
    }();

    double time = 0.0;
    uint64_t sample = 0;
    std::string stop_reason = "duration_complete";

    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();
      time += dt;
      ++sample;

      std::array<double, 7> coriolis_array = model.coriolis(state);
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(state.tau_J_d.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext(state.tau_ext_hat_filtered.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

      Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
      Eigen::Vector3d position = transform.translation();
      Eigen::Quaterniond orientation(transform.linear());

      Eigen::Vector3d position_d = position_d0;
      Eigen::Vector3d velocity_d = Eigen::Vector3d::Zero();
      const double w = 2.0 * M_PI * opt.frequency;
      const double s = std::sin(w * time);
      const double c = std::cos(w * time);
      if (opt.mode == "sine_x") {
        position_d.x() += opt.amplitude * s;
        velocity_d.x() = opt.amplitude * w * c;
      } else if (opt.mode == "sine_y") {
        position_d.y() += opt.amplitude * s;
        velocity_d.y() = opt.amplitude * w * c;
      } else if (opt.mode == "sine_z") {
        position_d.z() += opt.amplitude * s;
        velocity_d.z() = opt.amplitude * w * c;
      }

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) = position - position_d;

      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() = -orientation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      error.tail(3) = -transform.rotation() * error.tail(3);

      Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
      Eigen::Matrix<double, 6, 1> velocity_desired = Eigen::Matrix<double, 6, 1>::Zero();
      velocity_desired.head(3) = velocity_d;

      Eigen::Matrix<double, 6, 1> wrench_cmd = -stiffness * error - damping * (velocity - velocity_desired);

      const Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
      const double lambda = 0.03;
      Eigen::Matrix<double, 6, 1> wrench_ext =
          (JJt + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(jacobian * tau_ext);
      const double force_norm = wrench_ext.head(3).norm();
      const double wrench_norm = wrench_ext.norm();
      const double tau_ext_norm = tau_ext.norm();
      const double ee_speed = velocity.head(3).norm();
      const int disturbance_flag = (force_norm >= opt.disturbance_threshold) ? 1 : 0;

      Eigen::Matrix<double, 7, 7> identity = Eigen::Matrix<double, 7, 7>::Identity();
      Eigen::Matrix<double, 7, 6> jacobian_transpose_pinv =
          jacobian.transpose() * (JJt + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
      Eigen::Matrix<double, 7, 7> null_projector = identity - jacobian.transpose() * jacobian_transpose_pinv.transpose();

      Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * wrench_cmd;
      Eigen::Matrix<double, 7, 1> tau_null = null_projector *
          (opt.nullspace_stiffness * (q_d_nullspace - q) - opt.nullspace_damping * dq);
      Eigen::Matrix<double, 7, 1> tau_d_calculated = tau_task + tau_null + coriolis;
      Eigen::Matrix<double, 7, 1> tau_d = saturateTorqueRate(tau_d_calculated, tau_J_d);

      std::array<double, 7> tau_d_array{};
      for (int i = 0; i < 7; ++i) {
        tau_d_array[static_cast<size_t>(i)] = tau_d(i);
      }

      if (sample % static_cast<uint64_t>(opt.log_decimation) == 0) {
        log << time << ',' << opt.mode << ',' << disturbance_flag << ','
            << position.x() << ',' << position.y() << ',' << position.z() << ','
            << position_d.x() << ',' << position_d.y() << ',' << position_d.z() << ','
            << (position_d.x() - position.x()) << ','
            << (position_d.y() - position.y()) << ','
            << (position_d.z() - position.z()) << ','
            << (position_d - position).norm() << ','
            << velocity(0) << ',' << velocity(1) << ',' << velocity(2) << ','
            << velocity_d.x() << ',' << velocity_d.y() << ',' << velocity_d.z() << ','
            << ee_speed << ','
            << wrench_ext(0) << ',' << wrench_ext(1) << ',' << wrench_ext(2) << ','
            << wrench_ext(3) << ',' << wrench_ext(4) << ',' << wrench_ext(5) << ','
            << force_norm << ',' << wrench_norm << ',' << tau_ext_norm << ','
            << wrench_cmd(0) << ',' << wrench_cmd(1) << ',' << wrench_cmd(2) << ','
            << wrench_cmd(3) << ',' << wrench_cmd(4) << ',' << wrench_cmd(5) << ',';
        for (int i = 0; i < 7; ++i) log << tau_J(i) << ',';
        for (int i = 0; i < 7; ++i) log << tau_ext(i) << ',';
        for (int i = 0; i < 7; ++i) {
          log << tau_d(i);
          if (i < 6) log << ',';
        }
        log << '\n';
      }

      bool finish = false;
      if (g_stop_requested) {
        stop_reason = "user_interrupt";
        finish = true;
      } else if (time >= opt.duration) {
        stop_reason = "duration_complete";
        finish = true;
      } else if (force_norm > opt.max_force) {
        stop_reason = "safety_force_norm";
        finish = true;
      } else if (tau_ext_norm > opt.max_external_torque_norm) {
        stop_reason = "safety_external_torque_norm";
        finish = true;
      } else if (ee_speed > opt.max_ee_speed) {
        stop_reason = "safety_ee_speed";
        finish = true;
      }

      if (finish) {
        std::cout << "Motion finished. Stop reason: " << stop_reason
                  << " t=" << time
                  << " force_norm=" << force_norm
                  << " tau_ext_norm=" << tau_ext_norm
                  << " ee_speed=" << ee_speed << "\n";
        return franka::MotionFinished(franka::Torques(tau_d_array));
      }

      return tau_d_array;
    });

    std::cout << "Saved CSV log: " << opt.output << "\n";
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    std::cerr << "Saved partial CSV log: " << opt.output << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    std::cerr << "Saved partial CSV log: " << opt.output << "\n";
    return 1;
  }
}

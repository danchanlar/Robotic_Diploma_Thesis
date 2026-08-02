#include <array>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

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
  double duration = 35.0;
  double frequency = 0.08;
  double ramp_time = 4.0;

  // Human-readable joint numbering: J2, J4, J6.
  double j2_amplitude = 0.20;  // rad
  double j4_amplitude = 0.18;  // rad
  double j6_amplitude = 0.25;  // rad

  double joint_stiffness = 60.0;  // Nm/rad
  double joint_damping = 10.0;    // Nms/rad

  double max_force = 25.0;
  double max_external_torque_norm = 12.0;
  double max_ee_speed = 0.35;
  double disturbance_threshold = 3.0;

  int log_decimation = 5;
  std::string output = "/tmp/fr3_exp4_joint_trajectory_push.csv";
  bool acknowledged_risk = false;
};

void printUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " --robot-ip 172.16.0.2 --output log.csv --i-understand-real-robot-risk [options]\n\n"
            << "Options:\n"
            << "  --duration SECONDS                 Experiment duration. Default: 35\n"
            << "  --frequency HZ                     Joint sine frequency. Default: 0.08\n"
            << "  --ramp-time SECONDS                Smooth amplitude ramp-in time. Default: 4\n"
            << "  --j2-amplitude RAD                 Joint 2 sine amplitude. Default: 0.20\n"
            << "  --j4-amplitude RAD                 Joint 4 sine amplitude. Default: 0.18\n"
            << "  --j6-amplitude RAD                 Joint 6 sine amplitude. Default: 0.25\n"
            << "  --joint-stiffness NM_PER_RAD       Joint impedance stiffness. Default: 60\n"
            << "  --joint-damping NMS_PER_RAD        Joint impedance damping. Default: 10\n"
            << "  --max-force N                      Safety limit on estimated external force norm\n"
            << "  --max-ext-torque-norm NM           Safety limit on external joint torque norm\n"
            << "  --max-ee-speed M_PER_S             Safety limit on end-effector speed\n"
            << "  --disturbance-threshold N          Force norm above which disturbance_flag=1\n"
            << "  --log-decimation N                 Log every N control cycles. Default: 5\n";
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
    } else if (a == "--duration") {
      if (!parseDouble(need_value(a), opt.duration)) throw std::runtime_error("Invalid --duration");
    } else if (a == "--frequency") {
      if (!parseDouble(need_value(a), opt.frequency)) throw std::runtime_error("Invalid --frequency");
    } else if (a == "--ramp-time") {
      if (!parseDouble(need_value(a), opt.ramp_time)) throw std::runtime_error("Invalid --ramp-time");
    } else if (a == "--j2-amplitude") {
      if (!parseDouble(need_value(a), opt.j2_amplitude)) throw std::runtime_error("Invalid --j2-amplitude");
    } else if (a == "--j4-amplitude") {
      if (!parseDouble(need_value(a), opt.j4_amplitude)) throw std::runtime_error("Invalid --j4-amplitude");
    } else if (a == "--j6-amplitude") {
      if (!parseDouble(need_value(a), opt.j6_amplitude)) throw std::runtime_error("Invalid --j6-amplitude");
    } else if (a == "--joint-stiffness") {
      if (!parseDouble(need_value(a), opt.joint_stiffness)) throw std::runtime_error("Invalid --joint-stiffness");
    } else if (a == "--joint-damping") {
      if (!parseDouble(need_value(a), opt.joint_damping)) throw std::runtime_error("Invalid --joint-damping");
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
  if (opt.duration <= 0.0 || opt.duration > 120.0) {
    throw std::runtime_error("--duration must be in (0, 120] s");
  }
  if (opt.frequency <= 0.0 || opt.frequency > 0.30) {
    throw std::runtime_error("--frequency must be in (0, 0.30] Hz");
  }
  if (opt.ramp_time < 0.0 || opt.ramp_time > 20.0) {
    throw std::runtime_error("--ramp-time must be in [0, 20] s");
  }
  if (std::abs(opt.j2_amplitude) > 0.45 || std::abs(opt.j4_amplitude) > 0.45 || std::abs(opt.j6_amplitude) > 0.65) {
    throw std::runtime_error("Joint amplitudes too large. Use |J2|,|J4| <= 0.45 rad and |J6| <= 0.65 rad");
  }
  if (opt.joint_stiffness <= 0.0 || opt.joint_stiffness > 180.0) {
    throw std::runtime_error("--joint-stiffness must be in (0, 180] Nm/rad");
  }
  if (opt.joint_damping <= 0.0 || opt.joint_damping > 40.0) {
    throw std::runtime_error("--joint-damping must be in (0, 40] Nms/rad");
  }
  if (opt.max_force <= 0.0 || opt.max_force > 60.0) {
    throw std::runtime_error("--max-force must be in (0, 60] N");
  }
  if (opt.max_external_torque_norm <= 0.0 || opt.max_external_torque_norm > 30.0) {
    throw std::runtime_error("--max-ext-torque-norm must be in (0, 30] Nm");
  }
  if (opt.max_ee_speed <= 0.0 || opt.max_ee_speed > 0.70) {
    throw std::runtime_error("--max-ee-speed must be in (0, 0.70] m/s");
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
      << "vx,vy,vz,ee_speed,"
      << "fx_ext,fy_ext,fz_ext,mx_ext,my_ext,mz_ext,force_norm,wrench_norm,tau_ext_norm,"
      << "fx_cmd,fy_cmd,fz_cmd,";
  for (int i = 1; i <= 7; ++i) log << "q" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "qd" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "dq" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "dqd" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "qerr" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "tau_J" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "tau_ext" << i << ",";
  for (int i = 1; i <= 7; ++i) {
    log << "tau_cmd" << i;
    if (i < 7) log << ",";
  }
  log << "\n";
}

struct SmoothRamp {
  double alpha;
  double alpha_dot;
};

SmoothRamp smoothRamp(double time, double ramp_time) {
  if (ramp_time <= 1e-9 || time >= ramp_time) {
    return {1.0, 0.0};
  }
  if (time <= 0.0) {
    return {0.0, 0.0};
  }
  const double sigma = std::clamp(time / ramp_time, 0.0, 1.0);
  const double alpha = 10.0 * std::pow(sigma, 3) - 15.0 * std::pow(sigma, 4) + 6.0 * std::pow(sigma, 5);
  const double dalpha_dsigma = 30.0 * std::pow(sigma, 2) - 60.0 * std::pow(sigma, 3) + 30.0 * std::pow(sigma, 4);
  return {alpha, dalpha_dsigma / ramp_time};
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
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position0 = initial_transform.translation();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q0(initial_state.q.data());

    std::cout << "Experiment 4B: Human push during joint trajectory\n";
    std::cout << "Initial EE position: [" << position0.transpose() << "] m\n";
    std::cout << "Moving J2, J4 and J6 with sinusoidal joint references.\n";
    std::cout << "J2 amplitude: " << opt.j2_amplitude << " rad\n";
    std::cout << "J4 amplitude: " << opt.j4_amplitude << " rad\n";
    std::cout << "J6 amplitude: " << opt.j6_amplitude << " rad\n";
    std::cout << "Frequency: " << opt.frequency << " Hz\n";
    std::cout << "Joint stiffness: " << opt.joint_stiffness << " Nm/rad\n";
    std::cout << "Joint damping: " << opt.joint_damping << " Nms/rad\n";
    std::cout << "Apply only small gentle pushes and keep the emergency stop ready.\n";

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
      Eigen::Matrix<double, 6, 1> cart_velocity = jacobian * dq;

      Eigen::Matrix<double, 7, 1> qd = q0;
      Eigen::Matrix<double, 7, 1> dqd = Eigen::Matrix<double, 7, 1>::Zero();

      const SmoothRamp ramp = smoothRamp(time, opt.ramp_time);
      const double w = 2.0 * M_PI * opt.frequency;
      const std::array<int, 3> idx = {1, 3, 5};  // J2, J4, J6 in zero-based indexing.
      const std::array<double, 3> amp = {opt.j2_amplitude, opt.j4_amplitude, opt.j6_amplitude};
      const std::array<double, 3> phase = {0.0, M_PI / 2.0, M_PI};

      for (size_t k = 0; k < idx.size(); ++k) {
        const double angle = w * time + phase[k];
        qd(idx[k]) += ramp.alpha * amp[k] * std::sin(angle);
        dqd(idx[k]) = ramp.alpha_dot * amp[k] * std::sin(angle)
                    + ramp.alpha * amp[k] * w * std::cos(angle);
      }

      Eigen::Matrix<double, 7, 1> tau_d_calculated =
          opt.joint_stiffness * (qd - q) + opt.joint_damping * (dqd - dq) + coriolis;
      Eigen::Matrix<double, 7, 1> tau_d = saturateTorqueRate(tau_d_calculated, tau_J_d);

      const Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
      const double lambda = 0.03;
      Eigen::Matrix<double, 6, 1> wrench_ext =
          (JJt + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(jacobian * tau_ext);
      const double force_norm = wrench_ext.head(3).norm();
      const double wrench_norm = wrench_ext.norm();
      const double tau_ext_norm = tau_ext.norm();
      const double ee_speed = cart_velocity.head(3).norm();
      const int disturbance_flag = (force_norm >= opt.disturbance_threshold) ? 1 : 0;

      Eigen::Matrix<double, 7, 1> tau_cmd_no_coriolis = tau_d_calculated - coriolis;
      Eigen::Matrix<double, 6, 1> wrench_cmd =
          (JJt + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(jacobian * tau_cmd_no_coriolis);

      std::array<double, 7> tau_d_array{};
      for (int i = 0; i < 7; ++i) {
        tau_d_array[static_cast<size_t>(i)] = tau_d(i);
      }

      if (sample % static_cast<uint64_t>(opt.log_decimation) == 0) {
        log << time << ",joint_sine_246," << disturbance_flag << ','
            << position.x() << ',' << position.y() << ',' << position.z() << ','
            << position0.x() << ',' << position0.y() << ',' << position0.z() << ','
            << (position0.x() - position.x()) << ','
            << (position0.y() - position.y()) << ','
            << (position0.z() - position.z()) << ','
            << (position0 - position).norm() << ','
            << cart_velocity(0) << ',' << cart_velocity(1) << ',' << cart_velocity(2) << ','
            << ee_speed << ','
            << wrench_ext(0) << ',' << wrench_ext(1) << ',' << wrench_ext(2) << ','
            << wrench_ext(3) << ',' << wrench_ext(4) << ',' << wrench_ext(5) << ','
            << force_norm << ',' << wrench_norm << ',' << tau_ext_norm << ','
            << wrench_cmd(0) << ',' << wrench_cmd(1) << ',' << wrench_cmd(2) << ',';
        for (int i = 0; i < 7; ++i) log << q(i) << ',';
        for (int i = 0; i < 7; ++i) log << qd(i) << ',';
        for (int i = 0; i < 7; ++i) log << dq(i) << ',';
        for (int i = 0; i < 7; ++i) log << dqd(i) << ',';
        for (int i = 0; i < 7; ++i) log << (qd(i) - q(i)) << ',';
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

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int) {
  g_stop_requested = 1;
}

struct Options {
  std::string robot_ip = "172.16.0.2";
  std::string output = "exp1_joint_trajectory.csv";
  double duration = 18.0;
  double kp = 70.0;
  double kd = 12.0;
  double max_control_torque = 8.0;     // Nm, limit only the PD correction term.
  double max_delta_tau = 0.7;          // Nm/sample, torque-rate limit at 1 kHz.
  double max_force = 15.0;             // N, baseline should remain below this.
  double max_ext_torque_norm = 8.0;    // Nm, baseline should remain below this.
  bool understand_real_robot_risk = false;
  std::map<int, double> joint_deltas;  // 0-based joint index -> peak offset [rad].
};

struct LogSample {
  double t{};
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> q_ref{};
  std::array<double, 7> dq_ref{};
  std::array<double, 7> e_q{};
  std::array<double, 7> tau_cmd{};
  std::array<double, 7> tau_J{};
  std::array<double, 7> tau_ext{};
  std::array<double, 6> wrench_ext{};
  double ext_force_norm{};
  double ext_torque_norm{};
  double ee_x{};
  double ee_y{};
  double ee_z{};
  int stop_reason{};  // 0 normal, 1 force limit, 2 external torque limit, 3 ctrl-c.
};

template <size_t N>
Eigen::Matrix<double, static_cast<int>(N), 1> arrayToEigen(const std::array<double, N>& a) {
  Eigen::Matrix<double, static_cast<int>(N), 1> v;
  for (size_t i = 0; i < N; ++i) {
    v(static_cast<int>(i)) = a[i];
  }
  return v;
}

std::array<double, 7> eigenToArray7(const Eigen::Matrix<double, 7, 1>& v) {
  std::array<double, 7> a{};
  for (size_t i = 0; i < 7; ++i) {
    a[i] = v(static_cast<int>(i));
  }
  return a;
}

double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

std::array<double, 7> saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                                         const std::array<double, 7>& tau_J_d,
                                         double max_delta_tau) {
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; ++i) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + clamp(difference, -max_delta_tau, max_delta_tau);
  }
  return tau_d_saturated;
}

std::array<double, 7> limitVectorAbs(const std::array<double, 7>& input, double abs_limit) {
  std::array<double, 7> output{};
  for (size_t i = 0; i < 7; ++i) {
    output[i] = clamp(input[i], -abs_limit, abs_limit);
  }
  return output;
}

double normFirst3(const std::array<double, 6>& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double normLast3(const std::array<double, 6>& v) {
  return std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
}

double norm7(const std::array<double, 7>& v) {
  double s = 0.0;
  for (double x : v) {
    s += x * x;
  }
  return std::sqrt(s);
}

std::map<int, double> parseJointDeltas(const std::string& text) {
  // Format: "2:0.20,4:-0.16,6:0.20". Joint numbers are 1-based for the user.
  std::map<int, double> result;
  std::stringstream ss(text);
  std::string token;

  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      continue;
    }
    const auto colon = token.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("Invalid --joint-deltas token: " + token);
    }
    const int joint_number = std::stoi(token.substr(0, colon));
    const double delta = std::stod(token.substr(colon + 1));
    if (joint_number < 1 || joint_number > 7) {
      throw std::runtime_error("Joint number must be between 1 and 7 in --joint-deltas.");
    }
    result[joint_number - 1] = delta;
  }

  if (result.empty()) {
    throw std::runtime_error("--joint-deltas did not contain any valid joint command.");
  }
  return result;
}

void printUsage(const char* program) {
  std::cout << "Usage:\n"
            << "  " << program << " --robot-ip 172.16.0.2 --joint-deltas \"2:0.20,4:-0.16,6:0.20\" "
            << "--output ~/fr3_force_logs/exp1.csv --i-understand-real-robot-risk\n\n"
            << "Options:\n"
            << "  --robot-ip <ip>                 Default: 172.16.0.2\n"
            << "  --duration <seconds>            Default: 18\n"
            << "  --joint-deltas <list>           Example: \"2:0.20,4:-0.16,6:0.20\". Joints are 1-based.\n"
            << "  --kp <gain>                     Default: 70\n"
            << "  --kd <gain>                     Default: 12\n"
            << "  --max-control-torque <Nm>       Default: 8\n"
            << "  --max-delta-tau <Nm/sample>     Default: 0.7\n"
            << "  --max-force <N>                 Default: 15\n"
            << "  --max-ext-torque-norm <Nm>      Default: 8\n"
            << "  --output <csv path>             Default: exp1_joint_trajectory.csv\n"
            << "  --i-understand-real-robot-risk  Required safety acknowledgement\n";
}

Options parseArgs(int argc, char** argv) {
  Options opt;
  opt.joint_deltas = {{1, 0.20}, {3, -0.16}, {5, 0.20}};  // default: joints 2,4,6.

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--robot-ip") {
      opt.robot_ip = requireValue(arg);
    } else if (arg == "--duration") {
      opt.duration = std::stod(requireValue(arg));
    } else if (arg == "--joint-deltas") {
      opt.joint_deltas = parseJointDeltas(requireValue(arg));
    } else if (arg == "--kp") {
      opt.kp = std::stod(requireValue(arg));
    } else if (arg == "--kd") {
      opt.kd = std::stod(requireValue(arg));
    } else if (arg == "--max-control-torque") {
      opt.max_control_torque = std::stod(requireValue(arg));
    } else if (arg == "--max-delta-tau") {
      opt.max_delta_tau = std::stod(requireValue(arg));
    } else if (arg == "--max-force") {
      opt.max_force = std::stod(requireValue(arg));
    } else if (arg == "--max-ext-torque-norm") {
      opt.max_ext_torque_norm = std::stod(requireValue(arg));
    } else if (arg == "--output") {
      opt.output = requireValue(arg);
    } else if (arg == "--i-understand-real-robot-risk") {
      opt.understand_real_robot_risk = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (!opt.understand_real_robot_risk) {
    throw std::runtime_error("Refusing to run on the real robot. Add --i-understand-real-robot-risk.");
  }
  if (opt.duration <= 2.0) {
    throw std::runtime_error("Duration is too short for a smooth baseline trajectory. Use > 2 s.");
  }
  if (opt.kp <= 0.0 || opt.kd < 0.0) {
    throw std::runtime_error("Invalid gains. Use kp > 0 and kd >= 0.");
  }
  if (opt.max_control_torque <= 0.0 || opt.max_delta_tau <= 0.0) {
    throw std::runtime_error("Torque limits must be positive.");
  }
  return opt;
}

void writeCsv(const std::string& path, const std::vector<LogSample>& log) {
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("Could not open output CSV: " + path);
  }

  file << std::fixed << std::setprecision(9);
  file << "t";
  for (int i = 1; i <= 7; ++i) file << ",q" << i;
  for (int i = 1; i <= 7; ++i) file << ",dq" << i;
  for (int i = 1; i <= 7; ++i) file << ",q_ref" << i;
  for (int i = 1; i <= 7; ++i) file << ",dq_ref" << i;
  for (int i = 1; i <= 7; ++i) file << ",e_q" << i;
  for (int i = 1; i <= 7; ++i) file << ",tau_cmd" << i;
  for (int i = 1; i <= 7; ++i) file << ",tau_J" << i;
  for (int i = 1; i <= 7; ++i) file << ",tau_ext" << i;
  file << ",Fx_ext,Fy_ext,Fz_ext,Tx_ext,Ty_ext,Tz_ext";
  file << ",ext_force_norm,ext_wrench_torque_norm,ext_joint_torque_norm";
  file << ",ee_x,ee_y,ee_z,stop_reason\n";

  for (const auto& s : log) {
    file << s.t;
    for (double x : s.q) file << ',' << x;
    for (double x : s.dq) file << ',' << x;
    for (double x : s.q_ref) file << ',' << x;
    for (double x : s.dq_ref) file << ',' << x;
    for (double x : s.e_q) file << ',' << x;
    for (double x : s.tau_cmd) file << ',' << x;
    for (double x : s.tau_J) file << ',' << x;
    for (double x : s.tau_ext) file << ',' << x;
    for (double x : s.wrench_ext) file << ',' << x;
    file << ',' << s.ext_force_norm;
    file << ',' << normLast3(s.wrench_ext);
    file << ',' << s.ext_torque_norm;
    file << ',' << s.ee_x << ',' << s.ee_y << ',' << s.ee_z;
    file << ',' << s.stop_reason << '\n';
  }
}

void printExperimentSummary(const Options& opt) {
  std::cout << "\nExperiment 1: Baseline Free-Space Joint Trajectory\n"
            << "Robot IP: " << opt.robot_ip << "\n"
            << "Duration: " << opt.duration << " s\n"
            << "PD gains: Kp=" << opt.kp << ", Kd=" << opt.kd << "\n"
            << "Moving joints, peak offsets:\n";
  for (const auto& [idx, delta] : opt.joint_deltas) {
    std::cout << "  joint " << (idx + 1) << ": " << delta << " rad\n";
  }
  std::cout << "Output: " << opt.output << "\n\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signalHandler);

  try {
    const Options opt = parseArgs(argc, argv);
    printExperimentSummary(opt);

    franka::Robot robot(opt.robot_ip);

    // Conservative thresholds for a free-space baseline experiment.
    // Adjust only if your lab supervisor has configured different safe values.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{15.0, 15.0, 15.0, 12.0, 12.0, 12.0}},
        {{15.0, 15.0, 15.0, 12.0, 12.0, 12.0}},
        {{15.0, 15.0, 15.0, 12.0, 12.0, 12.0}},
        {{15.0, 15.0, 15.0, 12.0, 12.0, 12.0}});

    franka::Model model = robot.loadModel();
    const franka::RobotState initial_state = robot.readOnce();
    const Eigen::Matrix<double, 7, 1> q0 = arrayToEigen(initial_state.q);

    Eigen::Matrix<double, 7, 1> q_peak_offset = Eigen::Matrix<double, 7, 1>::Zero();
    for (const auto& [idx, delta] : opt.joint_deltas) {
      q_peak_offset(idx) = delta;
    }

    std::vector<LogSample> log;
    log.reserve(static_cast<size_t>(opt.duration * 1000.0) + 2000);

    double elapsed = 0.0;
    int stop_reason = 0;

    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
      elapsed += period.toSec();
      const double t = std::min(elapsed, opt.duration);

      // Smooth one-cycle reference:
      // q_ref(0)=q0, q_ref(T/2)=q0+delta, q_ref(T)=q0, with zero velocity at start/end.
      const double s = 0.5 * (1.0 - std::cos(2.0 * kPi * t / opt.duration));
      const double ds = (kPi / opt.duration) * std::sin(2.0 * kPi * t / opt.duration);

      const Eigen::Matrix<double, 7, 1> q = arrayToEigen(state.q);
      const Eigen::Matrix<double, 7, 1> dq = arrayToEigen(state.dq);
      const Eigen::Matrix<double, 7, 1> q_ref = q0 + s * q_peak_offset;
      const Eigen::Matrix<double, 7, 1> dq_ref = ds * q_peak_offset;
      const Eigen::Matrix<double, 7, 1> e_q = q_ref - q;

      Eigen::Matrix<double, 7, 1> tau_task =
          opt.kp * e_q + opt.kd * (dq_ref - dq);
      for (int i = 0; i < 7; ++i) {
        tau_task(i) = clamp(tau_task(i), -opt.max_control_torque, opt.max_control_torque);
      }

      const Eigen::Matrix<double, 7, 1> coriolis = arrayToEigen(model.coriolis(state));
      const Eigen::Matrix<double, 7, 1> tau_cmd_eigen = coriolis + tau_task;
      std::array<double, 7> tau_cmd = eigenToArray7(tau_cmd_eigen);
      tau_cmd = saturateTorqueRate(tau_cmd, state.tau_J_d, opt.max_delta_tau);

      const double ext_force_norm = normFirst3(state.O_F_ext_hat_K);
      const double ext_torque_norm = norm7(state.tau_ext_hat_filtered);

      if (t > 0.5 && ext_force_norm > opt.max_force) {
        stop_reason = 1;
      }
      if (t > 0.5 && ext_torque_norm > opt.max_ext_torque_norm) {
        stop_reason = 2;
      }
      if (g_stop_requested) {
        stop_reason = 3;
      }

      if (log.size() < log.capacity()) {
        LogSample sample;
        sample.t = elapsed;
        sample.q = state.q;
        sample.dq = state.dq;
        sample.q_ref = eigenToArray7(q_ref);
        sample.dq_ref = eigenToArray7(dq_ref);
        sample.e_q = eigenToArray7(e_q);
        sample.tau_cmd = tau_cmd;
        sample.tau_J = state.tau_J;
        sample.tau_ext = state.tau_ext_hat_filtered;
        sample.wrench_ext = state.O_F_ext_hat_K;
        sample.ext_force_norm = ext_force_norm;
        sample.ext_torque_norm = ext_torque_norm;
        sample.ee_x = state.O_T_EE[12];
        sample.ee_y = state.O_T_EE[13];
        sample.ee_z = state.O_T_EE[14];
        sample.stop_reason = stop_reason;
        log.push_back(sample);
      }

      if (stop_reason != 0 || elapsed >= opt.duration) {
        return franka::MotionFinished(franka::Torques(tau_cmd));
      }
      return franka::Torques(tau_cmd);
    });

    writeCsv(opt.output, log);

    std::cout << "\nFinished. Wrote " << log.size() << " samples to:\n  " << opt.output << "\n";
    if (stop_reason == 1) {
      std::cout << "Stopped because external force norm exceeded --max-force.\n";
    } else if (stop_reason == 2) {
      std::cout << "Stopped because external joint torque norm exceeded --max-ext-torque-norm.\n";
    } else if (stop_reason == 3) {
      std::cout << "Stopped by Ctrl+C.\n";
    } else {
      std::cout << "Normal completion.\n";
    }

    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka exception: " << e.what() << '\n';
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }
}

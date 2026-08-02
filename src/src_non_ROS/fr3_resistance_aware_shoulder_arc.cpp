#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;

constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int) {
  g_stop_requested = 1;
}

enum class RehabMode {
  kCalibration,
  kMove,
  kSlowAssist,
  kCompliantHold,
  kFinalHold,
};

const char* modeName(RehabMode mode) {
  switch (mode) {
    case RehabMode::kCalibration: return "CALIBRATION";
    case RehabMode::kMove: return "MOVE";
    case RehabMode::kSlowAssist: return "SLOW_ASSIST";
    case RehabMode::kCompliantHold: return "COMPLIANT_HOLD";
    case RehabMode::kFinalHold: return "FINAL_HOLD";
  }
  return "UNKNOWN";
}

struct Config {
  std::string robot_ip{"172.16.0.2"};

  // The handle is already centred between the fingers at the current pose.
  double handle_width_m{0.025};
  double gripper_speed_mps{0.010};
  double gripper_force_n{5.0};
  double grasp_epsilon_inner_m{0.004};
  double grasp_epsilon_outer_m{0.004};

  // Human shoulder centre relative to the current handle position, expressed
  // in the FR3 base frame. Measure this vector for the actual setup.
  Eigen::Vector3d shoulder_center_offset{-0.550, 0.0, 0.0};
  Eigen::Vector3d rotation_axis{0.0, 1.0, 0.0};

  // About 21.3 deg gives roughly 0.20 m vertical rise for radius 0.55 m.
  double amplitude_deg{-21.3};
  double half_cycle_s{5.0};
  int repetitions{5};
  double settle_s{2.0};
  double final_hold_s{1.0};

  // Virtual-channel Cartesian impedance.
  double stiffness_tangent_npm{35.0};
  double stiffness_radial_npm{100.0};
  double stiffness_axis_npm{100.0};
  double hold_tangent_stiffness_npm{0.0};
  double rotational_stiffness_nmprad{1.5};
  double vertical_support_force_n{0.0};

  // Bounded assist-as-needed force along the movement tangent.
  double assist_stiffness_npm{100.0};
  double assist_deadband_m{0.005};
  double assist_force_max_n{4.0};

  // Resistance-aware progression. RobotState::O_F_ext_hat_K is expressed in
  // the base frame. Official libfranka semantics make force applied by the
  // robot to the environment positive, so +1 is the default projection sign.
  double force_projection_sign{1.0};
  double force_free_n{1.5};
  double force_pause_n{5.0};
  double force_resume_n{2.5};
  double force_filter_tau_s{0.10};
  double pause_persistence_s{0.15};
  double resume_persistence_s{0.25};
  double resistance_hold_timeout_s{3.0};
  double speed_scale_rate_per_s{2.0};

  // Hard software stops. These are engineering placeholders, not clinical
  // limits and not substitutes for Watchman and a validated risk assessment.
  double force_delta_hard_stop_n{12.0};
  double force_absolute_hard_stop_n{40.0};
  double external_joint_torque_norm_stop_nm{18.0};
  double perpendicular_error_stop_m{0.050};
  double perpendicular_error_persistence_s{0.20};
  double tangential_error_hard_stop_m{0.100};
  double tangential_error_persistence_s{0.30};
  double ee_speed_stop_mps{0.130};

  // Real-time-safe logging. Samples are copied into a preallocated memory
  // buffer inside the 1 kHz callback and written to disk only after control ends.
  std::string log_root{"logs"};
  double log_rate_hz{100.0};
  double max_log_duration_s{600.0};

  bool risk_acknowledged{false};
};

struct ArcSample {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d radial{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitY()};
};

struct RunSummary {
  int stop_reason{0};
  RehabMode final_mode{RehabMode::kCalibration};
  int completed_half_cycles{0};
  double max_absolute_force_n{0.0};
  double max_delta_force_n{0.0};
  double max_opposing_force_n{0.0};
  double max_assist_force_n{0.0};
  double max_perpendicular_error_m{0.0};
  double max_tangential_error_m{0.0};
  double max_ee_speed_mps{0.0};
  double minimum_speed_scale{1.0};
  int hold_entries{0};
};

struct LogSample {
  double time_s{0.0};
  double dt_s{0.0};
  int mode{0};
  int half_cycle_index{0};
  int completed_half_cycles{0};
  double phase{0.0};
  double speed_scale{0.0};

  std::array<double, 3> position{};
  std::array<double, 3> desired_position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> desired_velocity{};
  std::array<double, 3> position_error{};

  double tracking_error_norm_m{0.0};
  double tangential_error_m{0.0};
  double radial_error_m{0.0};
  double axis_error_m{0.0};
  double perpendicular_error_m{0.0};

  std::array<double, 3> external_force{};
  std::array<double, 3> delta_force{};
  std::array<double, 3> commanded_force{};
  double absolute_force_n{0.0};
  double delta_force_norm_n{0.0};
  double projected_force_n{0.0};
  double opposing_force_n{0.0};
  double filtered_opposing_force_n{0.0};
  double assist_force_n{0.0};
  double ee_speed_mps{0.0};
  double external_joint_torque_norm_nm{0.0};

  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau_measured{};
  std::array<double, 7> tau_commanded{};
  std::array<double, 7> tau_external{};
};

std::string timestampString() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string stopReasonText(int reason) {
  switch (reason) {
    case 0: return "normal completion";
    case 1: return "Ctrl+C";
    case 2: return "absolute-force hard threshold";
    case 3: return "force-change hard threshold";
    case 4: return "external-joint-torque threshold";
    case 5: return "perpendicular path-error threshold";
    case 6: return "tangential hard-error threshold";
    case 7: return "end-effector-speed threshold";
    case 8: return "resistance remained high during compliant hold";
    default: return "unspecified";
  }
}

std::filesystem::path createLogDirectory(const Config& cfg) {
  const std::filesystem::path root(cfg.log_root);
  const std::filesystem::path directory =
      root / ("shoulder_resistance_aware_" + timestampString());
  std::filesystem::create_directories(directory);
  return directory;
}

void writeRunDataCsv(const std::filesystem::path& path,
                     const std::vector<LogSample>& samples,
                     std::size_t sample_count) {
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("Could not create log file: " + path.string());
  }
  file << std::setprecision(10);
  file << "time_s,dt_s,mode,half_cycle_index,completed_half_cycles,phase,speed_scale,";
  file << "x_m,y_m,z_m,x_des_m,y_des_m,z_des_m,";
  file << "vx_mps,vy_mps,vz_mps,vx_des_mps,vy_des_mps,vz_des_mps,";
  file << "ex_m,ey_m,ez_m,tracking_error_norm_m,tangential_error_m,radial_error_m,axis_error_m,perpendicular_error_m,";
  file << "fx_ext_n,fy_ext_n,fz_ext_n,fx_delta_n,fy_delta_n,fz_delta_n,";
  file << "fx_cmd_n,fy_cmd_n,fz_cmd_n,absolute_force_n,delta_force_norm_n,";
  file << "projected_force_n,opposing_force_n,filtered_opposing_force_n,assist_force_n,";
  file << "ee_speed_mps,external_joint_torque_norm_nm";
  for (int j = 1; j <= 7; ++j) file << ",q" << j << "_rad";
  for (int j = 1; j <= 7; ++j) file << ",dq" << j << "_radps";
  for (int j = 1; j <= 7; ++j) file << ",tau_measured_j" << j << "_nm";
  for (int j = 1; j <= 7; ++j) file << ",tau_commanded_j" << j << "_nm";
  for (int j = 1; j <= 7; ++j) file << ",tau_external_j" << j << "_nm";
  file << '\n';

  for (std::size_t i = 0; i < sample_count; ++i) {
    const LogSample& row = samples[i];
    file << row.time_s << ',' << row.dt_s << ',' << row.mode << ','
         << row.half_cycle_index << ',' << row.completed_half_cycles << ','
         << row.phase << ',' << row.speed_scale;
    for (double value : row.position) file << ',' << value;
    for (double value : row.desired_position) file << ',' << value;
    for (double value : row.velocity) file << ',' << value;
    for (double value : row.desired_velocity) file << ',' << value;
    for (double value : row.position_error) file << ',' << value;
    file << ',' << row.tracking_error_norm_m
         << ',' << row.tangential_error_m
         << ',' << row.radial_error_m
         << ',' << row.axis_error_m
         << ',' << row.perpendicular_error_m;
    for (double value : row.external_force) file << ',' << value;
    for (double value : row.delta_force) file << ',' << value;
    for (double value : row.commanded_force) file << ',' << value;
    file << ',' << row.absolute_force_n
         << ',' << row.delta_force_norm_n
         << ',' << row.projected_force_n
         << ',' << row.opposing_force_n
         << ',' << row.filtered_opposing_force_n
         << ',' << row.assist_force_n
         << ',' << row.ee_speed_mps
         << ',' << row.external_joint_torque_norm_nm;
    for (double value : row.q) file << ',' << value;
    for (double value : row.dq) file << ',' << value;
    for (double value : row.tau_measured) file << ',' << value;
    for (double value : row.tau_commanded) file << ',' << value;
    for (double value : row.tau_external) file << ',' << value;
    file << '\n';
  }
}

void writeConfigCsv(const std::filesystem::path& path, const Config& cfg) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("Could not create config file: " + path.string());
  file << "parameter,value\n";
  file << "robot_ip," << cfg.robot_ip << '\n';
  file << "handle_width_m," << cfg.handle_width_m << '\n';
  file << "gripper_force_n," << cfg.gripper_force_n << '\n';
  file << "shoulder_dx_m," << cfg.shoulder_center_offset.x() << '\n';
  file << "shoulder_dy_m," << cfg.shoulder_center_offset.y() << '\n';
  file << "shoulder_dz_m," << cfg.shoulder_center_offset.z() << '\n';
  file << "axis_x," << cfg.rotation_axis.x() << '\n';
  file << "axis_y," << cfg.rotation_axis.y() << '\n';
  file << "axis_z," << cfg.rotation_axis.z() << '\n';
  file << "amplitude_deg," << cfg.amplitude_deg << '\n';
  file << "half_cycle_s," << cfg.half_cycle_s << '\n';
  file << "repetitions," << cfg.repetitions << '\n';
  file << "stiffness_tangent_npm," << cfg.stiffness_tangent_npm << '\n';
  file << "stiffness_radial_npm," << cfg.stiffness_radial_npm << '\n';
  file << "stiffness_axis_npm," << cfg.stiffness_axis_npm << '\n';
  file << "assist_stiffness_npm," << cfg.assist_stiffness_npm << '\n';
  file << "assist_deadband_m," << cfg.assist_deadband_m << '\n';
  file << "assist_force_max_n," << cfg.assist_force_max_n << '\n';
  file << "force_free_n," << cfg.force_free_n << '\n';
  file << "force_resume_n," << cfg.force_resume_n << '\n';
  file << "force_pause_n," << cfg.force_pause_n << '\n';
  file << "log_rate_hz," << cfg.log_rate_hz << '\n';
}

void writeSummaryCsv(const std::filesystem::path& path,
                     const RunSummary& summary,
                     std::size_t sample_count,
                     bool log_overflow) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("Could not create summary file: " + path.string());
  file << "metric,value\n";
  file << "stop_reason_code," << summary.stop_reason << '\n';
  file << "stop_reason," << stopReasonText(summary.stop_reason) << '\n';
  file << "final_mode," << modeName(summary.final_mode) << '\n';
  file << "completed_half_cycles," << summary.completed_half_cycles << '\n';
  file << "resistance_hold_entries," << summary.hold_entries << '\n';
  file << "max_absolute_external_force_n," << summary.max_absolute_force_n << '\n';
  file << "max_force_change_from_bias_n," << summary.max_delta_force_n << '\n';
  file << "max_filtered_opposing_force_n," << summary.max_opposing_force_n << '\n';
  file << "max_bounded_assist_force_n," << summary.max_assist_force_n << '\n';
  file << "max_perpendicular_error_m," << summary.max_perpendicular_error_m << '\n';
  file << "max_absolute_tangential_error_m," << summary.max_tangential_error_m << '\n';
  file << "max_end_effector_speed_mps," << summary.max_ee_speed_mps << '\n';
  file << "minimum_speed_scale," << summary.minimum_speed_scale << '\n';
  file << "logged_samples," << sample_count << '\n';
  file << "log_overflow," << (log_overflow ? 1 : 0) << '\n';
}

double deg2rad(double degrees) {
  return degrees * kPi / 180.0;
}

double minJerk(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4) + 6.0 * std::pow(u, 5);
}

double minJerkDerivative(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return 30.0 * std::pow(u, 2) - 60.0 * std::pow(u, 3) + 30.0 * std::pow(u, 4);
}

double parseDouble(const std::string& text, const std::string& option) {
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
      throw std::invalid_argument("invalid");
    }
    return value;
  } catch (...) {
    throw std::runtime_error("Invalid numerical value for " + option + ": " + text);
  }
}

int parseInt(const std::string& text, const std::string& option) {
  try {
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size()) {
      throw std::invalid_argument("invalid");
    }
    return value;
  } catch (...) {
    throw std::runtime_error("Invalid integer value for " + option + ": " + text);
  }
}

bool promptExact(const std::string& message, const std::string& expected) {
  std::cout << message;
  std::string input;
  std::getline(std::cin, input);
  return input == expected;
}

void printUsage(const char* executable, const Config& cfg) {
  std::cout
      << "Usage:\n  " << executable
      << " --i-understand-real-robot-human-contact-risk [options]\n\n"
      << "The current end-effector pose is the lower exercise position. The 2.5 cm\n"
      << "handle must already be centred between the gripper fingers. The program\n"
      << "grasps the handle, performs a resistance-aware shoulder arc, and returns.\n\n"
      << "Geometry and exercise:\n"
      << "  --robot-ip IP                         default " << cfg.robot_ip << "\n"
      << "  --handle-width M                      default " << cfg.handle_width_m << "\n"
      << "  --gripper-speed MPS                   default " << cfg.gripper_speed_mps << "\n"
      << "  --gripper-force N                     default " << cfg.gripper_force_n << "\n"
      << "  --shoulder-dx M                       default " << cfg.shoulder_center_offset.x() << "\n"
      << "  --shoulder-dy M                       default " << cfg.shoulder_center_offset.y() << "\n"
      << "  --shoulder-dz M                       default " << cfg.shoulder_center_offset.z() << "\n"
      << "  --axis-x VALUE                        default " << cfg.rotation_axis.x() << "\n"
      << "  --axis-y VALUE                        default " << cfg.rotation_axis.y() << "\n"
      << "  --axis-z VALUE                        default " << cfg.rotation_axis.z() << "\n"
      << "  --amplitude-deg DEG                   default " << cfg.amplitude_deg << "\n"
      << "  --half-cycle S                        default " << cfg.half_cycle_s << "\n"
      << "  --repetitions N                       default " << cfg.repetitions << "\n\n"
      << "Impedance and assistance:\n"
      << "  --stiffness-tangent NPM               default " << cfg.stiffness_tangent_npm << "\n"
      << "  --stiffness-radial NPM                default " << cfg.stiffness_radial_npm << "\n"
      << "  --stiffness-axis NPM                  default " << cfg.stiffness_axis_npm << "\n"
      << "  --hold-tangent-stiffness NPM          default " << cfg.hold_tangent_stiffness_npm << "\n"
      << "  --assist-stiffness NPM                default " << cfg.assist_stiffness_npm << "\n"
      << "  --assist-deadband M                   default " << cfg.assist_deadband_m << "\n"
      << "  --assist-force-max N                  default " << cfg.assist_force_max_n << "\n"
      << "  --vertical-support-force N            default " << cfg.vertical_support_force_n << "\n\n"
      << "Resistance response:\n"
      << "  --force-sign VALUE                    default " << cfg.force_projection_sign << " (+1 or -1)\n"
      << "  --force-free N                        default " << cfg.force_free_n << "\n"
      << "  --force-pause N                       default " << cfg.force_pause_n << "\n"
      << "  --force-resume N                      default " << cfg.force_resume_n << "\n"
      << "  --force-filter-tau S                  default " << cfg.force_filter_tau_s << "\n"
      << "  --pause-persistence S                 default " << cfg.pause_persistence_s << "\n"
      << "  --resume-persistence S                default " << cfg.resume_persistence_s << "\n"
      << "  --hold-timeout S                      default " << cfg.resistance_hold_timeout_s << "\n"
      << "  --speed-scale-rate PER_S              default " << cfg.speed_scale_rate_per_s << "\n\n"
      << "Hard software stops:\n"
      << "  --force-delta-hard-stop N             default " << cfg.force_delta_hard_stop_n << "\n"
      << "  --force-absolute-hard-stop N          default " << cfg.force_absolute_hard_stop_n << "\n"
      << "  --perpendicular-error-stop M          default " << cfg.perpendicular_error_stop_m << "\n"
      << "  --tangential-error-hard-stop M        default " << cfg.tangential_error_hard_stop_m << "\n\n"
      << "Logging:\n"
      << "  --log-dir PATH                        default " << cfg.log_root << "\n"
      << "  --log-rate HZ                         default " << cfg.log_rate_hz << "\n"
      << "  --max-log-duration S                  default " << cfg.max_log_duration_s << "\n"
      << "  --help\n";
}

Config parseArguments(int argc, char** argv, Config cfg) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const std::string& option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value after " + option);
      }
      return argv[++i];
    };

    if (arg == "--robot-ip") {
      cfg.robot_ip = next(arg);
    } else if (arg == "--handle-width") {
      cfg.handle_width_m = parseDouble(next(arg), arg);
    } else if (arg == "--gripper-speed") {
      cfg.gripper_speed_mps = parseDouble(next(arg), arg);
    } else if (arg == "--gripper-force") {
      cfg.gripper_force_n = parseDouble(next(arg), arg);
    } else if (arg == "--shoulder-dx") {
      cfg.shoulder_center_offset.x() = parseDouble(next(arg), arg);
    } else if (arg == "--shoulder-dy") {
      cfg.shoulder_center_offset.y() = parseDouble(next(arg), arg);
    } else if (arg == "--shoulder-dz") {
      cfg.shoulder_center_offset.z() = parseDouble(next(arg), arg);
    } else if (arg == "--axis-x") {
      cfg.rotation_axis.x() = parseDouble(next(arg), arg);
    } else if (arg == "--axis-y") {
      cfg.rotation_axis.y() = parseDouble(next(arg), arg);
    } else if (arg == "--axis-z") {
      cfg.rotation_axis.z() = parseDouble(next(arg), arg);
    } else if (arg == "--amplitude-deg") {
      cfg.amplitude_deg = parseDouble(next(arg), arg);
    } else if (arg == "--half-cycle") {
      cfg.half_cycle_s = parseDouble(next(arg), arg);
    } else if (arg == "--repetitions") {
      cfg.repetitions = parseInt(next(arg), arg);
    } else if (arg == "--stiffness-tangent") {
      cfg.stiffness_tangent_npm = parseDouble(next(arg), arg);
    } else if (arg == "--stiffness-radial") {
      cfg.stiffness_radial_npm = parseDouble(next(arg), arg);
    } else if (arg == "--stiffness-axis") {
      cfg.stiffness_axis_npm = parseDouble(next(arg), arg);
    } else if (arg == "--hold-tangent-stiffness") {
      cfg.hold_tangent_stiffness_npm = parseDouble(next(arg), arg);
    } else if (arg == "--assist-stiffness") {
      cfg.assist_stiffness_npm = parseDouble(next(arg), arg);
    } else if (arg == "--assist-deadband") {
      cfg.assist_deadband_m = parseDouble(next(arg), arg);
    } else if (arg == "--assist-force-max") {
      cfg.assist_force_max_n = parseDouble(next(arg), arg);
    } else if (arg == "--vertical-support-force") {
      cfg.vertical_support_force_n = parseDouble(next(arg), arg);
    } else if (arg == "--force-sign") {
      cfg.force_projection_sign = parseDouble(next(arg), arg);
    } else if (arg == "--force-free") {
      cfg.force_free_n = parseDouble(next(arg), arg);
    } else if (arg == "--force-pause") {
      cfg.force_pause_n = parseDouble(next(arg), arg);
    } else if (arg == "--force-resume") {
      cfg.force_resume_n = parseDouble(next(arg), arg);
    } else if (arg == "--force-filter-tau") {
      cfg.force_filter_tau_s = parseDouble(next(arg), arg);
    } else if (arg == "--pause-persistence") {
      cfg.pause_persistence_s = parseDouble(next(arg), arg);
    } else if (arg == "--resume-persistence") {
      cfg.resume_persistence_s = parseDouble(next(arg), arg);
    } else if (arg == "--hold-timeout") {
      cfg.resistance_hold_timeout_s = parseDouble(next(arg), arg);
    } else if (arg == "--speed-scale-rate") {
      cfg.speed_scale_rate_per_s = parseDouble(next(arg), arg);
    } else if (arg == "--force-delta-hard-stop") {
      cfg.force_delta_hard_stop_n = parseDouble(next(arg), arg);
    } else if (arg == "--force-absolute-hard-stop") {
      cfg.force_absolute_hard_stop_n = parseDouble(next(arg), arg);
    } else if (arg == "--perpendicular-error-stop") {
      cfg.perpendicular_error_stop_m = parseDouble(next(arg), arg);
    } else if (arg == "--tangential-error-hard-stop") {
      cfg.tangential_error_hard_stop_m = parseDouble(next(arg), arg);
    } else if (arg == "--log-dir") {
      cfg.log_root = next(arg);
    } else if (arg == "--log-rate") {
      cfg.log_rate_hz = parseDouble(next(arg), arg);
    } else if (arg == "--max-log-duration") {
      cfg.max_log_duration_s = parseDouble(next(arg), arg);
    } else if (arg == "--i-understand-real-robot-human-contact-risk") {
      cfg.risk_acknowledged = true;
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0], cfg);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  if (cfg.handle_width_m <= 0.0 || cfg.handle_width_m > 0.080) {
    throw std::runtime_error("Handle width must be in (0, 0.080] m.");
  }
  if (cfg.gripper_speed_mps <= 0.0 || cfg.gripper_speed_mps > 0.030) {
    throw std::runtime_error("Gripper speed must be in (0, 0.030] m/s.");
  }
  if (cfg.gripper_force_n <= 0.0 || cfg.gripper_force_n > 15.0) {
    throw std::runtime_error("This prototype restricts gripper force to (0, 15] N.");
  }
  if (cfg.rotation_axis.norm() < 1e-9) {
    throw std::runtime_error("Rotation axis cannot be zero.");
  }

  const double radius = cfg.shoulder_center_offset.norm();
  if (radius < 0.25 || radius > 0.80) {
    throw std::runtime_error("Shoulder-to-handle radius must be in [0.25, 0.80] m.");
  }
  const Eigen::Vector3d axis = cfg.rotation_axis.normalized();
  const Eigen::Vector3d initial_radius = -cfg.shoulder_center_offset;
  if (std::abs(axis.dot(initial_radius.normalized())) > 0.10) {
    throw std::runtime_error(
        "For this planar shoulder arc, the rotation axis must be approximately "
        "perpendicular to the shoulder-to-handle radius.");
  }
  if (std::abs(cfg.amplitude_deg) > 35.0) {
    throw std::runtime_error("This prototype refuses amplitudes above 35 degrees.");
  }
  if (cfg.half_cycle_s < 2.0) {
    throw std::runtime_error("Half-cycle must be at least 2.0 s.");
  }
  if (cfg.repetitions < 1 || cfg.repetitions > 10) {
    throw std::runtime_error("Repetitions must be between 1 and 10.");
  }

  auto stiffness_valid = [](double value) { return value >= 0.0 && value <= 180.0; };
  if (!stiffness_valid(cfg.stiffness_tangent_npm) ||
      !stiffness_valid(cfg.stiffness_radial_npm) ||
      !stiffness_valid(cfg.stiffness_axis_npm) ||
      !stiffness_valid(cfg.hold_tangent_stiffness_npm)) {
    throw std::runtime_error("Translational stiffness values must be in [0, 180] N/m.");
  }
  if (cfg.stiffness_radial_npm < 20.0 || cfg.stiffness_axis_npm < 20.0) {
    throw std::runtime_error("Radial and axis stiffness must each be at least 20 N/m.");
  }
  if (cfg.rotational_stiffness_nmprad < 0.0 || cfg.rotational_stiffness_nmprad > 10.0) {
    throw std::runtime_error("Rotational stiffness must be in [0, 10] Nm/rad.");
  }
  if (cfg.vertical_support_force_n < 0.0 || cfg.vertical_support_force_n > 8.0) {
    throw std::runtime_error("Vertical support force must be in [0, 8] N.");
  }
  if (cfg.assist_stiffness_npm < 0.0 || cfg.assist_stiffness_npm > 200.0 ||
      cfg.assist_force_max_n < 0.0 || cfg.assist_force_max_n > 8.0 ||
      cfg.assist_deadband_m < 0.0 || cfg.assist_deadband_m > 0.030) {
    throw std::runtime_error("Invalid assist-as-needed parameters.");
  }
  if (std::abs(std::abs(cfg.force_projection_sign) - 1.0) > 1e-9) {
    throw std::runtime_error("--force-sign must be exactly +1 or -1.");
  }
  if (cfg.force_free_n < 0.0 || cfg.force_pause_n <= cfg.force_free_n ||
      cfg.force_resume_n < cfg.force_free_n || cfg.force_resume_n >= cfg.force_pause_n) {
    throw std::runtime_error(
        "Require 0 <= force-free <= force-resume < force-pause.");
  }
  if (cfg.force_pause_n >= cfg.force_delta_hard_stop_n) {
    throw std::runtime_error("force-pause must be below force-delta-hard-stop.");
  }
  if (cfg.force_filter_tau_s <= 0.0 || cfg.force_filter_tau_s > 1.0 ||
      cfg.pause_persistence_s < 0.02 || cfg.pause_persistence_s > 1.0 ||
      cfg.resume_persistence_s < 0.02 || cfg.resume_persistence_s > 2.0 ||
      cfg.resistance_hold_timeout_s < 0.5 || cfg.resistance_hold_timeout_s > 20.0 ||
      cfg.speed_scale_rate_per_s <= 0.0 || cfg.speed_scale_rate_per_s > 20.0) {
    throw std::runtime_error("Invalid resistance timing/filter parameters.");
  }
  if (cfg.perpendicular_error_stop_m < 0.020 || cfg.perpendicular_error_stop_m > 0.100 ||
      cfg.tangential_error_hard_stop_m < 0.040 ||
      cfg.tangential_error_hard_stop_m > 0.150) {
    throw std::runtime_error("Invalid path-error thresholds.");
  }

  if (cfg.log_root.empty()) {
    throw std::runtime_error("Log directory cannot be empty.");
  }
  if (cfg.log_rate_hz < 1.0 || cfg.log_rate_hz > 500.0) {
    throw std::runtime_error("Log rate must be in [1, 500] Hz.");
  }
  if (cfg.max_log_duration_s < 10.0 || cfg.max_log_duration_s > 3600.0) {
    throw std::runtime_error("Maximum log duration must be in [10, 3600] s.");
  }

  const double peak_speed =
      1.875 * radius * std::abs(deg2rad(cfg.amplitude_deg)) / cfg.half_cycle_s;
  if (peak_speed > 0.90 * cfg.ee_speed_stop_mps) {
    throw std::runtime_error(
        "Requested radius/amplitude/half-cycle predicts excessive speed. "
        "Increase --half-cycle or reduce --amplitude-deg.");
  }

  return cfg;
}

ArcSample sampleShoulderArc(const Config& cfg,
                            const Eigen::Vector3d& initial_position,
                            int half_cycle_index,
                            double phase,
                            double phase_dot) {
  const Eigen::Vector3d axis = cfg.rotation_axis.normalized();
  const Eigen::Vector3d center = initial_position + cfg.shoulder_center_offset;
  const Eigen::Vector3d radius0 = initial_position - center;
  const double amplitude = deg2rad(cfg.amplitude_deg);

  phase = std::clamp(phase, 0.0, 1.0);
  const bool upward_segment = (half_cycle_index % 2) == 0;
  const double start_phi = upward_segment ? 0.0 : amplitude;
  const double end_phi = upward_segment ? amplitude : 0.0;
  const double delta_phi = end_phi - start_phi;

  const double s = minJerk(phase);
  const double s_dot = minJerkDerivative(phase) * phase_dot;
  const double phi = start_phi + delta_phi * s;
  const double phi_dot = delta_phi * s_dot;

  const Eigen::Vector3d radius = Eigen::AngleAxisd(phi, axis) * radius0;
  const Eigen::Vector3d radial = radius.normalized();
  const Eigen::Vector3d geometric_tangent = axis.cross(radial).normalized();
  const double direction_sign = delta_phi >= 0.0 ? 1.0 : -1.0;

  ArcSample sample;
  sample.position = center + radius;
  sample.velocity = phi_dot * axis.cross(radius);
  sample.radial = radial;
  sample.axis = axis;
  sample.tangent = direction_sign * geometric_tangent;
  return sample;
}

RunSummary runShoulderExercise(franka::Robot& robot, const Config& cfg) {
  franka::Model model = robot.loadModel();
  const franka::RobotState initial_state = robot.readOnce();
  const Eigen::Affine3d initial_transform(
      Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  const Eigen::Vector3d initial_position = initial_transform.translation();
  const Eigen::Quaterniond desired_orientation(initial_transform.rotation());

  const ArcSample top_sample = sampleShoulderArc(cfg, initial_position, 0, 1.0, 0.0);
  std::cout << std::fixed << std::setprecision(3)
            << "Predicted top-point displacement [m]: "
            << (top_sample.position - initial_position).transpose() << "\n";

  RunSummary summary;
  const int total_half_cycles = 2 * cfg.repetitions;

  const std::filesystem::path log_directory = createLogDirectory(cfg);
  const std::size_t max_log_samples = static_cast<std::size_t>(
      std::ceil(cfg.max_log_duration_s * cfg.log_rate_hz)) + 1U;
  std::vector<LogSample> log_buffer(max_log_samples);
  std::size_t log_count = 0;
  double log_elapsed_s = 0.0;
  const double log_period_s = 1.0 / cfg.log_rate_hz;
  bool log_overflow = false;
  std::cout << "Logging run data to: " << log_directory << "\n";

  double elapsed_s = 0.0;
  double final_hold_elapsed_s = 0.0;
  int half_cycle_index = 0;
  double phase = 0.0;
  double speed_scale = 0.0;

  RehabMode mode = RehabMode::kCalibration;

  Eigen::Vector3d force_bias_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_bias = Eigen::Vector3d::Zero();
  std::size_t force_bias_samples = 0;
  double filtered_opposing_force_n = 0.0;

  double pause_exceedance_s = 0.0;
  double resume_below_threshold_s = 0.0;
  double hold_elapsed_s = 0.0;
  double perpendicular_error_exceedance_s = 0.0;
  double tangential_error_exceedance_s = 0.0;

  robot.control([&](const franka::RobotState& state,
                    franka::Duration period) -> franka::Torques {
    const double dt = period.toSec();
    elapsed_s += dt;

    const auto coriolis_array = model.coriolis(state);
    const auto jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
    Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
    Eigen::Map<const Matrix67d> jacobian(jacobian_array.data());
    Eigen::Map<const Vector7d> dq(state.dq.data());

    const Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    const Eigen::Vector3d position = transform.translation();
    Eigen::Quaterniond orientation(transform.rotation());
    const Vector6d actual_twist = jacobian * dq;
    const Eigen::Vector3d linear_velocity = actual_twist.head<3>();

    // O_F_ext_hat_K is the wrench acting on stiffness frame K, expressed in
    // the robot base frame O, so it can be projected directly onto the arc tangent.
    const Eigen::Vector3d external_force_base(state.O_F_ext_hat_K[0],
                                               state.O_F_ext_hat_K[1],
                                               state.O_F_ext_hat_K[2]);
    const double absolute_force_n = external_force_base.norm();

    const bool calibrating = elapsed_s <= cfg.settle_s;
    if (calibrating) {
      force_bias_sum += external_force_base;
      ++force_bias_samples;
      force_bias = force_bias_sum / static_cast<double>(force_bias_samples);
      mode = RehabMode::kCalibration;
      phase = 0.0;
      speed_scale = 0.0;
    } else if (mode == RehabMode::kCalibration) {
      mode = RehabMode::kMove;
      speed_scale = 0.0;
      filtered_opposing_force_n = 0.0;
    }

    const Eigen::Vector3d delta_force_base = external_force_base - force_bias;
    const double delta_force_n = delta_force_base.norm();

    const double phase_dot_for_projection =
        (mode == RehabMode::kMove || mode == RehabMode::kSlowAssist)
            ? speed_scale / cfg.half_cycle_s
            : 0.0;
    const int sample_half_cycle =
        std::min(half_cycle_index, std::max(0, total_half_cycles - 1));
    ArcSample sample = sampleShoulderArc(
        cfg, initial_position, sample_half_cycle, phase, phase_dot_for_projection);

    const double projected_force_n =
        cfg.force_projection_sign * sample.tangent.dot(delta_force_base);
    const double opposing_force_n = std::max(0.0, projected_force_n);

    if (!calibrating) {
      const double filter_gain = dt / (cfg.force_filter_tau_s + dt);
      filtered_opposing_force_n +=
          filter_gain * (opposing_force_n - filtered_opposing_force_n);
    }

    bool resistance_hold_timeout = false;

    if (!calibrating && mode != RehabMode::kFinalHold) {
      if (mode == RehabMode::kCompliantHold) {
        hold_elapsed_s += dt;
        if (filtered_opposing_force_n <= cfg.force_resume_n) {
          resume_below_threshold_s += dt;
        } else {
          resume_below_threshold_s = 0.0;
        }

        if (resume_below_threshold_s >= cfg.resume_persistence_s) {
          mode = filtered_opposing_force_n <= cfg.force_free_n
                     ? RehabMode::kMove
                     : RehabMode::kSlowAssist;
          hold_elapsed_s = 0.0;
          resume_below_threshold_s = 0.0;
          pause_exceedance_s = 0.0;
        } else if (hold_elapsed_s >= cfg.resistance_hold_timeout_s) {
          resistance_hold_timeout = true;
        }
      } else {
        if (filtered_opposing_force_n >= cfg.force_pause_n) {
          pause_exceedance_s += dt;
        } else {
          pause_exceedance_s = 0.0;
        }

        if (pause_exceedance_s >= cfg.pause_persistence_s) {
          mode = RehabMode::kCompliantHold;
          speed_scale = 0.0;
          hold_elapsed_s = 0.0;
          resume_below_threshold_s = 0.0;
          ++summary.hold_entries;
        } else if (filtered_opposing_force_n > cfg.force_free_n) {
          mode = RehabMode::kSlowAssist;
        } else {
          mode = RehabMode::kMove;
        }
      }
    }

    double speed_scale_target = 0.0;
    if (mode == RehabMode::kMove) {
      speed_scale_target = 1.0;
    } else if (mode == RehabMode::kSlowAssist) {
      speed_scale_target = std::clamp(
          (cfg.force_pause_n - filtered_opposing_force_n) /
              (cfg.force_pause_n - cfg.force_free_n),
          0.0,
          1.0);
    }

    const double max_speed_scale_change = cfg.speed_scale_rate_per_s * dt;
    speed_scale += std::clamp(speed_scale_target - speed_scale,
                              -max_speed_scale_change,
                              max_speed_scale_change);
    speed_scale = std::clamp(speed_scale, 0.0, 1.0);

    if (!calibrating &&
        (mode == RehabMode::kMove || mode == RehabMode::kSlowAssist) &&
        half_cycle_index < total_half_cycles) {
      phase += speed_scale * dt / cfg.half_cycle_s;
      if (phase >= 1.0) {
        phase = 0.0;
        ++half_cycle_index;
        summary.completed_half_cycles = half_cycle_index;
        if (half_cycle_index >= total_half_cycles) {
          mode = RehabMode::kFinalHold;
          final_hold_elapsed_s = 0.0;
          speed_scale = 0.0;
        }
      }
    }

    if (mode == RehabMode::kFinalHold) {
      final_hold_elapsed_s += dt;
    }

    const int command_half_cycle =
        std::min(half_cycle_index, std::max(0, total_half_cycles - 1));
    const double command_phase =
        half_cycle_index >= total_half_cycles ? 1.0 : phase;
    const double command_phase_dot =
        (mode == RehabMode::kMove || mode == RehabMode::kSlowAssist)
            ? speed_scale / cfg.half_cycle_s
            : 0.0;
    sample = sampleShoulderArc(
        cfg, initial_position, command_half_cycle, command_phase, command_phase_dot);

    const Eigen::Vector3d position_error = sample.position - position;
    const double tangential_error_m = sample.tangent.dot(position_error);
    const double radial_error_m = sample.radial.dot(position_error);
    const double axis_error_m = sample.axis.dot(position_error);
    const Eigen::Vector3d perpendicular_error_vector =
        radial_error_m * sample.radial + axis_error_m * sample.axis;
    const double perpendicular_error_m = perpendicular_error_vector.norm();

    const Eigen::Vector3d velocity_error = sample.velocity - linear_velocity;
    const double tangential_velocity_error = sample.tangent.dot(velocity_error);
    const double radial_velocity_error = sample.radial.dot(velocity_error);
    const double axis_velocity_error = sample.axis.dot(velocity_error);

    const double active_tangent_stiffness =
        mode == RehabMode::kCompliantHold
            ? cfg.hold_tangent_stiffness_npm
            : cfg.stiffness_tangent_npm;
    const double tangent_damping = 2.0 * std::sqrt(std::max(0.0, active_tangent_stiffness));
    const double radial_damping = 2.0 * std::sqrt(cfg.stiffness_radial_npm);
    const double axis_damping = 2.0 * std::sqrt(cfg.stiffness_axis_npm);

    Eigen::Vector3d commanded_force =
        (active_tangent_stiffness * tangential_error_m +
         tangent_damping * tangential_velocity_error) * sample.tangent +
        (cfg.stiffness_radial_npm * radial_error_m +
         radial_damping * radial_velocity_error) * sample.radial +
        (cfg.stiffness_axis_npm * axis_error_m +
         axis_damping * axis_velocity_error) * sample.axis;

    double assist_force_n = 0.0;
    if (mode == RehabMode::kMove || mode == RehabMode::kSlowAssist) {
      const double positive_lag_m =
          std::max(0.0, tangential_error_m - cfg.assist_deadband_m);
      assist_force_n = std::clamp(
          cfg.assist_stiffness_npm * positive_lag_m,
          0.0,
          cfg.assist_force_max_n);
      commanded_force += assist_force_n * sample.tangent;
    }

    commanded_force.z() += cfg.vertical_support_force_n;

    Vector6d desired_wrench = Vector6d::Zero();
    desired_wrench.head<3>() = commanded_force;

    if (desired_orientation.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() = -orientation.coeffs();
    }
    const Eigen::Quaterniond orientation_error_quaternion(
        orientation.inverse() * desired_orientation);
    Eigen::Vector3d orientation_error(
        orientation_error_quaternion.x(),
        orientation_error_quaternion.y(),
        orientation_error_quaternion.z());
    orientation_error = -transform.rotation() * orientation_error;
    const double rotational_damping =
        2.0 * std::sqrt(cfg.rotational_stiffness_nmprad);
    desired_wrench.tail<3>() =
        -cfg.rotational_stiffness_nmprad * orientation_error -
        rotational_damping * actual_twist.tail<3>();

    const Vector7d tau_command =
        jacobian.transpose() * desired_wrench + coriolis;
    std::array<double, 7> tau_array{};
    for (std::size_t i = 0; i < tau_array.size(); ++i) {
      tau_array[i] = tau_command(static_cast<Eigen::Index>(i));
    }
    const auto tau_limited =
        franka::limitRate(franka::kMaxTorqueRate, tau_array, state.tau_J_d);

    Eigen::Map<const Vector7d> tau_external(state.tau_ext_hat_filtered.data());
    const double external_joint_torque_norm_nm = tau_external.norm();
    const double ee_speed_mps = linear_velocity.norm();

    summary.max_absolute_force_n =
        std::max(summary.max_absolute_force_n, absolute_force_n);
    summary.max_delta_force_n =
        std::max(summary.max_delta_force_n, delta_force_n);
    summary.max_opposing_force_n =
        std::max(summary.max_opposing_force_n, filtered_opposing_force_n);
    summary.max_assist_force_n =
        std::max(summary.max_assist_force_n, assist_force_n);
    summary.max_perpendicular_error_m =
        std::max(summary.max_perpendicular_error_m, perpendicular_error_m);
    summary.max_tangential_error_m =
        std::max(summary.max_tangential_error_m, std::abs(tangential_error_m));
    summary.max_ee_speed_mps =
        std::max(summary.max_ee_speed_mps, ee_speed_mps);
    if (!calibrating && mode != RehabMode::kFinalHold) {
      summary.minimum_speed_scale =
          std::min(summary.minimum_speed_scale, speed_scale);
    }

    log_elapsed_s += dt;
    if (log_elapsed_s >= log_period_s) {
      log_elapsed_s = std::fmod(log_elapsed_s, log_period_s);
      if (log_count < log_buffer.size()) {
        LogSample& row = log_buffer[log_count++];
        row.time_s = elapsed_s;
        row.dt_s = dt;
        row.mode = static_cast<int>(mode);
        row.half_cycle_index = half_cycle_index;
        row.completed_half_cycles = summary.completed_half_cycles;
        row.phase = phase;
        row.speed_scale = speed_scale;
        for (int i = 0; i < 3; ++i) {
          row.position[static_cast<std::size_t>(i)] = position(i);
          row.desired_position[static_cast<std::size_t>(i)] = sample.position(i);
          row.velocity[static_cast<std::size_t>(i)] = linear_velocity(i);
          row.desired_velocity[static_cast<std::size_t>(i)] = sample.velocity(i);
          row.position_error[static_cast<std::size_t>(i)] = position_error(i);
          row.external_force[static_cast<std::size_t>(i)] = external_force_base(i);
          row.delta_force[static_cast<std::size_t>(i)] = delta_force_base(i);
          row.commanded_force[static_cast<std::size_t>(i)] = commanded_force(i);
        }
        row.tracking_error_norm_m = position_error.norm();
        row.tangential_error_m = tangential_error_m;
        row.radial_error_m = radial_error_m;
        row.axis_error_m = axis_error_m;
        row.perpendicular_error_m = perpendicular_error_m;
        row.absolute_force_n = absolute_force_n;
        row.delta_force_norm_n = delta_force_n;
        row.projected_force_n = projected_force_n;
        row.opposing_force_n = opposing_force_n;
        row.filtered_opposing_force_n = filtered_opposing_force_n;
        row.assist_force_n = assist_force_n;
        row.ee_speed_mps = ee_speed_mps;
        row.external_joint_torque_norm_nm = external_joint_torque_norm_nm;
        for (int j = 0; j < 7; ++j) {
          const std::size_t index = static_cast<std::size_t>(j);
          row.q[index] = state.q[index];
          row.dq[index] = state.dq[index];
          row.tau_measured[index] = state.tau_J[index];
          row.tau_commanded[index] = tau_limited[index];
          row.tau_external[index] = state.tau_ext_hat_filtered[index];
        }
      } else {
        log_overflow = true;
      }
    }

    if (!calibrating && perpendicular_error_m > cfg.perpendicular_error_stop_m) {
      perpendicular_error_exceedance_s += dt;
    } else {
      perpendicular_error_exceedance_s = 0.0;
    }

    if (!calibrating &&
        std::abs(tangential_error_m) > cfg.tangential_error_hard_stop_m) {
      tangential_error_exceedance_s += dt;
    } else {
      tangential_error_exceedance_s = 0.0;
    }

    bool stop = false;
    if (g_stop_requested != 0) {
      summary.stop_reason = 1;
      stop = true;
    } else if (absolute_force_n > cfg.force_absolute_hard_stop_n) {
      summary.stop_reason = 2;
      stop = true;
    } else if (!calibrating && delta_force_n > cfg.force_delta_hard_stop_n) {
      summary.stop_reason = 3;
      stop = true;
    } else if (external_joint_torque_norm_nm >
               cfg.external_joint_torque_norm_stop_nm) {
      summary.stop_reason = 4;
      stop = true;
    } else if (perpendicular_error_exceedance_s >
               cfg.perpendicular_error_persistence_s) {
      summary.stop_reason = 5;
      stop = true;
    } else if (tangential_error_exceedance_s >
               cfg.tangential_error_persistence_s) {
      summary.stop_reason = 6;
      stop = true;
    } else if (ee_speed_mps > cfg.ee_speed_stop_mps) {
      summary.stop_reason = 7;
      stop = true;
    } else if (resistance_hold_timeout) {
      summary.stop_reason = 8;
      stop = true;
    } else if (mode == RehabMode::kFinalHold &&
               final_hold_elapsed_s >= cfg.final_hold_s) {
      summary.stop_reason = 0;
      stop = true;
    }

    summary.final_mode = mode;

    if (stop) {
      std::array<double, 7> zero_torque{};
      const auto stop_torque =
          franka::limitRate(franka::kMaxTorqueRate, zero_torque, state.tau_J_d);
      return franka::MotionFinished(franka::Torques(stop_torque));
    }

    return franka::Torques(tau_limited);
  });

  writeRunDataCsv(log_directory / "run_data.csv", log_buffer, log_count);
  writeConfigCsv(log_directory / "config.csv", cfg);
  writeSummaryCsv(log_directory / "controller_summary.csv", summary, log_count, log_overflow);

  std::cout << std::fixed << std::setprecision(3)
            << "\nExercise finished.\n"
            << "Final mode: " << modeName(summary.final_mode) << "\n"
            << "Completed half-cycles: " << summary.completed_half_cycles
            << " / " << total_half_cycles << "\n"
            << "Resistance-hold entries: " << summary.hold_entries << "\n"
            << "Max absolute external force: " << summary.max_absolute_force_n << " N\n"
            << "Max force change from bias: " << summary.max_delta_force_n << " N\n"
            << "Max filtered opposing force: " << summary.max_opposing_force_n << " N\n"
            << "Max bounded assist force: " << summary.max_assist_force_n << " N\n"
            << "Max perpendicular error: " << summary.max_perpendicular_error_m << " m\n"
            << "Max absolute tangential error: " << summary.max_tangential_error_m << " m\n"
            << "Max end-effector speed: " << summary.max_ee_speed_mps << " m/s\n"
            << "Minimum speed scale: " << summary.minimum_speed_scale << "\n"
            << "Logged samples: " << log_count << "\n"
            << "CSV log: " << (log_directory / "run_data.csv") << "\n"
            << "Controller summary: " << (log_directory / "controller_summary.csv") << "\n";
  if (log_overflow) {
    std::cout << "WARNING: log buffer filled; later samples were not recorded.\n";
  }

  switch (summary.stop_reason) {
    case 0:
      std::cout << "Stop reason: normal completion.\n";
      break;
    case 1:
      std::cout << "Stop reason: Ctrl+C.\n";
      break;
    case 2:
      std::cout << "Stop reason: absolute-force hard threshold.\n";
      break;
    case 3:
      std::cout << "Stop reason: force-change hard threshold.\n";
      break;
    case 4:
      std::cout << "Stop reason: external-joint-torque threshold.\n";
      break;
    case 5:
      std::cout << "Stop reason: perpendicular path-error threshold.\n";
      break;
    case 6:
      std::cout << "Stop reason: tangential hard-error threshold.\n";
      break;
    case 7:
      std::cout << "Stop reason: end-effector-speed threshold.\n";
      break;
    case 8:
      std::cout << "Stop reason: resistance remained high during compliant hold.\n";
      break;
    default:
      std::cout << "Stop reason: unspecified.\n";
      break;
  }

  return summary;
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  try {
    cfg = parseArguments(argc, argv, cfg);
  } catch (const std::exception& e) {
    std::cerr << "Argument error: " << e.what() << "\n\n";
    printUsage(argv[0], cfg);
    return EXIT_FAILURE;
  }

  if (!cfg.risk_acknowledged) {
    std::cerr
        << "Refusing to run without the explicit risk acknowledgement flag.\n";
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, signalHandler);
  g_stop_requested = 0;

  std::cout
      << "\n=== FR3 resistance-aware shoulder-arc exercise ===\n"
      << "The current pose is the lower point. No automatic approach is used.\n"
      << "The HANDLE, not the human hand, must already be between the fingers.\n\n"
      << "Robot IP: " << cfg.robot_ip << "\n"
      << "Handle width: " << cfg.handle_width_m << " m\n"
      << "Gripper force: " << cfg.gripper_force_n << " N\n"
      << "Shoulder-centre offset [m]: "
      << cfg.shoulder_center_offset.transpose() << "\n"
      << "Rotation axis: " << cfg.rotation_axis.transpose() << "\n"
      << "Amplitude: " << cfg.amplitude_deg << " deg\n"
      << "Nominal half-cycle: " << cfg.half_cycle_s << " s\n"
      << "Repetitions: " << cfg.repetitions << "\n"
      << "Resistance thresholds [free/resume/pause]: "
      << cfg.force_free_n << " / " << cfg.force_resume_n << " / "
      << cfg.force_pause_n << " N\n"
      << "Bounded assist maximum: " << cfg.assist_force_max_n << " N\n"
      << "Log root: " << cfg.log_root << "\n"
      << "Log rate: " << cfg.log_rate_hz << " Hz\n\n";

  try {
    franka::Robot robot(cfg.robot_ip);
    franka::Gripper gripper(cfg.robot_ip);

    const franka::GripperState before_grasp = gripper.readOnce();
    std::cout << std::fixed << std::setprecision(4)
              << "Current gripper width: " << before_grasp.width << " m\n"
              << "Current gripper max_width: " << before_grasp.max_width << " m\n";

    if (before_grasp.max_width > 1e-6 &&
        cfg.handle_width_m > before_grasp.max_width) {
      throw std::runtime_error(
          "Configured handle width exceeds the calibrated gripper max_width.");
    }
    if (before_grasp.width + cfg.grasp_epsilon_inner_m < cfg.handle_width_m) {
      throw std::runtime_error(
          "The current gripper gap is smaller than the configured handle width.");
    }

    if (!promptExact(
            "Confirm that the HANDLE is centred between the fingers, the participant "
            "is not holding it yet, and the physical user stop is held. Type GRASP: ",
            "GRASP")) {
      std::cout << "Cancelled.\n";
      return EXIT_SUCCESS;
    }

    std::cout << "Closing gripper around the 2.5 cm handle...\n";
    const bool grasped = gripper.grasp(cfg.handle_width_m,
                                       cfg.gripper_speed_mps,
                                       cfg.gripper_force_n,
                                       cfg.grasp_epsilon_inner_m,
                                       cfg.grasp_epsilon_outer_m);
    const franka::GripperState after_grasp = gripper.readOnce();
    if (!grasped || !after_grasp.is_grasped) {
      throw std::runtime_error(
          "Handle grasp was not verified. Check diameter, alignment and epsilon values.");
    }
    std::cout << "Handle grasp verified at width " << after_grasp.width << " m.\n";

    if (!promptExact(
            "The participant may now hold the handle with the arm extended but not "
            "hyperextended. Keep the participant relaxed during the initial force-bias "
            "period. Type READY: ",
            "READY")) {
      std::cout << "Cancelled after grasp. The gripper remains closed.\n";
      return EXIT_SUCCESS;
    }

    std::cout
        << "Starting resistance-aware shoulder arc. Mild resistance slows the path; "
        << "higher sustained resistance causes a compliant hold. Ctrl+C or the "
        << "physical user stop can interrupt it.\n";

    const RunSummary summary = runShoulderExercise(robot, cfg);

    if (!promptExact(
            "The participant must release the handle and the handle must be supported. "
            "Type OPEN to release it: ",
            "OPEN")) {
      std::cout << "The gripper remains closed.\n";
      return summary.stop_reason == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    gripper.stop();
    const franka::GripperState release_state = gripper.readOnce();
    if (release_state.max_width <= cfg.handle_width_m + 0.002) {
      std::cerr
          << "Automatic opening skipped because calibrated max_width is too small.\n";
      return EXIT_FAILURE;
    }

    const double release_width =
        std::min(0.070, release_state.max_width - 0.001);
    std::cout << "Opening gripper to " << release_width << " m...\n";
    if (!gripper.move(release_width, cfg.gripper_speed_mps)) {
      throw std::runtime_error("Failed to open gripper after the exercise.");
    }

    std::cout << "Program complete.\n";
    return summary.stop_reason == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka error: " << e.what() << "\n";
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}

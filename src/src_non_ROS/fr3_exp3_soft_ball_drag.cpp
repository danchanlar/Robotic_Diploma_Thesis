#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
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
#include <franka/robot.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMaxLogRows = 300000;

struct Options {
  std::string robot_ip = "172.16.0.2";
  std::string output_csv = "/home/danai/fr3_force_logs/exp3_soft_ball_drag.csv";

  // Gripper setup. The gripper closes before the robot motion begins.
  bool skip_gripper = false;
  bool home_gripper = false;
  double gripper_width = 0.025;  // m, small opening / almost closed
  double gripper_speed = 0.03;   // m/s

  // Vertical contact approach.
  int direction = -1;                 // -1: downward in base z
  int force_sign = -1;                // choose sign so contact_force is positive in contact
  double bias_time = 1.5;             // s
  double approach_distance = 0.100;   // m, max downward travel
  double approach_speed = 0.0015;     // m/s
  double min_contact_descent = 0.020; // m, ignore force before this descent
  double contact_threshold = 1.0;     // N

  // Touchdown/press after contact. This is what creates normal force on the foam ball.
  double touchdown_depth = 0.006;     // m, extra downward compression after contact
  double touchdown_time = 1.5;        // s

  // Straight-line dragging on the table.
  std::string drag_axis = "x";        // x or y
  int drag_direction = 1;             // +1 or -1 along drag_axis
  double drag_distance = 0.080;       // m
  double drag_speed = 0.004;          // m/s

  // Retract.
  double retract_distance = 0.050;    // m upward
  double retract_time = 5.0;          // s

  // Cartesian impedance gains.
  double xy_stiffness = 350.0;        // N/m
  double xy_damping = 30.0;           // Ns/m
  double z_stiffness = 500.0;         // N/m, approach/touchdown/retract
  double z_damping = 40.0;            // Ns/m
  double z_drag_stiffness = 550.0;    // N/m, hold pressing height during dragging
  double z_drag_damping = 45.0;       // Ns/m
  double rot_stiffness = 20.0;        // Nm/rad
  double rot_damping = 3.5;           // Nms/rad

  // Safety limits.
  double max_force = 15.0;            // N, bias-compensated external force norm
  double max_ext_torque_norm = 8.0;   // Nm
  double max_ee_speed = 0.10;         // m/s
  double max_delta_tau = 0.8;         // Nm per control step

  bool risk_ack = false;
};

void printUsage(const char* argv0) {
  std::cout << "Usage:\n"
            << "  " << argv0 << " --robot-ip 172.16.0.2 --output ~/fr3_force_logs/exp3_soft_ball_drag.csv \\\n"
            << "    --gripper-width 0.025 --direction -1 --force-sign -1 \\\n"
            << "    --contact-threshold 1.0 --touchdown-depth 0.006 --drag-axis x --drag-distance 0.08 \\\n"
            << "    --i-understand-real-robot-risk\n";
}

double parseDouble(const std::string& v, const std::string& name) {
  try {
    size_t idx = 0;
    const double out = std::stod(v, &idx);
    if (idx != v.size()) throw std::invalid_argument("trailing characters");
    return out;
  } catch (...) {
    throw std::runtime_error("Invalid numeric value for " + name + ": " + v);
  }
}

int parseInt(const std::string& v, const std::string& name) {
  try {
    size_t idx = 0;
    const int out = std::stoi(v, &idx);
    if (idx != v.size()) throw std::invalid_argument("trailing characters");
    return out;
  } catch (...) {
    throw std::runtime_error("Invalid integer value for " + name + ": " + v);
  }
}

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("Missing value after " + name);
      return argv[++i];
    };

    if (a == "--help" || a == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (a == "--robot-ip") {
      opt.robot_ip = need(a);
    } else if (a == "--output") {
      opt.output_csv = need(a);
    } else if (a == "--skip-gripper") {
      opt.skip_gripper = true;
    } else if (a == "--home-gripper") {
      opt.home_gripper = true;
    } else if (a == "--gripper-width") {
      opt.gripper_width = parseDouble(need(a), a);
    } else if (a == "--gripper-speed") {
      opt.gripper_speed = parseDouble(need(a), a);
    } else if (a == "--direction") {
      opt.direction = parseInt(need(a), a);
    } else if (a == "--force-sign") {
      opt.force_sign = parseInt(need(a), a);
    } else if (a == "--bias-time") {
      opt.bias_time = parseDouble(need(a), a);
    } else if (a == "--approach-distance") {
      opt.approach_distance = parseDouble(need(a), a);
    } else if (a == "--approach-speed") {
      opt.approach_speed = parseDouble(need(a), a);
    } else if (a == "--min-contact-descent") {
      opt.min_contact_descent = parseDouble(need(a), a);
    } else if (a == "--contact-threshold") {
      opt.contact_threshold = parseDouble(need(a), a);
    } else if (a == "--touchdown-depth") {
      opt.touchdown_depth = parseDouble(need(a), a);
    } else if (a == "--touchdown-time") {
      opt.touchdown_time = parseDouble(need(a), a);
    } else if (a == "--drag-axis") {
      opt.drag_axis = need(a);
    } else if (a == "--drag-direction") {
      opt.drag_direction = parseInt(need(a), a);
    } else if (a == "--drag-distance") {
      opt.drag_distance = parseDouble(need(a), a);
    } else if (a == "--drag-speed") {
      opt.drag_speed = parseDouble(need(a), a);
    } else if (a == "--retract-distance") {
      opt.retract_distance = parseDouble(need(a), a);
    } else if (a == "--retract-time") {
      opt.retract_time = parseDouble(need(a), a);
    } else if (a == "--xy-stiffness") {
      opt.xy_stiffness = parseDouble(need(a), a);
    } else if (a == "--xy-damping") {
      opt.xy_damping = parseDouble(need(a), a);
    } else if (a == "--z-stiffness") {
      opt.z_stiffness = parseDouble(need(a), a);
    } else if (a == "--z-damping") {
      opt.z_damping = parseDouble(need(a), a);
    } else if (a == "--z-drag-stiffness") {
      opt.z_drag_stiffness = parseDouble(need(a), a);
    } else if (a == "--z-drag-damping") {
      opt.z_drag_damping = parseDouble(need(a), a);
    } else if (a == "--rot-stiffness") {
      opt.rot_stiffness = parseDouble(need(a), a);
    } else if (a == "--rot-damping") {
      opt.rot_damping = parseDouble(need(a), a);
    } else if (a == "--max-force") {
      opt.max_force = parseDouble(need(a), a);
    } else if (a == "--max-ext-torque-norm") {
      opt.max_ext_torque_norm = parseDouble(need(a), a);
    } else if (a == "--max-ee-speed") {
      opt.max_ee_speed = parseDouble(need(a), a);
    } else if (a == "--max-delta-tau") {
      opt.max_delta_tau = parseDouble(need(a), a);
    } else if (a == "--i-understand-real-robot-risk") {
      opt.risk_ack = true;
    } else {
      throw std::runtime_error("Unknown argument: " + a);
    }
  }

  if (!opt.risk_ack) throw std::runtime_error("Refusing to run without --i-understand-real-robot-risk");
  if (opt.direction != -1 && opt.direction != 1) throw std::runtime_error("--direction must be -1 or 1");
  if (opt.force_sign != -1 && opt.force_sign != 1) throw std::runtime_error("--force-sign must be -1 or 1");
  if (opt.drag_direction != -1 && opt.drag_direction != 1) throw std::runtime_error("--drag-direction must be -1 or 1");
  if (opt.drag_axis != "x" && opt.drag_axis != "y") throw std::runtime_error("--drag-axis must be x or y");
  if (opt.gripper_width < 0.0 || opt.gripper_width > 0.08) throw std::runtime_error("--gripper-width must be in [0, 0.08] m");
  if (opt.approach_distance <= 0.0 || opt.approach_distance > 0.16) throw std::runtime_error("--approach-distance must be in (0, 0.16] m");
  if (opt.min_contact_descent < 0.0 || opt.min_contact_descent > opt.approach_distance) throw std::runtime_error("--min-contact-descent must be in [0, approach-distance]");
  if (opt.touchdown_depth < 0.0 || opt.touchdown_depth > 0.050) throw std::runtime_error("--touchdown-depth must be in [0, 0.030] m");
  if (opt.drag_distance <= 0.0 || opt.drag_distance > 0.20) throw std::runtime_error("--drag-distance must be in (0, 0.20] m");
  if (opt.drag_speed <= 0.0 || opt.drag_speed > 0.04) throw std::runtime_error("--drag-speed must be in (0, 0.04] m/s");

  return opt;
}

Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

std::array<double, 7> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                         const std::array<double, 7>& tau_J_d,
                                         double max_delta_tau) {
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; ++i) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, max_delta_tau), -max_delta_tau);
  }
  return tau_d_saturated;
}

std::string phaseName(int phase) {
  switch (phase) {
    case 0: return "bias_hold";
    case 1: return "approach_to_ball";
    case 2: return "touchdown_press";
    case 3: return "drag_ball";
    case 4: return "retract";
    case 5: return "finished";
    case 9: return "safety_stop";
    default: return "unknown";
  }
}

struct LogRow {
  double time = 0.0;
  int phase_id = 0;
  std::string phase_name;

  double x = 0.0, y = 0.0, z = 0.0;
  double x_des = 0.0, y_des = 0.0, z_des = 0.0;
  double ex = 0.0, ey = 0.0, ez = 0.0;

  double fx_raw = 0.0, fy_raw = 0.0, fz_raw = 0.0;
  double fx_bias = 0.0, fy_bias = 0.0, fz_bias = 0.0;
  double fx = 0.0, fy = 0.0, fz = 0.0;
  double force_norm = 0.0;
  double contact_force_z = 0.0;

  double ee_speed = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double tau_ext_norm = 0.0;

  double wx_cmd = 0.0, wy_cmd = 0.0, wz_cmd = 0.0;
  double mx_cmd = 0.0, my_cmd = 0.0, mz_cmd = 0.0;

  std::array<double, 7> tau_cmd{};
  std::array<double, 7> tau_J{};
  std::array<double, 7> tau_ext{};
};

class DataLogger {
 public:
  DataLogger() { rows_.reserve(kMaxLogRows); }

  void log(const LogRow& r) {
    if (rows_.size() < kMaxLogRows) rows_.push_back(r);
  }

  void writeCsv(const std::string& path) const {
    std::ofstream out(path);
    if (!out.is_open()) throw std::runtime_error("Could not open CSV file: " + path);

    out << "time,phase_id,phase_name,"
        << "x,y,z,x_des,y_des,z_des,ex,ey,ez,"
        << "fx_raw,fy_raw,fz_raw,fx_bias,fy_bias,fz_bias,fx,fy,fz,force_norm,contact_force_z,"
        << "ee_speed,vx,vy,vz,tau_ext_norm,"
        << "wx_cmd,wy_cmd,wz_cmd,mx_cmd,my_cmd,mz_cmd,"
        << "tau_cmd1,tau_cmd2,tau_cmd3,tau_cmd4,tau_cmd5,tau_cmd6,tau_cmd7,"
        << "tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7,"
        << "tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7\n";

    out << std::fixed << std::setprecision(9);
    for (const auto& r : rows_) {
      out << r.time << "," << r.phase_id << "," << r.phase_name << ","
          << r.x << "," << r.y << "," << r.z << ","
          << r.x_des << "," << r.y_des << "," << r.z_des << ","
          << r.ex << "," << r.ey << "," << r.ez << ","
          << r.fx_raw << "," << r.fy_raw << "," << r.fz_raw << ","
          << r.fx_bias << "," << r.fy_bias << "," << r.fz_bias << ","
          << r.fx << "," << r.fy << "," << r.fz << ","
          << r.force_norm << "," << r.contact_force_z << ","
          << r.ee_speed << "," << r.vx << "," << r.vy << "," << r.vz << ","
          << r.tau_ext_norm << ","
          << r.wx_cmd << "," << r.wy_cmd << "," << r.wz_cmd << ","
          << r.mx_cmd << "," << r.my_cmd << "," << r.mz_cmd;
      for (double v : r.tau_cmd) out << "," << v;
      for (double v : r.tau_J) out << "," << v;
      for (double v : r.tau_ext) out << "," << v;
      out << "\n";
    }

    std::cout << "CSV written: " << path << " (" << rows_.size() << " rows)\n";
  }

 private:
  std::vector<LogRow> rows_;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    Options opt = parseArgs(argc, argv);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Connecting to FR3 at " << opt.robot_ip << "...\n";

    if (!opt.skip_gripper) {
      std::cout << "Connecting to gripper...\n";
      franka::Gripper gripper(opt.robot_ip);
      if (opt.home_gripper) {
        std::cout << "Homing gripper...\n";
        gripper.homing();
      }
      std::cout << "Closing gripper to width " << opt.gripper_width << " m...\n";
      bool ok = gripper.move(opt.gripper_width, opt.gripper_speed);
      if (!ok) {
        std::cerr << "WARNING: gripper.move returned false. Continuing with robot motion.\n";
      }
    }

    franka::Robot robot(opt.robot_ip);
    franka::Model model = robot.loadModel();

    try {
      std::cout << "Running automatic error recovery...\n";
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << "\n";
    }

    franka::RobotState initial_state = robot.readOnce();
    Eigen::Matrix4d T0 = arrayToEigen(initial_state.O_T_EE);
    Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
    Eigen::Quaterniond q_des(T0.block<3, 3>(0, 0));
    q_des.normalize();

    const Eigen::Vector3d approach_axis(0.0, 0.0, static_cast<double>(opt.direction));
    Eigen::Vector3d drag_axis = Eigen::Vector3d::Zero();
    if (opt.drag_axis == "x") drag_axis.x() = static_cast<double>(opt.drag_direction);
    if (opt.drag_axis == "y") drag_axis.y() = static_cast<double>(opt.drag_direction);
    const double drag_time = opt.drag_distance / opt.drag_speed;

    std::cout << "Initial EE position: [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "] m\n";
    std::cout << "Approach direction: z * " << opt.direction << "\n";
    std::cout << "Contact threshold: " << opt.contact_threshold << " N, force-sign=" << opt.force_sign << "\n";
    std::cout << "Touchdown depth: " << opt.touchdown_depth << " m\n";
    std::cout << "Drag: axis=" << opt.drag_axis << ", direction=" << opt.drag_direction
              << ", distance=" << opt.drag_distance << " m, speed=" << opt.drag_speed << " m/s\n";
    std::cout << "Estimated drag time: " << drag_time << " s\n";
    std::cout << "Output CSV: " << opt.output_csv << "\n\n";

    DataLogger logger;

    int phase = 0;
    bool printed_contact = false;
    bool safety_stop = false;
    std::string stop_reason = "finished";

    double time = 0.0;
    double phase_time = 0.0;
    double last_console_time = -1.0;

    Eigen::Vector3d force_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_bias_sum = Eigen::Vector3d::Zero();
    size_t force_bias_count = 0;

    Eigen::Vector3d p_contact = p0;
    Eigen::Vector3d p_touchdown = p0;
    Eigen::Vector3d p_drag_start = p0;
    Eigen::Vector3d p_retract_start = p0;
    Eigen::Vector3d p_retract_target = p0;

    Eigen::Vector3d p_prev = p0;
    Eigen::Quaterniond q_prev = q_des;
    bool has_prev = false;

    auto finishTorques = [&](const Eigen::Matrix<double, 7, 1>& tau_d,
                             const std::array<double, 7>& tau_J_d) -> franka::Torques {
      std::array<double, 7> tau_out = saturateTorqueRate(tau_d, tau_J_d, opt.max_delta_tau);
      return franka::MotionFinished(franka::Torques(tau_out));
    };

    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();
      time += dt;
      phase_time += dt;

      Eigen::Matrix4d T = arrayToEigen(state.O_T_EE);
      Eigen::Vector3d p = T.block<3, 1>(0, 3);
      Eigen::Quaterniond q(T.block<3, 3>(0, 0));
      q.normalize();

      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      Eigen::Vector3d omega = Eigen::Vector3d::Zero();
      if (has_prev && dt > 1e-9) {
        v = (p - p_prev) / dt;
        Eigen::Quaterniond dq_quat = q * q_prev.conjugate();
        dq_quat.normalize();
        if (dq_quat.w() < 0.0) dq_quat.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(dq_quat);
        omega = aa.axis() * aa.angle() / dt;
      }
      p_prev = p;
      q_prev = q;
      has_prev = true;

      const auto& Fraw = state.O_F_ext_hat_K;
      Eigen::Vector3d F_meas(Fraw[0], Fraw[1], Fraw[2]);
      Eigen::Vector3d F_comp = F_meas - force_bias;
      double force_norm = F_comp.norm();
      double contact_force_z = static_cast<double>(opt.force_sign) * F_comp.z();

      double tau_ext_norm = 0.0;
      for (size_t i = 0; i < 7; ++i) tau_ext_norm += state.tau_ext_hat_filtered[i] * state.tau_ext_hat_filtered[i];
      tau_ext_norm = std::sqrt(tau_ext_norm);

      const double ee_speed = v.norm();

      if (!safety_stop && phase != 0) {
        if (force_norm > opt.max_force) {
          safety_stop = true;
          stop_reason = "Bias-compensated external force norm exceeded max-force";
        } else if (tau_ext_norm > opt.max_ext_torque_norm) {
          safety_stop = true;
          stop_reason = "External joint torque norm exceeded max-ext-torque-norm";
        } else if (ee_speed > opt.max_ee_speed) {
          safety_stop = true;
          stop_reason = "End-effector speed exceeded max-ee-speed";
        }
      }

      if (safety_stop && phase != 9) {
        phase = 9;
        phase_time = 0.0;
        std::cerr << "SAFETY STOP: " << stop_reason << "\n";
      }

      Eigen::Vector3d p_des = p0;
      Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
      Eigen::Matrix<double, 6, 1> W_cmd;
      W_cmd.setZero();

      if (phase == 0) {
        p_des = p0;
        if (time > 0.2) {
          force_bias_sum += F_meas;
          force_bias_count++;
        }
        if (phase_time >= opt.bias_time) {
          if (force_bias_count > 0) force_bias = force_bias_sum / static_cast<double>(force_bias_count);
          phase = 1;
          phase_time = 0.0;
          std::cout << "Force bias estimated: [" << force_bias.x() << ", " << force_bias.y() << ", " << force_bias.z() << "] N\n";
          std::cout << "Phase 1: approach to soft ball.\n";
        }
      }

      if (phase == 1) {
        const double distance = std::min(opt.approach_speed * phase_time, opt.approach_distance);
        p_des = p0 + approach_axis * distance;
        v_des = approach_axis * opt.approach_speed;

        if (distance >= opt.min_contact_descent && contact_force_z >= opt.contact_threshold && phase_time > 0.3) {
          phase = 2;
          phase_time = 0.0;
          p_contact = p;
          printed_contact = true;
          std::cout << "Contact detected at t=" << time
                    << " s, contact_force_z=" << contact_force_z
                    << " N, position=[" << p.x() << ", " << p.y() << ", " << p.z() << "]\n";
          std::cout << "Phase 2: touchdown/press into soft ball.\n";
        } else if (distance >= opt.approach_distance) {
          phase = 4;
          phase_time = 0.0;
          p_retract_start = p;
          p_retract_target = p - approach_axis * opt.retract_distance;
          stop_reason = "Approach distance completed without contact";
          std::cerr << "WARNING: " << stop_reason << ". Retracting.\n";
        }
      }

      if (phase == 2) {
        const double s = std::min(phase_time / opt.touchdown_time, 1.0);
        const double alpha = 0.5 - 0.5 * std::cos(kPi * s);
        p_des = p_contact + alpha * approach_axis * opt.touchdown_depth;
        v_des = approach_axis * (0.5 * kPi / opt.touchdown_time) * std::sin(kPi * s) * opt.touchdown_depth;
        if (s >= 1.0) {
          phase = 3;
          phase_time = 0.0;
          p_touchdown = p;
          p_drag_start = p;
          std::cout << "Touchdown completed. Phase 3: drag soft ball.\n";
        }
      }

      if (phase == 3) {
        const double s_dist = std::min(opt.drag_speed * phase_time, opt.drag_distance);
        p_des = p_drag_start + drag_axis * s_dist;
        v_des = drag_axis * opt.drag_speed;
        // Keep the z height reached after pressing. This keeps vertical pressure on the soft ball.
        p_des.z() = p_drag_start.z();
        v_des.z() = 0.0;
        if (s_dist >= opt.drag_distance) {
          phase = 4;
          phase_time = 0.0;
          p_retract_start = p;
          p_retract_target = p - approach_axis * opt.retract_distance;
          std::cout << "Drag completed. Phase 4: retract.\n";
        }
      }

      if (phase == 4) {
        const double s = std::min(phase_time / opt.retract_time, 1.0);
        const double alpha = 0.5 - 0.5 * std::cos(kPi * s);
        p_des = p_retract_start + alpha * (p_retract_target - p_retract_start);
        if (s >= 1.0) {
          phase = 5;
          phase_time = 0.0;
          std::cout << "Retract completed. Finishing control.\n";
        }
      }

      if (phase == 5 || phase == 9) {
        p_des = p;
        v_des.setZero();
      }

      // Translational Cartesian wrench command.
      if (phase == 3) {
        // Dragging: position control in x-y and stiff z hold at touchdown height.
        W_cmd.x() = opt.xy_stiffness * (p_des.x() - p.x()) + opt.xy_damping * (v_des.x() - v.x());
        W_cmd.y() = opt.xy_stiffness * (p_des.y() - p.y()) + opt.xy_damping * (v_des.y() - v.y());
        W_cmd.z() = opt.z_drag_stiffness * (p_des.z() - p.z()) + opt.z_drag_damping * (v_des.z() - v.z());
      } else if (phase == 9) {
        W_cmd.setZero();
      } else {
        W_cmd.x() = opt.xy_stiffness * (p_des.x() - p.x()) + opt.xy_damping * (v_des.x() - v.x());
        W_cmd.y() = opt.xy_stiffness * (p_des.y() - p.y()) + opt.xy_damping * (v_des.y() - v.y());
        W_cmd.z() = opt.z_stiffness * (p_des.z() - p.z()) + opt.z_damping * (v_des.z() - v.z());
      }

      // Orientation stabilization around the initial orientation.
      Eigen::Quaterniond q_err = q_des * q.conjugate();
      q_err.normalize();
      if (q_err.w() < 0.0) q_err.coeffs() *= -1.0;
      Eigen::Vector3d rot_error = 2.0 * q_err.vec();
      W_cmd.tail<3>() = opt.rot_stiffness * rot_error - opt.rot_damping * omega;

      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      std::array<double, 7> coriolis_array = model.coriolis(state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

      Eigen::Matrix<double, 7, 1> tau_d = J.transpose() * W_cmd + coriolis;
      std::array<double, 7> tau_out = saturateTorqueRate(tau_d, state.tau_J_d, opt.max_delta_tau);

      LogRow r;
      r.time = time;
      r.phase_id = phase;
      r.phase_name = phaseName(phase);
      r.x = p.x(); r.y = p.y(); r.z = p.z();
      r.x_des = p_des.x(); r.y_des = p_des.y(); r.z_des = p_des.z();
      r.ex = p_des.x() - p.x(); r.ey = p_des.y() - p.y(); r.ez = p_des.z() - p.z();
      r.fx_raw = F_meas.x(); r.fy_raw = F_meas.y(); r.fz_raw = F_meas.z();
      r.fx_bias = force_bias.x(); r.fy_bias = force_bias.y(); r.fz_bias = force_bias.z();
      r.fx = F_comp.x(); r.fy = F_comp.y(); r.fz = F_comp.z();
      r.force_norm = force_norm;
      r.contact_force_z = contact_force_z;
      r.ee_speed = ee_speed;
      r.vx = v.x(); r.vy = v.y(); r.vz = v.z();
      r.tau_ext_norm = tau_ext_norm;
      r.wx_cmd = W_cmd.x(); r.wy_cmd = W_cmd.y(); r.wz_cmd = W_cmd.z();
      r.mx_cmd = W_cmd[3]; r.my_cmd = W_cmd[4]; r.mz_cmd = W_cmd[5];
      r.tau_cmd = tau_out;
      for (size_t i = 0; i < 7; ++i) {
        r.tau_J[i] = state.tau_J[i];
        r.tau_ext[i] = state.tau_ext_hat_filtered[i];
      }
      logger.log(r);

      if (time - last_console_time >= 0.5) {
        last_console_time = time;
        std::cout << "t=" << std::setw(6) << std::setprecision(2) << time
                  << " phase=" << phaseName(phase)
                  << " Fz_contact=" << std::setprecision(3) << contact_force_z
                  << " |Fcomp|=" << force_norm
                  << " p=[" << p.x() << "," << p.y() << "," << p.z() << "]\n";
      }

      if (phase == 5) return finishTorques(tau_d, state.tau_J_d);
      if (phase == 9) return finishTorques(coriolis, state.tau_J_d);

      return franka::Torques(tau_out);
    });

    logger.writeCsv(opt.output_csv);

    if (!printed_contact) {
      std::cerr << "WARNING: No contact was detected. Try smaller --contact-threshold, larger --approach-distance, or switch --force-sign.\n";
    }
    std::cout << "Experiment completed. Stop reason: " << stop_reason << "\n";

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return -1;
  }
  return 0;
}

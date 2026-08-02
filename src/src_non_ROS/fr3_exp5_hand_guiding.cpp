#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kDeltaTauMax = 1.0;  // Nm per 1 ms, conservative torque-rate saturation.

struct Options {
  std::string robot_ip = "172.16.0.2";
  std::string output = "/home/danai/fr3_force_logs/exp5_hand_guiding_xy.csv";
  std::string axis_mode = "xy";  // xy or xyz

  double duration = 45.0;
  double bias_time = 2.0;

  // Admittance model: M*xdd + D*xd + K*x = F_ext
  double virtual_mass = 4.0;       // kg
  double virtual_damping = 45.0;   // Ns/m
  double virtual_stiffness = 0.0;  // N/m, 0 means no spring to the start pose
  double force_deadband = 1.0;     // N
  double force_scale = 1.0;        // use -1 if motion is opposite to the push direction

  // Limits of the generated admittance reference
  double max_offset = 0.120;       // m, max norm of virtual displacement
  double max_velocity = 0.080;     // m/s, max norm of virtual velocity
  double max_acceleration = 0.40;  // m/s^2, per-axis clamp
  double max_z_up = 0.060;         // m
  double max_z_down = 0.040;       // m

  // Cartesian impedance tracking of admittance reference
  double trans_stiffness = 120.0;  // N/m
  double trans_damping = 28.0;     // Ns/m
  double orient_stiffness = 12.0;  // Nm/rad
  double orient_damping = 6.0;     // Nms/rad
  double joint_damping = 0.4;      // Nm/(rad/s), small nullspace-like damping

  // Safety
  double max_force = 18.0;          // N
  double max_force_comp = 14.0;     // N
  double max_ext_torque_norm = 10.0; // Nm
  double max_ee_speed = 0.15;       // m/s

  double log_rate = 100.0;
  bool risk_accepted = false;
};

std::string require_value(int& i, int argc, char** argv) {
  if (i + 1 >= argc) {
    throw std::runtime_error(std::string("Missing value for argument ") + argv[i]);
  }
  ++i;
  return std::string(argv[i]);
}

double require_double(int& i, int argc, char** argv) {
  return std::stod(require_value(i, argc, argv));
}

Options parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a == "--robot-ip") opt.robot_ip = require_value(i, argc, argv);
    else if (a == "--output") opt.output = require_value(i, argc, argv);
    else if (a == "--axis-mode") opt.axis_mode = require_value(i, argc, argv);
    else if (a == "--duration") opt.duration = require_double(i, argc, argv);
    else if (a == "--bias-time") opt.bias_time = require_double(i, argc, argv);
    else if (a == "--virtual-mass") opt.virtual_mass = require_double(i, argc, argv);
    else if (a == "--virtual-damping") opt.virtual_damping = require_double(i, argc, argv);
    else if (a == "--virtual-stiffness") opt.virtual_stiffness = require_double(i, argc, argv);
    else if (a == "--force-deadband") opt.force_deadband = require_double(i, argc, argv);
    else if (a == "--force-scale") opt.force_scale = require_double(i, argc, argv);
    else if (a == "--max-offset") opt.max_offset = require_double(i, argc, argv);
    else if (a == "--max-velocity") opt.max_velocity = require_double(i, argc, argv);
    else if (a == "--max-acceleration") opt.max_acceleration = require_double(i, argc, argv);
    else if (a == "--max-z-up") opt.max_z_up = require_double(i, argc, argv);
    else if (a == "--max-z-down") opt.max_z_down = require_double(i, argc, argv);
    else if (a == "--trans-stiffness") opt.trans_stiffness = require_double(i, argc, argv);
    else if (a == "--trans-damping") opt.trans_damping = require_double(i, argc, argv);
    else if (a == "--orient-stiffness") opt.orient_stiffness = require_double(i, argc, argv);
    else if (a == "--orient-damping") opt.orient_damping = require_double(i, argc, argv);
    else if (a == "--joint-damping") opt.joint_damping = require_double(i, argc, argv);
    else if (a == "--max-force") opt.max_force = require_double(i, argc, argv);
    else if (a == "--max-force-comp") opt.max_force_comp = require_double(i, argc, argv);
    else if (a == "--max-ext-torque-norm") opt.max_ext_torque_norm = require_double(i, argc, argv);
    else if (a == "--max-ee-speed") opt.max_ee_speed = require_double(i, argc, argv);
    else if (a == "--log-rate") opt.log_rate = require_double(i, argc, argv);
    else if (a == "--i-understand-real-robot-risk") opt.risk_accepted = true;
    else if (a == "--help") {
      std::cout <<
        "FR3 Experiment 5: Hand-Guiding / Lead-Through Control\n\n"
        "Required:\n"
        "  --robot-ip IP\n"
        "  --i-understand-real-robot-risk\n\n"
        "Useful options:\n"
        "  --axis-mode xy|xyz\n"
        "  --duration SEC\n"
        "  --virtual-mass KG\n"
        "  --virtual-damping NS_PER_M\n"
        "  --virtual-stiffness N_PER_M\n"
        "  --force-deadband N\n"
        "  --force-scale 1|-1\n"
        "  --max-offset M\n"
        "  --max-velocity M_PER_S\n"
        "  --trans-stiffness N_PER_M\n"
        "  --trans-damping NS_PER_M\n"
        "  --max-force N\n"
        "  --output CSV\n";
      std::exit(0);
    }
    else {
      throw std::runtime_error("Unknown argument: " + a);
    }
  }
  return opt;
}

void validate(const Options& opt) {
  if (!opt.risk_accepted) {
    throw std::runtime_error("Refusing to run on a real robot without --i-understand-real-robot-risk");
  }
  if (!(opt.axis_mode == "xy" || opt.axis_mode == "xyz")) {
    throw std::runtime_error("--axis-mode must be either xy or xyz");
  }
  if (opt.duration <= 1.0 || opt.duration > 180.0) {
    throw std::runtime_error("--duration must be in (1, 180] s");
  }
  if (opt.bias_time < 0.2 || opt.bias_time > 10.0) {
    throw std::runtime_error("--bias-time must be in [0.2, 10] s");
  }
  if (opt.virtual_mass <= 0.2 || opt.virtual_mass > 50.0) {
    throw std::runtime_error("--virtual-mass must be in (0.2, 50] kg");
  }
  if (opt.virtual_damping < 1.0 || opt.virtual_damping > 300.0) {
    throw std::runtime_error("--virtual-damping must be in [1, 300] Ns/m");
  }
  if (opt.virtual_stiffness < 0.0 || opt.virtual_stiffness > 300.0) {
    throw std::runtime_error("--virtual-stiffness must be in [0, 300] N/m");
  }
  if (opt.max_offset <= 0.005 || opt.max_offset > 0.25) {
    throw std::runtime_error("--max-offset must be in (0.005, 0.25] m");
  }
  if (opt.max_velocity <= 0.005 || opt.max_velocity > 0.30) {
    throw std::runtime_error("--max-velocity must be in (0.005, 0.30] m/s");
  }
  if (opt.trans_stiffness < 20.0 || opt.trans_stiffness > 1000.0) {
    throw std::runtime_error("--trans-stiffness must be in [20, 1000] N/m");
  }
  if (opt.max_force < 3.0 || opt.max_force > 60.0) {
    throw std::runtime_error("--max-force must be in [3, 60] N");
  }
  if (opt.log_rate < 1.0 || opt.log_rate > 1000.0) {
    throw std::runtime_error("--log-rate must be in [1, 1000] Hz");
  }
}

Eigen::Vector3d apply_deadband(const Eigen::Vector3d& f, double deadband) {
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    const double a = std::abs(f[i]);
    if (a > deadband) {
      out[i] = std::copysign(a - deadband, f[i]);
    }
  }
  return out;
}

Eigen::Vector3d clamp_norm(const Eigen::Vector3d& v, double max_norm) {
  const double n = v.norm();
  if (n > max_norm && n > 1e-12) {
    return v * (max_norm / n);
  }
  return v;
}

std::array<double, 7> saturate_torque_rate(const Eigen::Matrix<double, 7, 1>& tau_calculated,
                                           const std::array<double, 7>& tau_J_d) {
  std::array<double, 7> tau_saturated{};
  for (size_t i = 0; i < 7; ++i) {
    const double diff = tau_calculated(static_cast<Eigen::Index>(i)) - tau_J_d[i];
    tau_saturated[i] = tau_J_d[i] + std::clamp(diff, -kDeltaTauMax, kDeltaTauMax);
  }
  return tau_saturated;
}

void write_header(std::ofstream& log) {
  log << "time,phase_id,phase_name,";
  log << "x,y,z,x_ref,y_ref,z_ref,ex,ey,ez,";
  log << "adm_x,adm_y,adm_z,adm_vx,adm_vy,adm_vz,";
  log << "fx,fy,fz,mx,my,mz,force_norm,max_force_component,";
  log << "tau_ext_norm,ee_speed,";
  for (int i = 1; i <= 7; ++i) log << "q" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "dq" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "tau_J" << i << ",";
  for (int i = 1; i <= 7; ++i) log << "tau_cmd" << i << ",";
  for (int i = 1; i <= 7; ++i) {
    log << "tau_ext" << i;
    if (i < 7) log << ",";
  }
  log << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parse_args(argc, argv);
    validate(opt);

    std::ofstream log(opt.output);
    if (!log.is_open()) {
      throw std::runtime_error("Could not open output CSV: " + opt.output);
    }
    log << std::fixed << std::setprecision(9);
    write_header(log);

    std::cout << "Connecting to FR3 at " << opt.robot_ip << "...\n";
    franka::Robot robot(opt.robot_ip);
    franka::Model model = robot.loadModel();

    std::cout << "Experiment 5: Hand-guiding / lead-through control\n";
    std::cout << "Axis mode: " << opt.axis_mode << "\n";
    std::cout << "The robot will first estimate external force bias for " << opt.bias_time << " s.\n";
    std::cout << "Then gently push the end-effector/arm and release it. Keep E-stop ready.\n";

    double time = 0.0;
    double last_log_time = -1e9;
    const double log_period = 1.0 / opt.log_rate;
    bool initialized = false;
    bool bias_ready = false;
    bool safety_stop = false;
    std::string stop_reason = "duration_finished";

    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_d = Eigen::Quaterniond::Identity();
    Eigen::Vector3d f_bias_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_bias_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d f_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_bias = Eigen::Vector3d::Zero();
    int bias_count = 0;

    Eigen::Vector3d adm_x = Eigen::Vector3d::Zero();
    Eigen::Vector3d adm_v = Eigen::Vector3d::Zero();

    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();
      time += dt;

      const std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      const std::array<double, 7> coriolis_array = model.coriolis(state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext(state.tau_ext_hat_filtered.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> wrench_raw(state.O_F_ext_hat_K.data());
      Eigen::Map<const Eigen::Matrix<double, 4, 4>> transform(state.O_T_EE.data());

      const Eigen::Vector3d p = transform.block<3, 1>(0, 3);
      const Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
      const Eigen::Vector3d v = J.topRows<3>() * dq;
      const Eigen::Vector3d omega = J.bottomRows<3>() * dq;

      if (!initialized) {
        p0 = p;
        orientation_d = Eigen::Quaterniond(R);
        initialized = true;
        std::cout << "Initial position p0=[" << p0.transpose() << "]\n";
      }

      const Eigen::Vector3d f_raw = wrench_raw.head<3>();
      const Eigen::Vector3d m_raw = wrench_raw.tail<3>();

      int phase_id = 1;
      std::string phase_name = "hand_guiding";

      if (!bias_ready) {
        phase_id = 0;
        phase_name = "bias_hold";
        f_bias_sum += f_raw;
        m_bias_sum += m_raw;
        ++bias_count;
        if (time >= opt.bias_time && bias_count > 0) {
          f_bias = f_bias_sum / static_cast<double>(bias_count);
          m_bias = m_bias_sum / static_cast<double>(bias_count);
          bias_ready = true;
          adm_x.setZero();
          adm_v.setZero();
          std::cout << "Force bias estimated: [" << f_bias.transpose() << "] N\n";
        }
      }

      Eigen::Vector3d f_ext = f_raw - f_bias;
      Eigen::Vector3d m_ext = m_raw - m_bias;
      Eigen::Vector3d f_input = Eigen::Vector3d::Zero();

      if (bias_ready && !safety_stop) {
        f_input = opt.force_scale * apply_deadband(f_ext, opt.force_deadband);
        if (opt.axis_mode == "xy") {
          f_input.z() = 0.0;
          adm_x.z() = 0.0;
          adm_v.z() = 0.0;
        }

        Eigen::Vector3d adm_a = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; ++i) {
          adm_a[i] = (f_input[i] - opt.virtual_damping * adm_v[i] - opt.virtual_stiffness * adm_x[i]) /
                     opt.virtual_mass;
          adm_a[i] = std::clamp(adm_a[i], -opt.max_acceleration, opt.max_acceleration);
        }

        adm_v += adm_a * dt;
        adm_v = clamp_norm(adm_v, opt.max_velocity);
        adm_x += adm_v * dt;
        adm_x = clamp_norm(adm_x, opt.max_offset);
        adm_x.z() = std::clamp(adm_x.z(), -opt.max_z_down, opt.max_z_up);
      }

      Eigen::Vector3d p_ref = p0 + adm_x;
      if (opt.axis_mode == "xy") {
        p_ref.z() = p0.z();
      }

      const Eigen::Vector3d pos_error = p_ref - p;
      const Eigen::Vector3d vel_error = adm_v - v;

      Eigen::Vector3d F_cmd = opt.trans_stiffness * pos_error + opt.trans_damping * vel_error;

      Eigen::Quaterniond orientation(R);
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() *= -1.0;
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      Eigen::Vector3d orientation_error(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
      orientation_error = -R * orientation_error;
      Eigen::Vector3d M_cmd = -opt.orient_stiffness * orientation_error - opt.orient_damping * omega;

      Eigen::Matrix<double, 6, 1> wrench_cmd;
      wrench_cmd.head<3>() = F_cmd;
      wrench_cmd.tail<3>() = M_cmd;

      Eigen::Matrix<double, 7, 1> tau_cmd = J.transpose() * wrench_cmd + coriolis - opt.joint_damping * dq;
      std::array<double, 7> tau_cmd_array = saturate_torque_rate(tau_cmd, state.tau_J_d);

      const double force_norm = f_ext.norm();
      const double max_force_component = f_ext.cwiseAbs().maxCoeff();
      const double tau_ext_norm = tau_ext.norm();
      const double ee_speed = v.norm();

      if (bias_ready && !safety_stop) {
        if (force_norm > opt.max_force || max_force_component > opt.max_force_comp ||
            tau_ext_norm > opt.max_ext_torque_norm || ee_speed > opt.max_ee_speed) {
          safety_stop = true;
          phase_id = 2;
          phase_name = "safety_stop";
          std::ostringstream oss;
          oss << "safety_stop: force_norm=" << force_norm
              << " max_force_component=" << max_force_component
              << " tau_ext_norm=" << tau_ext_norm
              << " ee_speed=" << ee_speed;
          stop_reason = oss.str();
          std::cout << "Safety stop triggered at t=" << time << " " << stop_reason << "\n";
        }
      }

      if (time - last_log_time >= log_period || safety_stop || time >= opt.duration) {
        last_log_time = time;
        log << time << "," << phase_id << "," << phase_name << ",";
        log << p.x() << "," << p.y() << "," << p.z() << ",";
        log << p_ref.x() << "," << p_ref.y() << "," << p_ref.z() << ",";
        log << pos_error.x() << "," << pos_error.y() << "," << pos_error.z() << ",";
        log << adm_x.x() << "," << adm_x.y() << "," << adm_x.z() << ",";
        log << adm_v.x() << "," << adm_v.y() << "," << adm_v.z() << ",";
        log << f_ext.x() << "," << f_ext.y() << "," << f_ext.z() << ",";
        log << m_ext.x() << "," << m_ext.y() << "," << m_ext.z() << ",";
        log << force_norm << "," << max_force_component << "," << tau_ext_norm << "," << ee_speed << ",";
        for (int i = 0; i < 7; ++i) log << q[i] << ",";
        for (int i = 0; i < 7; ++i) log << dq[i] << ",";
        for (int i = 0; i < 7; ++i) log << tau_J[i] << ",";
        for (int i = 0; i < 7; ++i) log << tau_cmd_array[static_cast<size_t>(i)] << ",";
        for (int i = 0; i < 7; ++i) {
          log << tau_ext[i];
          if (i < 6) log << ",";
        }
        log << "\n";
      }

      if (safety_stop || time >= opt.duration) {
        if (time >= opt.duration && !safety_stop) {
          stop_reason = "duration_finished";
        }
        return franka::MotionFinished(franka::Torques(tau_cmd_array));
      }

      return franka::Torques(tau_cmd_array);
    });

    log.close();
    std::cout << "Motion finished. Stop reason: " << stop_reason << "\n";
    std::cout << "Saved CSV log: " << opt.output << "\n";
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }
}

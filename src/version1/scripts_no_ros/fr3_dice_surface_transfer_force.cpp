
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
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeltaTauMax = 0.7;  // Nm/cycle, torque-rate limit

enum class Phase {
  kBiasHold = 0,
  kSlideWithForce = 1,
  kStopAndReleaseForce = 2,
  kDone = 3
};

struct Config {
  // ============================================================
  // USER PARAMETERS
  // ============================================================

  // Dice x/y in robot base frame.
  // You said the dice is at (0.45, 0, -13 cm).
  // The z=-0.13 is a table reference, NOT directly an EE command.
  double dice_x = 0.45;
  double dice_y = 0.00;
  double table_z_reference_m = -0.13;

  // Destination on the same table plane.
  // Start with a small move. After it works, use a farther target.
  double target_x = 0.35;
  double target_y = 0.00;

  // EE z values. These must be calibrated for your gripper/table.
  // The robot commands the EE frame, not the dice center.
  double pregrasp_ee_z = 0.065;
  double grasp_ee_z = 0.040;

  // Gripper settings for a small dice.
  // If grasp is not confirmed, adjust dice_width / epsilons / force.
  double gripper_open_width_m = 0.08;
  double dice_width_m = 0.040;          // typical small die: 16 mm
  double gripper_speed_mps = 0.02;
  double gripper_force_N = 25.0;
  double grasp_epsilon_inner_m = 0.006;
  double grasp_epsilon_outer_m = 0.010;

  // If true, the program stops when the gripper does not confirm grasp.
  // For a small dice, libfranka may sometimes not confirm even if it is lightly held.
  bool abort_if_grasp_fails = false;

  // Top-down orientation.
  double roll = kPi;
  double pitch = 0.0;
  double yaw = 0.0;

  // Normal force through the dice during sliding.
  // Start low.
  double normal_force_target_N = 0.8;

  // Force signs:
  // From your tests, -1.0 commanded downward motion/force toward the table.
  double force_command_sign = -1.0;

  // From your contact logs, downward/table contact often made Fz_raw - Fz_bias negative.
  // So contact_measure_sign = -1 makes contact force positive.
  double contact_measure_sign = -1.0;

  // Timings.
  double bias_hold_sec = 2.0;
  double slide_duration_sec = 18.0;
  double stop_duration_sec = 1.5;
  double release_force_sec = 4.0;
  double final_settle_sec = 3.0;

  // Safety.
  double max_down_from_slide_start_m = 0.012;  // 12 mm max downward during slide
  double min_contact_warning_N = 0.3;          // warning only

  // Position gains for x/y and orientation.
  double k_xy = 650.0;
  double d_xy = 2.0 * std::sqrt(650.0);

  double k_rot = 30.0;
  double d_rot = 2.0 * std::sqrt(30.0);

  // Force control along z.
  double k_force = 0.25;
  double d_force = 35.0;

  // Safety wall if z moves too low.
  double k_z_wall = 2000.0;

  // Safety clamps.
  double max_cartesian_force_N = 10.0;
  double max_cartesian_moment_Nm = 6.0;
  
  // limit for z force
  double max_z_force_N = 4.0;
  double force_feedforward_gain = 0.4;

  std::string csv_path = "dice_surface_transfer_force_log.csv";
};

struct LogSample {
  double t;
  double phase;

  double x, y, z;
  double xd, yd, zd;
  double ex, ey, ez;
  double vx, vy, vz;

  double fz_raw;
  double fz_bias;
  double fz_delta;
  double fz_contact;
  double fz_target;
  double fz_cmd;

  double fx_cmd, fy_cmd;
  double mx_cmd, my_cmd, mz_cmd;

  double contact_warning;
  double z_min;

  double tau1, tau2, tau3, tau4, tau5, tau6, tau7;
  double tau_ext1, tau_ext2, tau_ext3, tau_ext4, tau_ext5, tau_ext6, tau_ext7;
};

Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

std::array<double, 16> eigenToArray(const Eigen::Matrix4d& T) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
  return out;
}

double smootherStep(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 10.0 * std::pow(s, 3)
       - 15.0 * std::pow(s, 4)
       + 6.0 * std::pow(s, 5);
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

Eigen::Matrix<double, 7, 1> saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_calc,
    const Eigen::Matrix<double, 7, 1>& tau_prev) {
  Eigen::Matrix<double, 7, 1> tau_sat;
  for (int i = 0; i < 7; ++i) {
    double diff = tau_calc(i) - tau_prev(i);
    diff = std::max(std::min(diff, kDeltaTauMax), -kDeltaTauMax);
    tau_sat(i) = tau_prev(i) + diff;
  }
  return tau_sat;
}

Eigen::Matrix4d makeTransform(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw) {
  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

void writeCsv(const std::string& path, const std::vector<LogSample>& samples) {
  std::ofstream f(path);
  if (!f.is_open()) {
    std::cerr << "Could not open CSV file: " << path << std::endl;
    return;
  }

  f << "t,phase,"
    << "x,y,z,xd,yd,zd,ex,ey,ez,vx,vy,vz,"
    << "fz_raw,fz_bias,fz_delta,fz_contact,fz_target,fz_cmd,"
    << "fx_cmd,fy_cmd,mx_cmd,my_cmd,mz_cmd,"
    << "contact_warning,z_min,"
    << "tau1,tau2,tau3,tau4,tau5,tau6,tau7,"
    << "tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7\n";

  f << std::fixed << std::setprecision(8);

  for (const auto& s : samples) {
    f << s.t << "," << s.phase << ","
      << s.x << "," << s.y << "," << s.z << ","
      << s.xd << "," << s.yd << "," << s.zd << ","
      << s.ex << "," << s.ey << "," << s.ez << ","
      << s.vx << "," << s.vy << "," << s.vz << ","
      << s.fz_raw << "," << s.fz_bias << "," << s.fz_delta << ","
      << s.fz_contact << "," << s.fz_target << "," << s.fz_cmd << ","
      << s.fx_cmd << "," << s.fy_cmd << ","
      << s.mx_cmd << "," << s.my_cmd << "," << s.mz_cmd << ","
      << s.contact_warning << "," << s.z_min << ","
      << s.tau1 << "," << s.tau2 << "," << s.tau3 << ","
      << s.tau4 << "," << s.tau5 << "," << s.tau6 << "," << s.tau7 << ","
      << s.tau_ext1 << "," << s.tau_ext2 << "," << s.tau_ext3 << ","
      << s.tau_ext4 << "," << s.tau_ext5 << "," << s.tau_ext6 << "," << s.tau_ext7 << "\n";
  }

  std::cout << "CSV written to: " << path << std::endl;
}

void moveToPose(franka::Robot& robot,
                const Eigen::Matrix4d& target_T,
                double duration_sec,
                double max_allowed_distance_m = 0.50) {
  franka::RobotState initial_state = robot.readOnce();
  Eigen::Matrix4d start_T = arrayToEigen(initial_state.O_T_EE);

  Eigen::Vector3d p0 = start_T.block<3, 1>(0, 3);
  Eigen::Vector3d p1 = target_T.block<3, 1>(0, 3);

  double distance = (p1 - p0).norm();

  std::cout << "Current EE position: x=" << p0.x()
            << " y=" << p0.y()
            << " z=" << p0.z() << std::endl;
  std::cout << "Target EE position:  x=" << p1.x()
            << " y=" << p1.y()
            << " z=" << p1.z() << std::endl;
  std::cout << "Distance: " << distance << " m" << std::endl;

  if (distance > max_allowed_distance_m) {
    throw std::runtime_error("ABORT: target too far from current EE pose.");
  }

  Eigen::Quaterniond q0(start_T.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(target_T.block<3, 3>(0, 0));
  q0.normalize();
  q1.normalize();

  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }

  double time = 0.0;

  robot.control(
      [&](const franka::RobotState&, franka::Duration period)
          -> franka::CartesianPose {
        time += period.toSec();
        double s = std::min(time / duration_sec, 1.0);
        double alpha = smootherStep(s);

        Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        Eigen::Quaterniond q = q0.slerp(alpha, q1);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = p;

        franka::CartesianPose output_pose(eigenToArray(T));

        if (s >= 1.0) {
          return franka::MotionFinished(output_pose);
        }

        return output_pose;
      },
      franka::ControllerMode::kCartesianImpedance,
      true);
}

void slideOnSurfaceWithNormalForce(franka::Robot& robot, const Config& cfg) {
  franka::Model model = robot.loadModel();
  franka::RobotState initial_state = robot.readOnce();

  Eigen::Matrix4d T0 = arrayToEigen(initial_state.O_T_EE);
  Eigen::Vector3d p_start = T0.block<3, 1>(0, 3);
  Eigen::Quaterniond q_d(T0.block<3, 3>(0, 0));
  q_d.normalize();

  const double z_start = p_start.z();
  const double z_min = z_start - cfg.max_down_from_slide_start_m;

  Eigen::Vector3d p_target(cfg.target_x, cfg.target_y, z_start);

  std::cout << "\nSurface transfer starts from:"
            << " x=" << p_start.x()
            << " y=" << p_start.y()
            << " z=" << p_start.z() << std::endl;
  std::cout << "Surface transfer target:"
            << " x=" << p_target.x()
            << " y=" << p_target.y()
            << " z≈current surface z" << std::endl;
  std::cout << "z_min safety wall: " << z_min << " m" << std::endl;

  const double max_total_time =
      cfg.bias_hold_sec + cfg.slide_duration_sec +
      cfg.release_force_sec + cfg.stop_duration_sec +
      cfg.final_settle_sec + 4.0;

  std::vector<LogSample> samples;
  samples.resize(static_cast<size_t>(max_total_time * 100.0) + 1000);
  size_t sample_count = 0;

  Phase phase = Phase::kBiasHold;
  double time = 0.0;
  double phase_start_time = 0.0;
  double last_log = -1.0;
  double last_print = -1.0;

  double fz_bias_sum = 0.0;
  size_t fz_bias_count = 0;
  double fz_bias = initial_state.O_F_ext_hat_K[2];

  bool contact_warning_printed = false;

  robot.control(
      [&](const franka::RobotState& state, franka::Duration period)
          -> franka::Torques {
        const double dt = period.toSec();
        time += dt;

        auto coriolis_array = model.coriolis(state);
        auto jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_prev(state.tau_J_d.data());

        Eigen::Matrix<double, 6, 1> v = J * dq;

        Eigen::Matrix4d T = arrayToEigen(state.O_T_EE);
        Eigen::Vector3d p = T.block<3, 1>(0, 3);
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        q.normalize();

        const double fz_raw = state.O_F_ext_hat_K[2];

        if (phase == Phase::kBiasHold) {
          fz_bias_sum += fz_raw;
          fz_bias_count += 1;

          if (time >= cfg.bias_hold_sec) {
            if (fz_bias_count > 0) {
              fz_bias = fz_bias_sum / static_cast<double>(fz_bias_count);
            }

            phase = Phase::kSlideWithForce;
            phase_start_time = time;

            std::cout << "Bias done. Fz_bias = " << fz_bias << " N" << std::endl;
            std::cout << "Starting x/y transfer while applying normal z force..." << std::endl;
          }
        }

        const double fz_delta = fz_raw - fz_bias;
        const double fz_contact = cfg.contact_measure_sign * fz_delta;

        Eigen::Vector3d p_d = p_start;
        double fz_target = 0.0;

        if (phase == Phase::kSlideWithForce) {
          double s = (time - phase_start_time) / cfg.slide_duration_sec;
          double alpha = smootherStep(s);

          p_d.x() = p_start.x() + alpha * (p_target.x() - p_start.x());
          p_d.y() = p_start.y() + alpha * (p_target.y() - p_start.y());
          p_d.z() = z_start;

          // Ramp normal force smoothly at the beginning.
          double force_alpha = smooth01(std::min((time - phase_start_time) / 3.0, 1.0));
          fz_target = cfg.normal_force_target_N * force_alpha;

          if ((time - phase_start_time) > 3.5 &&
              std::abs(fz_contact) < cfg.min_contact_warning_N &&
              !contact_warning_printed) {
            contact_warning_printed = true;
            std::cout << "WARNING: very small measured contact force during slide. "
                      << "The dice may not be touching the table or may not be held correctly."
                      << std::endl;
          }

          if (s >= 1.0) {
            phase = Phase::kStopAndReleaseForce;
            phase_start_time = time;
            std::cout << "x/y transfer done. Stopping movement and releasing normal force..." << std::endl;
          }
        } else if (phase == Phase::kStopAndReleaseForce) {
          p_d = p_target;

          double s = (time - phase_start_time) / cfg.release_force_sec;
          fz_target = cfg.normal_force_target_N * (1.0 - smooth01(s));

          if (s >= 1.0) {
            phase = Phase::kDone;
            phase_start_time = time;
            std::cout << "Normal force released. Final settle before opening gripper..." << std::endl;
          }
        } else if (phase == Phase::kDone) {
  		p_d = p_target;   // keep final target position during final settle
  		fz_target = 0.0;
	}
        Eigen::Matrix<double, 6, 1> err;
        err.setZero();

        err(0) = p.x() - p_d.x();
        err(1) = p.y() - p_d.y();

        // z is not position-controlled during slide; this is logged only.
        err(2) = p.z() - p_d.z();

        if (q_d.coeffs().dot(q.coeffs()) < 0.0) {
          q.coeffs() *= -1.0;
        }

        Eigen::Quaterniond qe(q.inverse() * q_d);
        err.tail<3>() << qe.x(), qe.y(), qe.z();
        err.tail<3>() = -R * err.tail<3>();

        Eigen::Matrix<double, 6, 1> wrench;
        wrench.setZero();

        // Position-controlled subspace: x/y.
        wrench(0) = -cfg.k_xy * err(0) - cfg.d_xy * v(0);
        wrench(1) = -cfg.k_xy * err(1) - cfg.d_xy * v(1);

        // Force-controlled subspace: z.
        if (phase == Phase::kSlideWithForce || phase == Phase::kStopAndReleaseForce) {
          double force_error = fz_target - fz_contact;

          // Feedforward target force + proportional correction.
          double z_force_cmd =
    		cfg.force_command_sign * (cfg.force_feedforward_gain * fz_target + cfg.k_force * force_error) - cfg.d_force * v(2);

          // Extra clamp only for z force.
	  // This prevents sudden large pushing force through the dice.
	  z_force_cmd = std::max(std::min(z_force_cmd, cfg.max_z_force_N), -cfg.max_z_force_N);

	  wrench(2) = z_force_cmd;
        } else {
          wrench(2) = 0.0;
        }

        // Safety wall if the robot moves too low.
        if (p.z() < z_min) {
          wrench(2) += cfg.k_z_wall * (z_min - p.z());
        }

        // Orientation position control.
        wrench(3) = -cfg.k_rot * err(3) - cfg.d_rot * v(3);
        wrench(4) = -cfg.k_rot * err(4) - cfg.d_rot * v(4);
        wrench(5) = -cfg.k_rot * err(5) - cfg.d_rot * v(5);

        Eigen::Vector3d f_cmd = clampNorm(wrench.head<3>(), cfg.max_cartesian_force_N);
        Eigen::Vector3d m_cmd = clampNorm(wrench.tail<3>(), cfg.max_cartesian_moment_Nm);
        wrench.head<3>() = f_cmd;
        wrench.tail<3>() = m_cmd;

        Eigen::Matrix<double, 7, 1> tau_calc = J.transpose() * wrench + coriolis;
        Eigen::Matrix<double, 7, 1> tau = saturateTorqueRate(tau_calc, tau_prev);

        std::array<double, 7> tau_out{};
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau;

        if (time - last_log >= 0.01 && sample_count < samples.size()) {
          last_log = time;

          LogSample s{};
          s.t = time;
          s.phase = static_cast<int>(phase);

          s.x = p.x();
          s.y = p.y();
          s.z = p.z();

          s.xd = p_d.x();
          s.yd = p_d.y();
          s.zd = p_d.z();

          s.ex = err(0);
          s.ey = err(1);
          s.ez = err(2);

          s.vx = v(0);
          s.vy = v(1);
          s.vz = v(2);

          s.fz_raw = fz_raw;
          s.fz_bias = fz_bias;
          s.fz_delta = fz_delta;
          s.fz_contact = fz_contact;
          s.fz_target = fz_target;
          s.fz_cmd = wrench(2);

          s.fx_cmd = wrench(0);
          s.fy_cmd = wrench(1);
          s.mx_cmd = wrench(3);
          s.my_cmd = wrench(4);
          s.mz_cmd = wrench(5);

          s.contact_warning = (std::abs(fz_contact) < cfg.min_contact_warning_N) ? 1.0 : 0.0;
          s.z_min = z_min;

          s.tau1 = tau_out[0];
          s.tau2 = tau_out[1];
          s.tau3 = tau_out[2];
          s.tau4 = tau_out[3];
          s.tau5 = tau_out[4];
          s.tau6 = tau_out[5];
          s.tau7 = tau_out[6];

          s.tau_ext1 = state.tau_ext_hat_filtered[0];
          s.tau_ext2 = state.tau_ext_hat_filtered[1];
          s.tau_ext3 = state.tau_ext_hat_filtered[2];
          s.tau_ext4 = state.tau_ext_hat_filtered[3];
          s.tau_ext5 = state.tau_ext_hat_filtered[4];
          s.tau_ext6 = state.tau_ext_hat_filtered[5];
          s.tau_ext7 = state.tau_ext_hat_filtered[6];

          samples[sample_count++] = s;
        }

        if (time - last_print >= 0.5) {
          last_print = time;
          double xy_err_mm = std::sqrt(err(0) * err(0) + err(1) * err(1)) * 1000.0;

          std::cout << std::fixed << std::setprecision(4)
                    << "t=" << time
                    << " phase=" << static_cast<int>(phase)
                    << " pos=[" << p.x() << ", " << p.y() << ", " << p.z() << "]"
                    << " des=[" << p_d.x() << ", " << p_d.y() << ", " << p_d.z() << "]"
                    << " Fz_contact=" << fz_contact
                    << " Fz_target=" << fz_target
                    << " Fz_cmd=" << wrench(2)
                    << " xy_err=" << xy_err_mm << " mm"
                    << std::endl;
        }

        franka::Torques output(tau_out);

        bool finished =
            (phase == Phase::kDone) &&
            ((time - phase_start_time) >= cfg.final_settle_sec);

        bool timeout = time >= max_total_time;

        if (finished || timeout) {
          return franka::MotionFinished(output);
        }

        return output;
      },
      true);

  samples.resize(sample_count);
  writeCsv(cfg.csv_path, samples);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string robot_ip = "172.16.0.2";
    if (argc >= 2) {
      robot_ip = argv[1];
    }

    Config cfg;

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;
    std::cout << "Dice surface transfer with z normal force" << std::endl;
    std::cout << "Sequence:" << std::endl;
    std::cout << "  open gripper -> pregrasp -> grasp/close -> transfer on surface + z force -> stop -> open gripper" << std::endl;

    std::cout << "\nDice reference: x=" << cfg.dice_x
              << " y=" << cfg.dice_y
              << " table_z=" << cfg.table_z_reference_m << " m" << std::endl;
    std::cout << "Target: x=" << cfg.target_x
              << " y=" << cfg.target_y << std::endl;
    std::cout << "EE pregrasp z=" << cfg.pregrasp_ee_z
              << " grasp z=" << cfg.grasp_ee_z << std::endl;
    std::cout << "Normal force target=" << cfg.normal_force_target_N << " N" << std::endl;

    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);

    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    Eigen::Matrix4d T_pregrasp =
        makeTransform(cfg.dice_x, cfg.dice_y, cfg.pregrasp_ee_z,
                      cfg.roll, cfg.pitch, cfg.yaw);

    Eigen::Matrix4d T_grasp =
        makeTransform(cfg.dice_x, cfg.dice_y, cfg.grasp_ee_z,
                      cfg.roll, cfg.pitch, cfg.yaw);

    std::cout << "\nIMPORTANT:" << std::endl;
    std::cout << "  This program does NOT use contact approach." << std::endl;
    std::cout << "  It assumes that after closing the gripper, the dice is already on/near the table surface." << std::endl;
    std::cout << "  If the gripper does not actually hold the dice, the force-control slide will not work." << std::endl;
    std::cout << "Press ENTER to start..." << std::endl;
    std::cin.ignore();

    // Step 1
    std::cout << "\nStep 1: Open gripper..." << std::endl;
    if (!gripper.move(cfg.gripper_open_width_m, cfg.gripper_speed_mps)) {
      throw std::runtime_error("Gripper open failed.");
    }

    // Step 2
    std::cout << "\nStep 2: Move to pregrasp..." << std::endl;
    moveToPose(robot, T_pregrasp, 8.0);

    // Step 3
    std::cout << "\nStep 3: Move to grasp pose..." << std::endl;
    moveToPose(robot, T_grasp, 6.0);

    // Step 4
    std::cout << "\nStep 4: Close gripper on dice..." << std::endl;
    bool grasp_success = gripper.grasp(
        cfg.dice_width_m,
        cfg.gripper_speed_mps,
        cfg.gripper_force_N,
        cfg.grasp_epsilon_inner_m,
        cfg.grasp_epsilon_outer_m);

    if (!grasp_success) {
      std::cerr << "WARNING: Gripper did not confirm grasp." << std::endl;
      if (cfg.abort_if_grasp_fails) {
        throw std::runtime_error("Aborting because grasp was not confirmed.");
      }
      std::cerr << "Continuing because abort_if_grasp_fails=false." << std::endl;
    } else {
      std::cout << "Gripper grasp confirmed." << std::endl;
    }

    // Step 5
    std::cout << "\nStep 5: Transfer on surface while applying z force..." << std::endl;
    slideOnSurfaceWithNormalForce(robot, cfg);

    // Step 6
    std::cout << "\nStep 6: Open gripper / release dice..." << std::endl;
    if (!gripper.move(cfg.gripper_open_width_m, cfg.gripper_speed_mps)) {
      std::cerr << "WARNING: Gripper open failed." << std::endl;
    }

    std::cout << "\nDone." << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

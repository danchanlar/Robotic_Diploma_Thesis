#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/control_types.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

// ------------------------------------------------------------
// Convert Eigen 4x4 transform to libfranka array.
// IMPORTANT: libfranka uses column-major order.
// ------------------------------------------------------------
std::array<double, 16> eigenToArray(const Eigen::Matrix4d& T) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
  return out;
}

// ------------------------------------------------------------
// Convert libfranka array to Eigen 4x4 transform.
// IMPORTANT: libfranka O_T_EE is column-major.
// ------------------------------------------------------------
Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

// ------------------------------------------------------------
// Build homogeneous transform from x,y,z,roll,pitch,yaw.
// Rotation convention:
// R = Rz(yaw) * Ry(pitch) * Rx(roll)
// This is the usual fixed-axis RPY convention used in ROS/URDF.
// ------------------------------------------------------------
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

void printPose(const std::string& name, const Eigen::Matrix4d& T) {
  std::cout << "\n" << name << ":\n" << T << "\n" << std::endl;
}


struct LogRow {
  double t;
  int phase_id;
  std::string phase_name;

  double x;
  double y;
  double z;

  double x_des;
  double y_des;
  double z_des;

  double ex;
  double ey;
  double ez;

  double fx;
  double fy;
  double fz;
  double mx;
  double my;
  double mz;
  double force_norm;

  double tau_ext_norm;
  double ee_speed;

  std::array<double, 7> tau_J;
  std::array<double, 7> tau_ext;
};

class DataLogger {
 public:
  DataLogger() {
    rows_.reserve(250000);
  }

  void log(double t,
           int phase_id,
           const std::string& phase_name,
           const franka::RobotState& state,
           const Eigen::Vector3d& p_des,
           double ee_speed) {
    if (rows_.size() >= 250000) {
      return;
    }

    Eigen::Matrix4d T_actual = arrayToEigen(state.O_T_EE);
    Eigen::Vector3d p_actual = T_actual.block<3, 1>(0, 3);

    const auto& F = state.O_F_ext_hat_K;

    LogRow r{};
    r.t = t;
    r.phase_id = phase_id;
    r.phase_name = phase_name;

    r.x = p_actual.x();
    r.y = p_actual.y();
    r.z = p_actual.z();

    r.x_des = p_des.x();
    r.y_des = p_des.y();
    r.z_des = p_des.z();

    r.ex = r.x_des - r.x;
    r.ey = r.y_des - r.y;
    r.ez = r.z_des - r.z;

    r.fx = F[0];
    r.fy = F[1];
    r.fz = F[2];
    r.mx = F[3];
    r.my = F[4];
    r.mz = F[5];

    r.force_norm = std::sqrt(r.fx * r.fx + r.fy * r.fy + r.fz * r.fz);

    r.tau_ext_norm = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      r.tau_J[i] = state.tau_J[i];
      r.tau_ext[i] = state.tau_ext_hat_filtered[i];
      r.tau_ext_norm += r.tau_ext[i] * r.tau_ext[i];
    }
    r.tau_ext_norm = std::sqrt(r.tau_ext_norm);

    r.ee_speed = ee_speed;

    rows_.push_back(r);
  }

  void writeCsv(const std::string& path) const {
    std::ofstream out(path);
    if (!out.is_open()) {
      throw std::runtime_error("Could not open CSV output file: " + path);
    }

    out << "time,phase_id,phase_name,"
        << "x,y,z,x_des,y_des,z_des,ex,ey,ez,"
        << "fx,fy,fz,mx,my,mz,force_norm,"
        << "tau_ext_norm,ee_speed,"
        << "tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7,"
        << "tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,tau_ext7\n";

    out << std::fixed << std::setprecision(9);

    for (const auto& r : rows_) {
      out << r.t << ","
          << r.phase_id << ","
          << r.phase_name << ","
          << r.x << "," << r.y << "," << r.z << ","
          << r.x_des << "," << r.y_des << "," << r.z_des << ","
          << r.ex << "," << r.ey << "," << r.ez << ","
          << r.fx << "," << r.fy << "," << r.fz << ","
          << r.mx << "," << r.my << "," << r.mz << ","
          << r.force_norm << ","
          << r.tau_ext_norm << ","
          << r.ee_speed;

      for (size_t i = 0; i < 7; ++i) {
        out << "," << r.tau_J[i];
      }

      for (size_t i = 0; i < 7; ++i) {
        out << "," << r.tau_ext[i];
      }

      out << "\n";
    }

    std::cout << "Wrote CSV log with " << rows_.size()
              << " samples to: " << path << std::endl;
  }

 private:
  std::vector<LogRow> rows_;
};


// ------------------------------------------------------------
// Smooth Cartesian motion from current EE pose to target EE pose.
// Position: cosine smooth interpolation.
// Orientation: quaternion slerp.
// ------------------------------------------------------------
void moveToPose(franka::Robot& robot,
                const Eigen::Matrix4d& target_T,
                double duration_sec,
                DataLogger& logger,
                int phase_id,
                const std::string& phase_name,
                double& experiment_time,
                double max_allowed_distance_m = 0.50) {
  franka::RobotState initial_state = robot.readOnce();

  Eigen::Matrix4d start_T = arrayToEigen(initial_state.O_T_EE);

  Eigen::Vector3d p0 = start_T.block<3, 1>(0, 3);
  Eigen::Vector3d p1 = target_T.block<3, 1>(0, 3);

  double distance = (p1 - p0).norm();

  std::cout << "\nPhase " << phase_id << " - " << phase_name << std::endl;

  std::cout << "Current EE position: "
            << "x=" << p0.x()
            << " y=" << p0.y()
            << " z=" << p0.z() << std::endl;

  std::cout << "Target EE position:  "
            << "x=" << p1.x()
            << " y=" << p1.y()
            << " z=" << p1.z() << std::endl;

  std::cout << "Distance to target: " << distance << " m" << std::endl;

  if (distance > max_allowed_distance_m) {
    throw std::runtime_error(
        "ABORT: target is too far from current EE pose. Move the robot closer first or increase max_allowed_distance_m carefully.");
  }

  Eigen::Quaterniond q0(start_T.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(target_T.block<3, 3>(0, 0));

  q0.normalize();
  q1.normalize();

  // Use the shortest quaternion path.
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }

  double time = 0.0;
  double last_log_time = -1.0;

  Eigen::Vector3d previous_actual_position = p0;
  bool has_previous_position = false;

  robot.control(
      [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::CartesianPose {
        const double dt = period.toSec();
        time += dt;
        experiment_time += dt;

        double s = std::min(time / duration_sec, 1.0);

        // Smoothstep 0 -> 1 with zero velocity at beginning/end.
        double alpha = 0.5 - 0.5 * std::cos(kPi * s);

        Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        Eigen::Quaterniond q = q0.slerp(alpha, q1);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = p;

        Eigen::Matrix4d T_actual = arrayToEigen(robot_state.O_T_EE);
        Eigen::Vector3d actual_position = T_actual.block<3, 1>(0, 3);

        double ee_speed = 0.0;
        if (has_previous_position && dt > 1e-9) {
          ee_speed = (actual_position - previous_actual_position).norm() / dt;
        }
        previous_actual_position = actual_position;
        has_previous_position = true;

        logger.log(experiment_time,
                   phase_id,
                   phase_name,
                   robot_state,
                   p,
                   ee_speed);

        // Terminal print every 0.25 sec.
        if (time - last_log_time > 0.25) {
          last_log_time = time;
          const auto& F = robot_state.O_F_ext_hat_K;
          std::cout << std::fixed << std::setprecision(3)
                    << "  t=" << time
                    << "  F_ext=["
                    << F[0] << ", " << F[1] << ", " << F[2]
                    << "] N"
                    << "  |F|=" << std::sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2])
                    << " N"
                    << std::endl;
        }

        std::array<double, 16> pose_array = eigenToArray(T);
        franka::CartesianPose output_pose(pose_array);

        if (s >= 1.0) {
          return franka::MotionFinished(output_pose);
        }

        return output_pose;
      },
      franka::ControllerMode::kCartesianImpedance,
      true);
}


// ------------------------------------------------------------
// Set payload for the object after successful grasp.
// F_x_Cload is expressed in flange frame F, in meters.
// Inertia is kg*m^2, column-major.
// ------------------------------------------------------------
void setObjectLoad(franka::Robot& robot,
                   double object_mass_kg,
                   const std::array<double, 3>& F_x_Cload,
                   const std::array<double, 9>& load_inertia) {
  std::cout << "Setting object load: " << object_mass_kg << " kg" << std::endl;
  robot.setLoad(object_mass_kg, F_x_Cload, load_inertia);
}

// ------------------------------------------------------------
// Clear external object payload after releasing the object.
// ------------------------------------------------------------
void clearObjectLoad(franka::Robot& robot) {
  std::cout << "Clearing object load." << std::endl;

  robot.setLoad(
      0.0,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0,
       0.0, 0.0, 0.0,
       0.0, 0.0, 0.0});
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string robot_ip = "172.16.0.2";

    if (argc >= 2) {
      robot_ip = argv[1];
    }

    std::string output_csv = "/home/danai/fr3_force_logs/pick_place_ball.csv";
    if (argc >= 3) {
      output_csv = argv[2];
    }

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;
    std::cout << "CSV log file: " << output_csv << std::endl;

    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);

    // Optional recovery. If no recovery is needed, it may throw; that is okay.
    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    DataLogger logger;
    double experiment_time = 0.0;

    // Ball current position in robot base frame, in meters.
    const double pick_x = 0.45;
    const double pick_y = 0.00;
    const double pick_z = -0.05;

    // Ball target/place position in robot base frame, in meters.
    const double place_x = 0.35;
    const double place_y = -0.25;
    const double place_z = -0.05;

    // Top-down gripper orientation.
    // EE/TCP z-axis points downward.
    const double roll = kPi;
    const double pitch = 0.0;
    const double yaw = 0.0;

    // Safe vertical offsets.
    const double pregrasp_height = 0.14;   // 14 cm above ball reference z
    const double grasp_height    = 0.07;   // 7 cm above ball reference z, lower gradually if needed
    const double lift_height     = 0.25;   // absolute z while carrying
    const double transfer_height = 0.25;   // absolute z during transfer

    // Gripper settings.
    // Franka Hand max opening is about 0.08 m.
    const double open_width = 0.08;
    const double gripper_speed_open = 0.03;

  
    // ball diameter = 4 cm -> grasp_width around 0.040 to 0.055 m.
    const double grasp_width = 0.040;
    const double grasp_speed = 0.02;
    const double grasp_force = 50.0;
    const double epsilon_inner = 0.010;
    const double epsilon_outer = 0.015;

    // Object load estimate.
    const double object_mass_kg = 0.37;

    // Safe starting estimate: 10 cm along flange z direction.
    const std::array<double, 3> F_x_Cload = {
        0.0,
        0.0,
        0.10
    };

    // Approximate inertia of a small/light ball.
    // For a solid sphere: I = 2/5 * m * r^2.
    // For m=0.10 kg, r=0.025 m, I≈0.000025 kg*m^2.

    const std::array<double, 9> load_inertia = {
        0.00003, 0.0,     0.0,
        0.0,     0.00003, 0.0,
        0.0,     0.0,     0.00003
    };

    // ============================================================
    // BUILD POSES
    // ============================================================

    Eigen::Matrix4d T_pregrasp =
        makeTransform(pick_x, pick_y, pick_z + pregrasp_height, roll, pitch, yaw);

    Eigen::Matrix4d T_grasp =
        makeTransform(pick_x, pick_y, pick_z + grasp_height, roll, pitch, yaw);

    Eigen::Matrix4d T_lift =
        makeTransform(pick_x, pick_y, lift_height, roll, pitch, yaw);

    Eigen::Matrix4d T_transfer =
        makeTransform(place_x, place_y, transfer_height, roll, pitch, yaw);

    Eigen::Matrix4d T_set_down =
        makeTransform(place_x, place_y, place_z + grasp_height, roll, pitch, yaw);

    printPose("T_pregrasp", T_pregrasp);
    printPose("T_grasp", T_grasp);
    printPose("T_lift", T_lift);
    printPose("T_transfer", T_transfer);
    printPose("T_set_down", T_set_down);

    // ============================================================
    // STEP 1: OPEN GRIPPER
    // ============================================================

    std::cout << "\nStep 1: Open gripper..." << std::endl;

    if (!gripper.move(open_width, gripper_speed_open)) {
      throw std::runtime_error("Gripper open failed.");
    }

    // ============================================================
    // STEP 2: PREGRASP
    // ============================================================

    std::cout << "\nStep 2: Move to pregrasp..." << std::endl;
    moveToPose(robot, T_pregrasp, 7.0, logger, 2, "move_to_pregrasp", experiment_time);

    // ============================================================
    // STEP 3: GRASP
    // ============================================================

    std::cout << "\nStep 3: Move down to grasp..." << std::endl;
    moveToPose(robot, T_grasp, 5.0, logger, 3, "descend_to_grasp", experiment_time);

    // ============================================================
    // STEP 4: CLOSE GRIPPER / GRASP BALL
    // ============================================================

    std::cout << "\nStep 4: Grasp ball..." << std::endl;

    bool grasp_success = gripper.grasp(
        grasp_width,
        grasp_speed,
        grasp_force,
        epsilon_inner,
        epsilon_outer);

    if (!grasp_success) {
      std::cerr << "WARNING: Gripper did not confirm successful grasp." << std::endl;
      std::cerr << "The program will NOT continue to lift/transfer." << std::endl;
      return 2;
    }

    std::cout << "Grasp successful." << std::endl;

    // Declare the ball payload before lifting.
    setObjectLoad(robot, object_mass_kg, F_x_Cload, load_inertia);

    // ============================================================
    // STEP 5: LIFT
    // ============================================================

    std::cout << "\nStep 5: Lift ball..." << std::endl;
    moveToPose(robot, T_lift, 4.0, logger, 5, "lift", experiment_time);

    // ============================================================
    // STEP 6: TRANSFER
    // ============================================================

    std::cout << "\nStep 6: Transfer ball..." << std::endl;
    moveToPose(robot, T_transfer, 7.0, logger, 6, "transfer", experiment_time);

    // ============================================================
    // STEP 7: SET DOWN
    // ============================================================

    std::cout << "\nStep 7: Set down ball..." << std::endl;
    moveToPose(robot, T_set_down, 5.0, logger, 7, "set_down", experiment_time);

    // ============================================================
    // STEP 8: OPEN GRIPPER
    // ============================================================

    std::cout << "\nStep 8: Open gripper / release ball..." << std::endl;

    if (!gripper.move(open_width, gripper_speed_open)) {
      std::cerr << "WARNING: Gripper open after set-down failed." << std::endl;
    }

    // Clear external load after releasing the ball.
    clearObjectLoad(robot);

    logger.writeCsv(output_csv);

    std::cout << "\nPick-place ball sequence completed." << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

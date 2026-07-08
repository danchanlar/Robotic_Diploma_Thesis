#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

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

// ------------------------------------------------------------
// Smooth Cartesian motion from current EE pose to target EE pose.
// Position: cosine smooth interpolation.
// Orientation: quaternion slerp.
// ------------------------------------------------------------
void moveToPose(franka::Robot& robot,
                const Eigen::Matrix4d& target_T,
                double duration_sec,
                double max_allowed_distance_m = 0.50) {
  franka::RobotState initial_state = robot.readOnce();

  Eigen::Matrix4d start_T = arrayToEigen(initial_state.O_T_EE);

  Eigen::Vector3d p0 = start_T.block<3, 1>(0, 3);
  Eigen::Vector3d p1 = target_T.block<3, 1>(0, 3);

  double distance = (p1 - p0).norm();

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

  robot.control(
      [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::CartesianPose {
        time += period.toSec();

        double s = std::min(time / duration_sec, 1.0);

        // Smoothstep 0 -> 1 with zero velocity at beginning/end.
        double alpha = 0.5 - 0.5 * std::cos(kPi * s);

        Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        Eigen::Quaterniond q = q0.slerp(alpha, q1);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = p;

        // Log external wrench about every 0.25 sec.
        // O_F_ext_hat_K = [Fx, Fy, Fz, Mx, My, Mz]
        if (time - last_log_time > 0.25) {
          last_log_time = time;
          const auto& F = robot_state.O_F_ext_hat_K;
          std::cout << std::fixed << std::setprecision(3)
                    << "  t=" << time
                    << "  F_ext=["
                    << F[0] << ", " << F[1] << ", " << F[2]
                    << "] N"
                    << "  M_ext=["
                    << F[3] << ", " << F[4] << ", " << F[5]
                    << "] Nm"
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

    std::cout << "Connecting to FR3 at IP: " << robot_ip << std::endl;

    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);

    // Optional recovery. If no recovery is needed, it may throw; that is okay.
    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    // ============================================================
    // EDIT THESE VALUES
    // ============================================================

    // Ball current position in robot base frame, in meters.
    const double pick_x = 0.45;
    const double pick_y = 0.00;
    const double pick_z = 0.08;

    // Ball target/place position in robot base frame, in meters.
    // Change this to your desired place location.
    const double place_x = 0.35;
    const double place_y = -0.25;
    const double place_z = 0.08;

    // Top-down gripper orientation.
    // EE/TCP z-axis points downward.
    const double roll = kPi;
    const double pitch = 0.0;
    const double yaw = 0.0;

    // Safe vertical offsets.
    // Keep these conservative at first.
    const double pregrasp_height = 0.14;   // 14 cm above ball reference z
    const double grasp_height    = 0.07;   // 7 cm above ball reference z, lower gradually if needed
    const double lift_height     = 0.25;   // absolute z while carrying
    const double transfer_height = 0.25;   // absolute z during transfer

    // Gripper settings.
    // Franka Hand max opening is about 0.08 m.
    const double open_width = 0.08;
    const double gripper_speed_open = 0.03;

    // For a ball, choose a final grasp width slightly smaller than the ball diameter.
    // Example:
    // ball diameter = 5 cm -> grasp_width around 0.035 to 0.045 m.
    const double grasp_width = 0.0032;
    const double grasp_speed = 0.02;
    const double grasp_force = 40.0;
    const double epsilon_inner = 0.006;
    const double epsilon_outer = 0.012;

    // Object load estimate.
    // Change this to the real ball mass.
    const double object_mass_kg = 0.10;

    // Approximate center of mass of the ball in flange frame F after grasp.
    // This depends on your gripper/TCP geometry.
    // Safe starting estimate: 10 cm along flange z direction.
    const std::array<double, 3> F_x_Cload = {
        0.0,
        0.0,
        0.10
    };

    // Approximate inertia of a small/light ball.
    // For a solid sphere: I = 2/5 * m * r^2.
    // For m=0.10 kg, r=0.025 m, I≈0.000025 kg*m^2.
    // We use a slightly conservative diagonal value.
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
    moveToPose(robot, T_pregrasp, 7.0);

    // ============================================================
    // STEP 3: GRASP
    // ============================================================

    std::cout << "\nStep 3: Move down to grasp..." << std::endl;
    moveToPose(robot, T_grasp, 5.0);

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
    moveToPose(robot, T_lift, 4.0);

    // ============================================================
    // STEP 6: TRANSFER
    // ============================================================

    std::cout << "\nStep 6: Transfer ball..." << std::endl;
    moveToPose(robot, T_transfer, 7.0);

    // ============================================================
    // STEP 7: SET DOWN
    // ============================================================

    std::cout << "\nStep 7: Set down ball..." << std::endl;
    moveToPose(robot, T_set_down, 5.0);

    // ============================================================
    // STEP 8: OPEN GRIPPER
    // ============================================================

    std::cout << "\nStep 8: Open gripper / release ball..." << std::endl;

    if (!gripper.move(open_width, gripper_speed_open)) {
      std::cerr << "WARNING: Gripper open after set-down failed." << std::endl;
    }

    // Clear external load after releasing the ball.
    clearObjectLoad(robot);

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

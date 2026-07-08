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

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::array<double, 16> eigenToArray(const Eigen::Matrix4d& T) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
  return out;
}

Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

double smootherStep(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);
}

// Side-grasp orientation for an upright bottle.
// EE z-axis points along base -x, EE x-axis points upward, EE y-axis points along base +y.
Eigen::Matrix4d makeSideBottleGraspTransform(double x, double y, double z) {
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  R.col(1) = Eigen::Vector3d(0.0, 1.0, 0.0);
  R.col(2) = Eigen::Vector3d(-1.0, 0.0, 0.0);

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

void moveToPose(franka::Robot& robot,
                const Eigen::Matrix4d& target_T,
                double duration_sec,
                double max_allowed_distance_m = 0.70) {
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
  std::cout << "Distance to target: " << distance << " m" << std::endl;

  if (distance > max_allowed_distance_m) {
    throw std::runtime_error("ABORT: target is too far from current EE pose.");
  }

  Eigen::Quaterniond q0(start_T.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(target_T.block<3, 3>(0, 0));
  q0.normalize();
  q1.normalize();
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
        double alpha = smootherStep(s);

        Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        Eigen::Quaterniond q = q0.slerp(alpha, q1);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = p;

        if (time - last_log_time > 0.50) {
          last_log_time = time;
          const auto& F = robot_state.O_F_ext_hat_K;
          std::cout << std::fixed << std::setprecision(4)
                    << "  t=" << time
                    << "  F_ext=[" << F[0] << ", " << F[1] << ", " << F[2] << "] N"
                    << "  M_ext=[" << F[3] << ", " << F[4] << ", " << F[5] << "] Nm"
                    << std::endl;
        }

        franka::CartesianPose output_pose(eigenToArray(T));
        if (s >= 1.0) {
          return franka::MotionFinished(output_pose);
        }
        return output_pose;
      },
      franka::ControllerMode::kCartesianImpedance,
      true);
}

void setBottleLoad(franka::Robot& robot,
                   double bottle_mass_kg,
                   const std::array<double, 3>& F_x_Cload,
                   const std::array<double, 9>& load_inertia) {
  std::cout << "Setting bottle load: " << bottle_mass_kg << " kg" << std::endl;
  robot.setLoad(bottle_mass_kg, F_x_Cload, load_inertia);
}

void clearObjectLoad(franka::Robot& robot) {
  std::cout << "Clearing external object load." << std::endl;
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

    try {
      std::cout << "Running automatic error recovery..." << std::endl;
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      std::cout << "automaticErrorRecovery skipped/failed: " << e.what() << std::endl;
    }

    // ============================================================
    // USER PARAMETERS
    // ============================================================

    const double bottle_x = 0.19;
    const double bottle_y = 0.33;
    const double bottle_table_z = 0.00;

    const double place_x = 0.30;
    const double place_y = -0.25;
    const double place_table_z = 0.00;

    const double bottle_mass_kg = 0.810;

    // Measure your real bottle and adjust these.
    const double bottle_diameter_m = 0.08;
    const double bottle_radius_m = bottle_diameter_m / 2.0;
    const double bottle_height_m = 0.24;

    // Side grasp height around the bottle body. This is EE/TCP z, not table z.
    const double grasp_center_z = bottle_table_z + 0.115;

    // Approach from +x side.
    const double side_pregrasp_offset_x = 0.120;

    const double lift_z = bottle_table_z + 0.280;
    const double transfer_z = bottle_table_z + 0.280;
    const double set_down_z = place_table_z + 0.115;

    // Gripper settings.
    const double open_width = 0.0041;
    const double gripper_speed_open = 0.03;
    const double grasp_width = 0.026;
    const double grasp_speed = 0.010;
    const double grasp_force = 50.0;
    const double epsilon_inner = 0.004;
    const double epsilon_outer = 0.018;
    const bool abort_if_grasp_fails = true;

    // Approximate payload center of mass in flange frame F.
    // Calibrate this for your actual gripper/TCP geometry.
    const std::array<double, 3> F_x_Cload = {0.0, 0.0, 0.10};

    // Approximate inertia for a cylindrical bottle.
    const double Ixx = (1.0 / 12.0) * bottle_mass_kg *
                       (3.0 * bottle_radius_m * bottle_radius_m +
                        bottle_height_m * bottle_height_m);
    const double Iyy = Ixx;
    const double Izz = 0.5 * bottle_mass_kg * bottle_radius_m * bottle_radius_m;

    const std::array<double, 9> load_inertia = {
        Ixx, 0.0, 0.0,
        0.0, Iyy, 0.0,
        0.0, 0.0, Izz};

    std::cout << "\nBottle task parameters:" << std::endl;
    std::cout << "  start reference = [" << bottle_x << ", " << bottle_y << ", " << bottle_table_z << "]" << std::endl;
    std::cout << "  target reference = [" << place_x << ", " << place_y << ", " << place_table_z << "]" << std::endl;
    std::cout << "  bottle mass = " << bottle_mass_kg << " kg" << std::endl;
    std::cout << "  bottle diameter estimate = " << bottle_diameter_m << " m" << std::endl;
    std::cout << "  grasp_width command = " << grasp_width << " m" << std::endl;
    std::cout << "  grasp_center_z = " << grasp_center_z << " m" << std::endl;
    std::cout << "  payload inertia diag = [" << Ixx << ", " << Iyy << ", " << Izz << "] kg*m^2" << std::endl;

    Eigen::Matrix4d T_pregrasp =
        makeSideBottleGraspTransform(bottle_x + side_pregrasp_offset_x,
                                     bottle_y,
                                     grasp_center_z);

    Eigen::Matrix4d T_grasp =
        makeSideBottleGraspTransform(bottle_x,
                                     bottle_y,
                                     grasp_center_z);

    Eigen::Matrix4d T_lift =
        makeSideBottleGraspTransform(bottle_x,
                                     bottle_y,
                                     lift_z);

    Eigen::Matrix4d T_transfer =
        makeSideBottleGraspTransform(place_x,
                                     place_y,
                                     transfer_z);

    Eigen::Matrix4d T_set_down =
        makeSideBottleGraspTransform(place_x,
                                     place_y,
                                     set_down_z);

    printPose("T_pregrasp", T_pregrasp);
    printPose("T_grasp", T_grasp);
    printPose("T_lift", T_lift);
    printPose("T_transfer", T_transfer);
    printPose("T_set_down", T_set_down);

    std::cout << "\nIMPORTANT SAFETY NOTES:" << std::endl;
    std::cout << "  1. The bottle is heavy: 0.810 kg." << std::endl;
    std::cout << "  2. Make sure the bottle diameter fits inside the gripper." << std::endl;
    std::cout << "  3. The side-grasp orientation may need adjustment for your real Franka Hand frame." << std::endl;
    std::cout << "  4. The payload COM/inertia are estimates and should be calibrated." << std::endl;
    std::cout << "  5. Keep one hand near the emergency stop." << std::endl;
    std::cout << "\nPress ENTER to start..." << std::endl;
    std::cin.ignore();

    std::cout << "\nStep 1: Open gripper..." << std::endl;
    if (!gripper.move(open_width, gripper_speed_open)) {
      throw std::runtime_error("Gripper open failed.");
    }

    std::cout << "\nStep 2: Move to side pregrasp..." << std::endl;
    moveToPose(robot, T_pregrasp, 8.0);

    std::cout << "\nStep 3: Move to bottle grasp pose..." << std::endl;
    moveToPose(robot, T_grasp, 6.0);

    std::cout << "\nStep 4: Grasp bottle..." << std::endl;
    bool grasp_success = gripper.grasp(
        grasp_width,
        grasp_speed,
        grasp_force,
        epsilon_inner,
        epsilon_outer);

    franka::GripperState gripper_state = gripper.readOnce();
    std::cout << "Gripper state after grasp:" << std::endl;
    std::cout << "  width = " << gripper_state.width << " m" << std::endl;
    std::cout << "  max_width = " << gripper_state.max_width << " m" << std::endl;
    std::cout << "  is_grasped = " << gripper_state.is_grasped << std::endl;

    if (!grasp_success) {
      std::cerr << "WARNING: Gripper did not confirm successful grasp." << std::endl;
      if (abort_if_grasp_fails) {
        std::cerr << "The program will NOT continue to lift/transfer a heavy bottle." << std::endl;
        return 2;
      }
    } else {
      std::cout << "Grasp successful." << std::endl;
    }

    setBottleLoad(robot, bottle_mass_kg, F_x_Cload, load_inertia);

    std::cout << "\nStep 5: Lift bottle..." << std::endl;
    moveToPose(robot, T_lift, 6.0);

    std::cout << "\nStep 6: Transfer bottle..." << std::endl;
    moveToPose(robot, T_transfer, 10.0);

    std::cout << "\nStep 7: Set down bottle..." << std::endl;
    moveToPose(robot, T_set_down, 6.0);

    std::cout << "\nStep 8: Open gripper / release bottle..." << std::endl;
    if (!gripper.move(open_width, gripper_speed_open)) {
      std::cerr << "WARNING: Gripper open after set-down failed." << std::endl;
    }

    clearObjectLoad(robot);
    std::cout << "\nBottle pick-and-place sequence completed." << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

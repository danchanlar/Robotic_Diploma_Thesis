#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/control_types.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Metatroph Eigen 4x4 se std::array gia libfranka.
// PROSOXH: To libfranka xrhsimopoiei column-major morfh.
std::array<double, 16> eigenToArray(const Eigen::Matrix4d& T) {
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
  return out;
}

// Metatroph std::array apo libfranka se Eigen 4x4.
// PROSOXH: To libfranka dinei O_T_EE se column-major morfh.
Eigen::Matrix4d arrayToEigen(const std::array<double, 16>& arr) {
  Eigen::Map<const Eigen::Matrix4d> T(arr.data());
  return T;
}

// Ftiaxnei omogenh metasxhmatismo apo x,y,z,roll,pitch,yaw.
// R = Rz(yaw) * Ry(pitch) * Rx(roll), opws sth ROS/URDF logikh.
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

// Omalo interpolation apo to trexon pose sto target pose.
// Xrhsimopoiei smoothstep gia th thesh kai slerp gia ton prosanatolismo.
void moveToPose(franka::Robot& robot,
                const Eigen::Matrix4d& target_T,
                double duration_sec,
                double max_allowed_distance_m = 0.40) {
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
        "ABORT: target is far from current EE pose. Move robot closer first.");
  }

  Eigen::Quaterniond q0(start_T.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(target_T.block<3, 3>(0, 0));

  q0.normalize();
  q1.normalize();

  // Gia na ginei h slerp apo th mikroterh diadromh.
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }

  double time = 0.0;

  robot.control(
      [&](const franka::RobotState&, franka::Duration period)
          -> franka::CartesianPose {
        time += period.toSec();

        double s = std::min(time / duration_sec, 1.0);

        // Smooth interpolation: 0 -> 1 xwris apoτοmes allages.
        double alpha = 0.5 - 0.5 * std::cos(kPi * s);

        Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        Eigen::Quaterniond q = q0.slerp(alpha, q1);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = p;

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

    // ============================================================
    // STOXOS ANTIKEIMENOU
    // ============================================================
    //
    // To antikeimeno einai sto:
    // x = 0.45 m
    // y = 0.00 m
    // z = 0.08 m
    //
    // Gia top-down grasp:
    // roll  = pi
    // pitch = 0
    // yaw   = 0
    //
    // Auto dinei rotation:
    // [ 1  0  0 ]
    // [ 0 -1  0 ]
    // [ 0  0 -1 ]
    //
    // Dhladh o z-axis tou end-effector koitaei pros ta katw.

    const double object_x = 0.45;
    const double object_y = 0.00;
    const double object_z = 0.08;

    const double roll = kPi;
    const double pitch = 0.0;
    const double yaw = 0.0;

    // Pre-grasp: 10 cm panw apo to antikeimeno.
    const double pre_grasp_z = object_z + 0.10;

    // Grasp: arxika stamata ligo panw apo to object_z.
    // An vlepeis oti den ftanei, kane to 0.09 h 0.08.
    const double grasp_z = object_z + 0.02;

    // Lift: shkwse to antikeimeno mexri 25 cm apo to base frame.
    const double lift_z = 0.25;

    Eigen::Matrix4d T_pre_grasp =
        makeTransform(object_x, object_y, pre_grasp_z, roll, pitch, yaw);

    Eigen::Matrix4d T_grasp =
        makeTransform(object_x, object_y, grasp_z, roll, pitch, yaw);

    Eigen::Matrix4d T_lift =
        makeTransform(object_x, object_y, lift_z, roll, pitch, yaw);

    std::cout << "\nT_pre_grasp:\n" << T_pre_grasp << "\n" << std::endl;
    std::cout << "T_grasp:\n" << T_grasp << "\n" << std::endl;
    std::cout << "T_lift:\n" << T_lift << "\n" << std::endl;

    // ============================================================
    // BHMA 1: OPEN GRIPPER
    // ============================================================

    std::cout << "\nStep 1: Opening gripper..." << std::endl;

    // Franka Hand max opening einai peripou 0.08 m.
    // 0.08 = 8 cm.
    const double open_width = 0.08;
    const double gripper_speed = 0.03;

    if (!gripper.move(open_width, gripper_speed)) {
      throw std::runtime_error("Gripper open failed.");
    }

    // ============================================================
    // BHMA 2: MOVE TO PRE-GRASP
    // ============================================================

    std::cout << "\nStep 2: Moving to pre-grasp..." << std::endl;

    moveToPose(robot, T_pre_grasp, 5.0);

    // ============================================================
    // BHMA 3: MOVE DOWN TO GRASP
    // ============================================================

    std::cout << "\nStep 3: Moving down to grasp..." << std::endl;

    moveToPose(robot, T_grasp, 3.0);

    // ============================================================
    // BHMA 4: CLOSE GRIPPER
    // ============================================================

    std::cout << "\nStep 4: Closing gripper / grasping object..." << std::endl;

    // ALLAKSE AUTO ANALOGA ME TO PLATOS TOU ANTIKEIMENOU.
    //
    // Paradeigma:
    // - object_width = 0.04 shmainei oti to antikeimeno exei platos 4 cm.
    // - To gripper tha prospathei na kleisei mexri peripou 4 cm.
    //
    // Gia mikro antikeimeno vale p.x. 0.02.
    // Gia megalutero vale p.x. 0.05.
    const double object_width = 0.04;

    const double grasp_speed = 0.02;
    const double grasp_force = 20.0;

    const double epsilon_inner = 0.005;
    const double epsilon_outer = 0.005;

    bool grasp_success = gripper.grasp(
        object_width,
        grasp_speed,
        grasp_force,
        epsilon_inner,
        epsilon_outer);

    if (!grasp_success) {
      std::cerr << "WARNING: Gripper did not confirm successful grasp."
                << std::endl;
    } else {
      std::cout << "Grasp successful." << std::endl;
    }

    // ============================================================
    // BHMA 5: LIFT OBJECT
    // ============================================================

    std::cout << "\nStep 5: Lifting object..." << std::endl;

    moveToPose(robot, T_lift, 3.0);

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

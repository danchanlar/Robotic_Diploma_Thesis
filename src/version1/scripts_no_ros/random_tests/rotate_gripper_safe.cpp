#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>

struct Quat {
  double w;
  double x;
  double y;
  double z;
};

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

Quat normalize(Quat q) {
  double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Quat multiply(const Quat& a, const Quat& b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Quat fromAxisAngle(double ax, double ay, double az, double angle) {
  double half = angle / 2.0;
  double s = std::sin(half);
  return normalize({std::cos(half), ax * s, ay * s, az * s});
}

Quat slerp(Quat q0, Quat q1, double t) {
  q0 = normalize(q0);
  q1 = normalize(q1);

  double dot = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;

  if (dot < 0.0) {
    q1 = {-q1.w, -q1.x, -q1.y, -q1.z};
    dot = -dot;
  }

  if (dot > 0.9995) {
    return normalize({
      q0.w + t * (q1.w - q0.w),
      q0.x + t * (q1.x - q0.x),
      q0.y + t * (q1.y - q0.y),
      q0.z + t * (q1.z - q0.z)
    });
  }

  double theta_0 = std::acos(dot);
  double theta = theta_0 * t;

  double sin_theta = std::sin(theta);
  double sin_theta_0 = std::sin(theta_0);

  double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  double s1 = sin_theta / sin_theta_0;

  return {
    s0*q0.w + s1*q1.w,
    s0*q0.x + s1*q1.x,
    s0*q0.y + s1*q1.y,
    s0*q0.z + s1*q1.z
  };
}

// Franka O_T_EE is column-major.
// R(row, col) = pose[col * 4 + row]
Quat poseRotationToQuat(const std::array<double, 16>& pose) {
  double r00 = pose[0];
  double r01 = pose[4];
  double r02 = pose[8];

  double r10 = pose[1];
  double r11 = pose[5];
  double r12 = pose[9];

  double r20 = pose[2];
  double r21 = pose[6];
  double r22 = pose[10];

  double trace = r00 + r11 + r22;
  Quat q{};

  if (trace > 0.0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    q.w = 0.25 / s;
    q.x = (r21 - r12) * s;
    q.y = (r02 - r20) * s;
    q.z = (r10 - r01) * s;
  } else if (r00 > r11 && r00 > r22) {
    double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    q.w = (r21 - r12) / s;
    q.x = 0.25 * s;
    q.y = (r01 + r10) / s;
    q.z = (r02 + r20) / s;
  } else if (r11 > r22) {
    double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    q.w = (r02 - r20) / s;
    q.x = (r01 + r10) / s;
    q.y = 0.25 * s;
    q.z = (r12 + r21) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    q.w = (r10 - r01) / s;
    q.x = (r02 + r20) / s;
    q.y = (r12 + r21) / s;
    q.z = 0.25 * s;
  }

  return normalize(q);
}

void setPoseRotationFromQuat(std::array<double, 16>& pose, Quat q) {
  q = normalize(q);

  double ww = q.w*q.w;
  double xx = q.x*q.x;
  double yy = q.y*q.y;
  double zz = q.z*q.z;

  double wx = q.w*q.x;
  double wy = q.w*q.y;
  double wz = q.w*q.z;
  double xy = q.x*q.y;
  double xz = q.x*q.z;
  double yz = q.y*q.z;

  double r00 = ww + xx - yy - zz;
  double r01 = 2.0 * (xy - wz);
  double r02 = 2.0 * (xz + wy);

  double r10 = 2.0 * (xy + wz);
  double r11 = ww - xx + yy - zz;
  double r12 = 2.0 * (yz - wx);

  double r20 = 2.0 * (xz - wy);
  double r21 = 2.0 * (yz + wx);
  double r22 = ww - xx - yy + zz;

  pose[0] = r00;
  pose[4] = r01;
  pose[8] = r02;

  pose[1] = r10;
  pose[5] = r11;
  pose[9] = r12;

  pose[2] = r20;
  pose[6] = r21;
  pose[10] = r22;
}

int main(int argc, char** argv) {
  const char* robot_ip = "172.16.0.2";

  // Default: στρίβει τον gripper γύρω από τον τοπικό Z άξονα.
  // Αυτό συνήθως φαίνεται σαν περιστροφή του καρπού/fingers.
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 25.0;

  if (argc >= 2) robot_ip = argv[1];
  if (argc >= 5) {
    roll_deg = std::stod(argv[2]);
    pitch_deg = std::stod(argv[3]);
    yaw_deg = std::stod(argv[4]);
  }

  if (std::abs(roll_deg) > 45.0 || std::abs(pitch_deg) > 45.0 || std::abs(yaw_deg) > 45.0) {
    std::cerr << "ABORT: For safety, each angle must be within [-45, 45] degrees.\n";
    return 1;
  }

  try {
    franka::Robot robot(robot_ip);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::RobotState initial_state = robot.readOnce();

    std::array<double, 16> start_pose = initial_state.O_T_EE;
    std::array<double, 16> goal_pose = start_pose;

    Quat q_start = poseRotationToQuat(start_pose);

    Quat q_roll = fromAxisAngle(1.0, 0.0, 0.0, deg2rad(roll_deg));
    Quat q_pitch = fromAxisAngle(0.0, 1.0, 0.0, deg2rad(pitch_deg));
    Quat q_yaw = fromAxisAngle(0.0, 0.0, 1.0, deg2rad(yaw_deg));

    // Local rotation: q_goal = q_start * q_delta
    Quat q_delta = multiply(multiply(q_roll, q_pitch), q_yaw);
    Quat q_goal = multiply(q_start, q_delta);
    q_goal = normalize(q_goal);

    setPoseRotationFromQuat(goal_pose, q_goal);

    const double duration = 8.0;
    double time = 0.0;

    std::cout << "Current position is kept fixed:\n";
    std::cout << "  x=" << start_pose[12]
              << " y=" << start_pose[13]
              << " z=" << start_pose[14] << "\n";

    std::cout << "Changing gripper orientation only.\n";
    std::cout << "Relative local rotation:\n";
    std::cout << "  roll  = " << roll_deg << " deg\n";
    std::cout << "  pitch = " << pitch_deg << " deg\n";
    std::cout << "  yaw   = " << yaw_deg << " deg\n";
    std::cout << "Duration: " << duration << " sec\n";
    std::cout << "Keep emergency stop ready.\n";

    robot.control([&](const franka::RobotState&,
                      franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      double t = std::min(time / duration, 1.0);
      double s = 3.0 * t * t - 2.0 * t * t * t;

      Quat q_cmd = slerp(q_start, q_goal, s);

      std::array<double, 16> commanded_pose = start_pose;
      setPoseRotationFromQuat(commanded_pose, q_cmd);

      // Keep exact same translation.
      commanded_pose[12] = start_pose[12];
      commanded_pose[13] = start_pose[13];
      commanded_pose[14] = start_pose[14];

      if (time >= duration) {
        std::cout << "Orientation motion finished.\n";
        return franka::MotionFinished(franka::CartesianPose(goal_pose));
      }

      return franka::CartesianPose(commanded_pose);
    });

    std::cout << "Done.\n";
    return 0;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception: " << e.what() << "\n";
    return 1;
  }
}

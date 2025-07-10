/// @file robot_types.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2022-09-25
/// @copyright Copyright (c) 2023
 


#ifndef ROBOT_TYPES_H_
#define ROBOT_TYPES_H_

#include <stdint.h>
#include <array>

/// @brief A struct used to store the data collected from an IMU sensor.
struct ImuData {
  int32_t timestamp; // Timestamp of the data in milliseconds.
  union {
    float buffer_float[9]; // An array containing 9 float elements.
    uint8_t buffer_byte[3][12]; // A 2D array with 3 rows and 12 columns of uint8_t elements used to store raw data.
    struct {
      float angle_roll; // Roll angle (unit: degrees).
      float angle_pitch; // Pitch angle (unit: degrees).
      float angle_yaw; // Yaw angle (unit: degrees).

      float angular_velocity_roll; // Roll angular velocity (unit: degrees/second).
      float angular_velocity_pitch; // Pitch angular velocity (unit: degrees/second).
      float angular_velocity_yaw; // Yaw angular velocity (unit: degrees/second).

      float acc_x; // X-axis acceleration (unit: m/s^2).
      float acc_y; // Y-axis acceleration (unit: m/s^2).
      float acc_z; // Z-axis acceleration (unit: m/s^2).
    }; // An anonymous structure used to access the buffer member as float.
  }; // An anonymous union used to provide different ways of accessing the data.
};

/// @brief Struct containing data related to a motor.
typedef struct {
  float position; // Motor position (unit: radians).
  float velocity; // Motor velocity (unit: radians/second).
  float torque; // Motor torque (unit: Nm). */
  float temperature; // Motor temperature (unit: Celsius).
} JointData;

/// @brief Struct containing data related to a robot.
typedef struct {
  union {
    JointData joint_data[12]; // Joint data for all 12 motors.
    struct {
      JointData fl_leg[3]; // Joint data for the front left leg motors.
      JointData fr_leg[3]; // Joint data for the front right leg motors.
      JointData hl_leg[3]; // Joint data for the hind left leg motors.
      JointData hr_leg[3]; // Joint data for the hind right leg motors.
    };
  };
} LegData;

/// @brief Struct containing command data for a joint.
typedef struct {
  float position; // Desired joint position (unit: radians).
  float velocity; // Desired joint velocity (unit: radians/second).
  float torque; // Desired joint torque (unit: Nm).
  float kp; // Proportional gain of joint controller.
  float kd; // Derivative gain of joint controller.
} JointCmd;

/// @brief Struct containing command data for a robot.
typedef struct {
  union {
    JointCmd joint_cmd[12];  // Joint commands for all 12 joints.
    struct {
      JointCmd fl_leg[3];   // Joint commands for the front left leg joints.
      JointCmd fr_leg[3];   // Joint commands for the front right leg joints.
      JointCmd hl_leg[3];   // Joint commands for the hind left leg joints.
      JointCmd hr_leg[3];   // Joint commands for the hind right leg joints.
    };
  };
} RobotCmd;

/// @brief Struct containing contact force data for a robot.
/// @description The `leg_force` array contains the forces for all 12 motors, while the
/// individual leg force arrays (`fl_leg`, `fr_leg`, `hl_leg`, `hr_leg`) contain the forces for
/// the specific legs of the robot. The forces are given in x, y, and z direction.
///
/// @note In most cases, only the z-direction force is used.
typedef struct {
  union {
    double leg_force[12]; // Array of leg forces for all 12 motors.
    struct {
      double fl_leg[3]; // Forces for the front left leg motors (in x, y, and z direction).
      double fr_leg[3]; // Forces for the front right leg motors (in x, y, and z direction).
      double hl_leg[3]; // Forces for the hind left leg motors (in x, y, and z direction).
      double hr_leg[3]; // Forces for the hind right leg motors (in x, y, and z direction).
    };
  };
} ContactForce;


/// @brief 
/// Struct containing data for a robot.
/// The tick field is a timestamp for the data. The imu, joint_data, and contact_force
/// fields contain the IMU data, joint data, and contact force data for the robot, respectively.
typedef struct{
  uint32_t tick;              // Timestamp for the data.
  ImuData imu;                // IMU data for the robot.
  LegData joint_data;         // Joint data for the robot.
  ContactForce contact_force; // Contact force data for the robot.
}RobotData;

#endif  ///< ROBOT_TYPES_H_
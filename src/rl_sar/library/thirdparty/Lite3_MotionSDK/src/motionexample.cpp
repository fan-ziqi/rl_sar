/// @file motionexample.cpp
/// @author ysc (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2023-03-03 
/// @copyright Copyright (c) 2023

#include "motionexample.h"

Vec3 goal_angle_fl, goal_angle_hl, goal_angle_fr, goal_angle_hr;
Vec3 init_angle_fl, init_angle_fr, init_angle_hl, init_angle_hr;
double init_time;

/// @brief Spend 1 s putting the robot's legs away and preparing to stand
/// @param cmd Issue control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
void MotionExample::PreStandUp(RobotCmd &cmd, double time,RobotData &data_state) {
  double standup_time = 1.0;
  double cycle_time = 0.001;
  goal_angle_fl << 0 * kDegree2Radian, -70 * kDegree2Radian,
    150 * kDegree2Radian;
  goal_angle_fr << 0 * kDegree2Radian, -70 * kDegree2Radian,
    150 * kDegree2Radian;
  goal_angle_hl << 0 * kDegree2Radian, -70 * kDegree2Radian,
    150 * kDegree2Radian;
  goal_angle_hr << 0 * kDegree2Radian, -70 * kDegree2Radian,
    150 * kDegree2Radian;

  if (time <= init_time + standup_time) {
    SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - init_time,
                 cycle_time, "FL", cmd, data_state);
    SwingToAngle(init_angle_fr, goal_angle_fr, standup_time, time - init_time,
                 cycle_time, "FR", cmd,data_state);
    SwingToAngle(init_angle_hl, goal_angle_hl, standup_time, time - init_time,
                 cycle_time, "HL", cmd,data_state);
    SwingToAngle(init_angle_hr, goal_angle_hr, standup_time, time - init_time,
                 cycle_time, "HR", cmd,data_state);
  }
}

/// @brief Spend 1.5s standing
/// @param cmd Issue control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
void MotionExample::StandUp(RobotCmd &cmd, double time,RobotData &data_state) {
  double standup_time = 1.5;
  double cycle_time = 0.001;
    goal_angle_fl << 0 * kDegree2Radian, -42 * kDegree2Radian,
      78 * kDegree2Radian;
  goal_angle_fr << 0 * kDegree2Radian, -42 * kDegree2Radian,
      78 * kDegree2Radian;
  goal_angle_hl << 0 * kDegree2Radian, -42 * kDegree2Radian,
      78 * kDegree2Radian;
  goal_angle_hr << 0 * kDegree2Radian, -42 * kDegree2Radian,
      78 * kDegree2Radian;

  if (time <= init_time + standup_time) {
    SwingToAngle(init_angle_fl, goal_angle_fl, standup_time, time - init_time,
                 cycle_time, "FL", cmd, data_state);
    SwingToAngle(init_angle_fr, goal_angle_fr, standup_time, time - init_time,
                 cycle_time, "FR", cmd,data_state);
    SwingToAngle(init_angle_hl, goal_angle_hl, standup_time, time - init_time,
                 cycle_time, "HL", cmd,data_state);
    SwingToAngle(init_angle_hr, goal_angle_hr, standup_time, time - init_time,
                 cycle_time, "HR", cmd,data_state);
  }else{
    for (int i = 0; i < 12; i++) {
      cmd.joint_cmd[i].torque = 0;
      cmd.joint_cmd[i].kp = 80;
      cmd.joint_cmd[i].kd = 0.7;
    }
    for (int i = 0; i < 4; i++) {
      cmd.joint_cmd[3*i].position = 0;
      cmd.joint_cmd[3*i+1].position = -42 * kDegree2Radian;
      cmd.joint_cmd[3*i+2].position = 78 * kDegree2Radian;
      cmd.joint_cmd[3*i].velocity = 0;
      cmd.joint_cmd[3*i+1].velocity = 0;
      cmd.joint_cmd[3*i+2].velocity = 0;
    }
  }
}

/// @brief Only the current moment and angle are recorded
/// @param data Current joint data
/// @param time Current timestamp
void MotionExample::GetInitData(LegData data, double time) {
  init_time = time;
  // Only the current moment and angle are recorded
  init_angle_fl[0] = data.fl_leg[0].position;
  init_angle_fl[1] = data.fl_leg[1].position;
  init_angle_fl[2] = data.fl_leg[2].position;

  init_angle_fr[0] = data.fr_leg[0].position;
  init_angle_fr[1] = data.fr_leg[1].position;
  init_angle_fr[2] = data.fr_leg[2].position;

  init_angle_hl[0] = data.hl_leg[0].position;
  init_angle_hl[1] = data.hl_leg[1].position;
  init_angle_hl[2] = data.hl_leg[2].position;

  init_angle_hr[0] = data.hr_leg[0].position;
  init_angle_hr[1] = data.hr_leg[1].position;
  init_angle_hr[2] = data.hr_leg[2].position;
}

/// @brief Specifically achieve swinging one leg of the robot to a specified position within a specified time
/// @param initial_angle 
/// @param final_angle
/// @param total_time
/// @param run_time
/// @param cycle_time Control cycle, default is 1ms
/// @param side Control which leg, FL is the left front leg, FR is the right front leg, HL is the left and right leg, and HR is the right rear leg
/// @param cmd Issue control command
/// @param data Real-time status data of robot
void MotionExample::SwingToAngle(Vec3 initial_angle, Vec3 final_angle,
                                 double total_time, double run_time,
                                 double cycle_time, string side,
                                 RobotCmd &cmd,
                                 RobotData &data) {
  Vec3 goal_angle;
  Vec3 goal_angle_next;
  Vec3 goal_angle_next2;
  Vec3 goal_velocity;
  int leg_side;

  if (side == "FL")
    leg_side = 0;
  else if (side == "FR")
    leg_side = 1;
  else if (side == "HL")
    leg_side = 2;
  else if (side == "HR")
    leg_side = 3;
  else
    cout << "Leg Side Error!!!" << endl;

  final_angle = final_angle;
  for (int j = 0; j < 3; j++) {
    CubicSpline(initial_angle[j], 0, (double)final_angle[j], 0, run_time,
                cycle_time, total_time, goal_angle[j], goal_angle_next[j],
                goal_angle_next2[j]);
  }

  goal_velocity = (goal_angle_next - goal_angle) / cycle_time;

  ///< The following two methods can be used to complete joint control. By default, position control mode is used, and hybrid control mode is supported

  ///< Joint pd control - position control method, namely, issuing kp, kd, goal_ angle，goal_ Vel, feedforward force tor=0, joint end completes joint pd control closed-loop
  if(true){
    cmd.joint_cmd[3 * leg_side].kp = 60;
    cmd.joint_cmd[3 * leg_side + 1].kp = 60;
    cmd.joint_cmd[3 * leg_side + 2].kp = 60;
    cmd.joint_cmd[3 * leg_side].kd = 0.7;
    cmd.joint_cmd[3 * leg_side + 1].kd = 0.7;
    cmd.joint_cmd[3 * leg_side + 2].kd = 0.7;
    cmd.joint_cmd[3 * leg_side].position = goal_angle[0];
    cmd.joint_cmd[3 * leg_side + 1].position = goal_angle[1];
    cmd.joint_cmd[3 * leg_side + 2].position = goal_angle[2];
    cmd.joint_cmd[3 * leg_side].velocity = goal_velocity[0];
    cmd.joint_cmd[3 * leg_side + 1].velocity = goal_velocity[1];
    cmd.joint_cmd[3 * leg_side + 2].velocity = goal_velocity[2];
    for (int i = 0; i < 12; i++) {
      cmd.joint_cmd[i].torque = 0;
    }
  } else{
    ///< Joint pd control - force control method, i.e. kp, kd, goal_ angle，goal_ Set level to 0
    ///< The upper layer obtains real-time joint data and compares it with the target, performs pd control, calculates the expected torque of each joint, and sends it as a feedforward force tor to the joint end, which directly executes the target torque
    cmd.joint_cmd[3 * leg_side].kp = 0;
    cmd.joint_cmd[3 * leg_side + 1].kp = 0;
    cmd.joint_cmd[3 * leg_side + 2].kp = 0;
    cmd.joint_cmd[3 * leg_side].kd = 0;
    cmd.joint_cmd[3 * leg_side + 1].kd = 0;
    cmd.joint_cmd[3 * leg_side + 2].kd = 0;
    cmd.joint_cmd[3 * leg_side].position = 0;
    cmd.joint_cmd[3 * leg_side + 1].position = 0;
    cmd.joint_cmd[3 * leg_side + 2].position = 0;
    cmd.joint_cmd[3 * leg_side].velocity = 0;
    cmd.joint_cmd[3 * leg_side + 1].velocity = 0;
    cmd.joint_cmd[3 * leg_side + 2].velocity = 0;
    cmd.joint_cmd[3* leg_side].torque = 60 * (goal_angle[0] - data.joint_data.joint_data[3* leg_side].position)
                                    + 0.7 * (goal_velocity[0] - data.joint_data.joint_data[3* leg_side].velocity);
    cmd.joint_cmd[3* leg_side+1].torque = 80 * (goal_angle[1] - data.joint_data.joint_data[3* leg_side+1].position)
                                    + 0.7 * (goal_velocity[1] - data.joint_data.joint_data[3* leg_side+1].velocity);
    cmd.joint_cmd[3* leg_side+2].torque = 80 * (goal_angle[2] - data.joint_data.joint_data[3* leg_side+2].position)
                                    + 0.7 * (goal_velocity[2] - data.joint_data.joint_data[3* leg_side+2].velocity);
  }
}

/// @brief Interpolation to find the path point, i.e. the target angle for each control cycle
/// @param init_position 
/// @param init_velocity 
/// @param goal_position 
/// @param goal_velocity 
/// @param run_time 
/// @param cycle_time Control cycle, default is 1ms
/// @param total_time 
/// @param sub_goal_position Target angle for the control cycle
/// @param sub_goal_position_next Target angle for the next control cycle
/// @param sub_goal_position_next2 Target angle for the next and next control cycle
void MotionExample::CubicSpline(double init_position, double init_velocity,
                                double goal_position, double goal_velocity,
                                double run_time, double cycle_time,
                                double total_time, double &sub_goal_position,
                                double &sub_goal_position_next,
                                double &sub_goal_position_next2) {
  double a, b, c, d;
  d = init_position;
  c = init_velocity;
  a = (goal_velocity * total_time - 2 * goal_position + init_velocity * total_time +
       2 * init_position) /
      pow(total_time, 3);
  b = (3 * goal_position - goal_velocity * total_time - 2 * init_velocity * total_time -
       3 * init_position) /
      pow(total_time, 2);

  if (run_time > total_time)
    run_time = total_time;
  sub_goal_position = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d;

  if (run_time + cycle_time > total_time)
    run_time = total_time - cycle_time;
  sub_goal_position_next = a * pow(run_time + cycle_time, 3) +
                      b * pow(run_time + cycle_time, 2) +
                      c * (run_time + cycle_time) + d;

  if (run_time + cycle_time * 2 > total_time)
    run_time = total_time - cycle_time * 2;
  sub_goal_position_next2 = a * pow(run_time + cycle_time * 2, 3) +
                       b * pow(run_time + cycle_time * 2, 2) +
                       c * (run_time + cycle_time * 2) + d;
}
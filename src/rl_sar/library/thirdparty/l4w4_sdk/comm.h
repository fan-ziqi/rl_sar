/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef L4W4_SDK_COMM_H
#define L4W4_SDK_COMM_H

#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

typedef struct
{
    float quaternion[4];               // quaternion, normalized, (w,x,y,z)
    float gyroscope[3];                // angular velocity （unit: rad/s)
    float accelerometer[3];            // m/(s2)
    float rpy[3];                      // euler angle（unit: rad)
    int8_t temperature;
} IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

typedef struct
{
    uint8_t mode;                      // motor working mode
    float q;                           // current angle (unit: radian)
    float dq;                          // current velocity (unit: radian/second)
    float ddq;                         // current acc (unit: radian/second*second)
    float tauEst;                      // current estimated output torque (unit: N.m)
    float q_raw;                       // current angle (unit: radian)
    float dq_raw;                      // current velocity (unit: radian/second)
    float ddq_raw;
    int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
    uint32_t reserve[2];
} MotorState;                          // motor feedback

typedef struct
{
    uint8_t mode;                      // desired working mode
    float q;                           // desired angle (unit: radian)
    float dq;                          // desired velocity (unit: radian/second)
    float tau;                         // desired output torque (unit: N.m)
    float Kp;                          // desired position stiffness (unit: N.m/rad )
    float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
    uint32_t reserve[3];
} MotorCmd;                            // motor control

typedef struct
{
    IMU imu;
    MotorState motorState[20];
    uint8_t wirelessRemote[40];        // wireless commands
} LowState;                            // low level feedback

typedef struct
{
    MotorCmd motorCmd[20];
    uint8_t wirelessRemote[40];
} LowCmd;                              // low level control

#endif  // L4W4_SDK_COMM_H

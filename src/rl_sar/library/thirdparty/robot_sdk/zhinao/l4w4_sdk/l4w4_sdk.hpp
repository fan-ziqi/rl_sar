/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef L4W4_SDK_HPP
#define L4W4_SDK_HPP

#include <iostream>

#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include "joystick.h"
#include "comm.h"

class L4W4SDK
{
private:
    float tmp_time_from_mcu = -1;
    int show_rc = 0;
    int ctr = 0;
    float t0 = -1;
    int n_cpu = 0;
    int n_run = 0;

    float read_float(unsigned char *buff, int *idx);
    float read_from1byte(unsigned char *buff, int *idx, float v_min, float v_max);
    float read_from2bytes(unsigned char *buff, int *idx, float v_min, float v_max);
    void write_into2bytes(float value, unsigned char *buff, int *idx, float v_min, float v_max);
    float read_byte(unsigned char *buff, int *idx);
    float Vector3_Dot(Vector3 v1, Vector3 v2);
    Vector3 Vector3_Cross(Vector3 u, Vector3 v);
    float Sign(float value);
    float Vector3_invSqrt(Vector3 v);
    Vector3 Vector3_Direction(Vector3 v);
    float Clamp(float value, float min, float max);
    float Angle_vA_2_vB(Vector3 vA, Vector3 vB, Vector3 axle);
    void Vector3_Normalize(Vector3 *v);
    Vector3 Quaternion_Transform(float vx, float vy, float vz, float qx, float qy, float qz, float qw);

public:
    L4W4SDK() {};
    ~L4W4SDK() {};
    int client_socket;
    struct sockaddr_in server_addr;
    int recv_len = 0;
    int ex_send_recv = -1;
    unsigned char sent_buff[256];
    unsigned char recv_buff[512];
    void InitUDP();
    void AnalyzeUDP(unsigned char *recv_buff, LowState &lowState);
    void SendUDP(LowCmd &lowCmd);
    void PrintMCU(int running_state);
    void InitCmdData(LowCmd &cmd);
};

void L4W4SDK::InitCmdData(LowCmd &cmd)
{
    for (int i = 0; i < 20; ++i)
    {
        cmd.motorCmd[i].mode = 0;
        cmd.motorCmd[i].q = 0.0;
        cmd.motorCmd[i].dq = 0.0;
        cmd.motorCmd[i].tau = 0.0;
        cmd.motorCmd[i].Kp = 0.0;
        cmd.motorCmd[i].Kd = 0.0;
    }
    memset(cmd.wirelessRemote, 0, sizeof(cmd.wirelessRemote));
}

void L4W4SDK::PrintMCU(int running_state)
{
    float t1 = tmp_time_from_mcu;
    if (t1 < t0)
        t0 = t1;

    float ms = (t1 - t0) * 1000.0;
    n_cpu++;
    n_run++;

    if (ms > 1000)
    {
        std::cout << "mcu time = " << tmp_time_from_mcu << ",   n_cpu = " << n_cpu << ", runningState= " << running_state << std::endl;
        t0 = t1;
        n_cpu = 0;
        n_run = 0;
    }
    else
    {
        if (n_run > 50)
        {
            std::cout << "  dull ms= " << ms << ",  n_cpu = " << n_cpu << ", runningState= " << running_state << std::endl;
            n_run = 0;
        }
    }
}

void L4W4SDK::SendUDP(LowCmd &lowCmd)
{
    int idx = 0;

    // front right
    write_into2bytes(lowCmd.motorCmd[4].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[4].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[4].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[5].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[5].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[5].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[6].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[6].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[6].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[7].dq, sent_buff, &idx, -200, 200);
    write_into2bytes(lowCmd.motorCmd[7].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[7].Kd, sent_buff, &idx, 0, 1000);

    // rear right
    write_into2bytes(lowCmd.motorCmd[12].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[12].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[12].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[13].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[13].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[13].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[14].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[14].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[14].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[15].dq, sent_buff, &idx, -200, 200);
    write_into2bytes(lowCmd.motorCmd[15].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[15].Kd, sent_buff, &idx, 0, 1000);

    // rear left
    write_into2bytes(lowCmd.motorCmd[8].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[8].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[8].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[9].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[9].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[9].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[10].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[10].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[10].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[11].dq, sent_buff, &idx, -200, 200);
    write_into2bytes(lowCmd.motorCmd[11].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[11].Kd, sent_buff, &idx, 0, 1000);

    // front left
    write_into2bytes(lowCmd.motorCmd[0].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[0].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[0].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[1].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[1].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[1].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[2].q, sent_buff, &idx, -3.2, 3.2);
    write_into2bytes(lowCmd.motorCmd[2].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[2].Kd, sent_buff, &idx, 0, 1000);
    write_into2bytes(-lowCmd.motorCmd[3].dq, sent_buff, &idx, -200, 200);
    write_into2bytes(lowCmd.motorCmd[3].Kp, sent_buff, &idx, 0, 1000);
    write_into2bytes(lowCmd.motorCmd[3].Kd, sent_buff, &idx, 0, 1000);

    write_into2bytes(0.0f, sent_buff, &idx, -10, 10);
    write_into2bytes(0.0f, sent_buff, &idx, -10, 10);
    write_into2bytes(0.0f, sent_buff, &idx, -10, 10);

    int sss = sendto(client_socket, sent_buff, idx, 0, (const sockaddr *)&server_addr, sizeof(server_addr));
}

void L4W4SDK::InitUDP()
{
    struct sockaddr_in client_addr, actual_addr;
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = INADDR_ANY; // INADDR_ANY表示自动获取本机地址       inet_addr("192.168.1.102");
    client_addr.sin_port = htons(0);          // 0表示让系统自动分配一个空闲端口
    client_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (client_socket < 0)
    {
        std::cout << "socket error" << std::endl;
        perror("socket error");
        exit(1);
    }
    else
    {
        std::cout << "socket created (" << client_socket << ")" << std::endl;
    }

    int ret = bind(client_socket, (struct sockaddr *)&client_addr, sizeof(client_addr));
    if (ret >= 0)
    {
        socklen_t addr_len = sizeof(actual_addr);
        getsockname(client_socket, (struct sockaddr *)&actual_addr, &addr_len);
        std::cout << "udp bind success (" << ret << ")" << std::endl;
        std::cout << "my ip = " << inet_ntoa(actual_addr.sin_addr) << std::endl;
        std::cout << "my port = " << actual_addr.sin_port << std::endl;
    }
    else
        std::cout << "udp bind failed (" << ret << ")" << std::endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(777);
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.101");
}

float L4W4SDK::read_float(unsigned char *buff, int *idx)
{
    float v = 0;
    *((unsigned char *)&v + 0) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 1) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 2) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 3) = buff[*idx];
    (*idx)++;

    return v;
}

float L4W4SDK::read_from1byte(unsigned char *buff, int *idx, float v_min, float v_max)
{
    float v = (float)buff[*idx] / 255.0 * (v_max - v_min) + v_min;
    (*idx)++;
    return v;
}

float L4W4SDK::read_from2bytes(unsigned char *buff, int *idx, float v_min, float v_max)
{
    unsigned int v = 0;
    *((unsigned char *)&v + 0) = buff[*idx];
    (*idx)++;
    *((unsigned char *)&v + 1) = buff[*idx];
    (*idx)++;

    return v / 65535.0 * (v_max - v_min) + v_min;
}

void L4W4SDK::write_into2bytes(float value, unsigned char *buff, int *idx, float v_min, float v_max)
{
    ushort v16 = (value - v_min) / (v_max - v_min) * 65535;
    buff[*idx] = (v16 >> 8) & 0xff;
    (*idx)++;
    buff[*idx] = v16 & 0xff;
    (*idx)++;
}

float L4W4SDK::read_byte(unsigned char *buff, int *idx)
{
    float v = (float)buff[*idx];
    (*idx)++;
    return v;
}

float L4W4SDK::Vector3_Dot(Vector3 v1, Vector3 v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 L4W4SDK::Vector3_Cross(Vector3 u, Vector3 v)
{
    Vector3 vn;
    vn.x = u.y * v.z - u.z * v.y;
    vn.y = u.z * v.x - u.x * v.z;
    vn.z = u.x * v.y - u.y * v.x;
    return vn;
}

float L4W4SDK::Sign(float value)
{
    if (value >= 0)
        return 1;
    else
        return -1;
}

float L4W4SDK::Vector3_invSqrt(Vector3 v)
{
    return 1.0 / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 L4W4SDK::Vector3_Direction(Vector3 v)
{
    float invSqrt = Vector3_invSqrt(v);
    Vector3 dv = v;
    dv.x *= invSqrt;
    dv.y *= invSqrt;
    dv.z *= invSqrt;

    return dv;
}

float L4W4SDK::Clamp(float value, float min, float max)
{
    if (min > max)
    {
        float tmp = min;
        min = max;
        max = tmp;
    }

    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

float L4W4SDK::Angle_vA_2_vB(Vector3 vA, Vector3 vB, Vector3 axle)
{
    Vector3 vA_dir = Vector3_Direction(vA);
    Vector3 vB_dir = Vector3_Direction(vB);
    Vector3 axle_dir = Vector3_Direction(axle);

    float sinA = Clamp(Vector3_Dot(axle_dir, Vector3_Cross(vA_dir, vB_dir)), -1, 1);
    float angle_raw = asinf(sinA);

    float angle = Vector3_Dot(vA_dir, vB_dir) > 0 ? angle_raw : Sign(angle_raw) * M_PI - angle_raw;

    return angle;
}

void L4W4SDK::Vector3_Normalize(Vector3 *v)
{
    float invSqrt = Vector3_invSqrt(*v);
    v->x *= invSqrt;
    v->y *= invSqrt;
    v->z *= invSqrt;
}
Vector3 L4W4SDK::Quaternion_Transform(float vx, float vy, float vz, float qx, float qy, float qz, float qw)
{
    float dot_u_v = qx * vx + qy * vy + qz * vz;
    float dot_u_u = qx * qx + qy * qy + qz * qz;
    float x_of_uXv = qy * vz - qz * vy;
    float y_of_uXv = qz * vx - qx * vz;
    float z_of_uXv = qx * vy - qy * vx;
    float k1 = 2.0 * dot_u_v;
    float k2 = qw * qw - dot_u_u;
    float k3 = 2.0 * qw;

    Vector3 vout;
    vout.x = k1 * qx + k2 * vx + k3 * x_of_uXv;
    vout.y = k1 * qy + k2 * vy + k3 * y_of_uXv;
    vout.z = k1 * qz + k2 * vz + k3 * z_of_uXv;
    return vout;
}

void L4W4SDK::AnalyzeUDP(unsigned char *recv_buff, LowState &lowState)
{
    int idx = 2;
    tmp_time_from_mcu = read_float(recv_buff, &idx);
    float BatteryVoltage = read_from1byte(recv_buff, &idx, 20, 60);
    float MCUTemperature = read_byte(recv_buff, &idx);

    float acc_lx = read_float(recv_buff, &idx);
    float acc_ly = read_float(recv_buff, &idx);
    float acc_lz = read_float(recv_buff, &idx);
    float omega_lx = read_float(recv_buff, &idx);
    float omega_ly = read_float(recv_buff, &idx);
    float omega_lz = read_float(recv_buff, &idx);
    float orientation_x = read_float(recv_buff, &idx);
    float orientation_y = read_float(recv_buff, &idx);
    float orientation_z = read_float(recv_buff, &idx);
    float orientation_w = read_float(recv_buff, &idx);

    Vector3 robot_up_w = Quaternion_Transform(0, 1, 0, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 current_trunk_front = Quaternion_Transform(1, 0, 0, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 current_trunk_right = Quaternion_Transform(0, 0, 1, orientation_x, orientation_y, orientation_z, orientation_w);
    Vector3 trunk_hori_front = current_trunk_front;
    trunk_hori_front.y = 0;
    Vector3_Normalize(&trunk_hori_front);
    Vector3 trunk_hori_right = Vector3_Cross(trunk_hori_front, {0, 1, 0});
    float r2d = 180.0f / M_PI;

    float robot_yaw_deg = Angle_vA_2_vB({1, 0, 0}, trunk_hori_front, {0, 1, 0}) * r2d;
    float robot_pitch_deg = Angle_vA_2_vB(trunk_hori_front, current_trunk_front, trunk_hori_right) * r2d;
    float robot_roll_deg = Angle_vA_2_vB(trunk_hori_right, current_trunk_right, current_trunk_front) * r2d;

    read_byte(recv_buff, &idx);
    read_byte(recv_buff, &idx);

    xRockerBtnDataStruct *rockerBtn = (xRockerBtnDataStruct *)(&(lowState.wirelessRemote));

    unsigned char key1 = recv_buff[idx];
    idx++;
    rockerBtn->btn.components.R1 = (key1 & 0x80) >> 7;
    rockerBtn->btn.components.L1 = (key1 & 0x40) >> 6;
    rockerBtn->btn.components.start = (key1 & 0x20) >> 5;
    rockerBtn->btn.components.select = (key1 & 0x10) >> 4;
    rockerBtn->btn.components.R2 = (key1 & 0x08) >> 3;
    rockerBtn->btn.components.L2 = (key1 & 0x04) >> 2;
    rockerBtn->btn.components.F1 = (key1 & 0x02) >> 1;
    rockerBtn->btn.components.F2 = (key1 & 0x01) >> 0;

    unsigned char key2 = recv_buff[idx];
    idx++;
    rockerBtn->btn.components.A = (key2 & 0x80) >> 7;
    rockerBtn->btn.components.B = (key2 & 0x40) >> 6;
    rockerBtn->btn.components.X = (key2 & 0x20) >> 5;
    rockerBtn->btn.components.Y = (key2 & 0x10) >> 4;
    rockerBtn->btn.components.up = (key2 & 0x08) >> 3;
    rockerBtn->btn.components.right = (key2 & 0x04) >> 2;
    rockerBtn->btn.components.down = (key2 & 0x02) >> 1;
    rockerBtn->btn.components.left = (key2 & 0x01) >> 0;

    rockerBtn->lx = read_float(recv_buff, &idx);
    rockerBtn->rx = read_float(recv_buff, &idx);
    rockerBtn->ly = read_float(recv_buff, &idx);
    rockerBtn->L2 = read_float(recv_buff, &idx);
    rockerBtn->ry = read_float(recv_buff, &idx);

    if (rockerBtn->btn.components.F1)
        show_rc = 1;
    else
        show_rc = 0;

    idx += 4 * 4;

    float motor_leg1_j0 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg1_j0_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg1_j1 = read_from2bytes(recv_buff, &idx, -1.3 * M_PI, 0.7 * M_PI);
    float motor_leg1_j1_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg1_j2 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg1_j2_dot = read_from2bytes(recv_buff, &idx, -33, 33);

    float motor_leg2_j0 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg2_j0_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg2_j1 = read_from2bytes(recv_buff, &idx, -1.3 * M_PI, 0.7 * M_PI);
    float motor_leg2_j1_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg2_j2 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg2_j2_dot = read_from2bytes(recv_buff, &idx, -33, 33);

    float motor_leg3_j0 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg3_j0_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg3_j1 = read_from2bytes(recv_buff, &idx, -1.3 * M_PI, 0.7 * M_PI);
    float motor_leg3_j1_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg3_j2 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg3_j2_dot = read_from2bytes(recv_buff, &idx, -33, 33);

    float motor_leg4_j0 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg4_j0_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg4_j1 = read_from2bytes(recv_buff, &idx, -1.3 * M_PI, 0.7 * M_PI);
    float motor_leg4_j1_dot = read_from2bytes(recv_buff, &idx, -33, 33);
    float motor_leg4_j2 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg4_j2_dot = read_from2bytes(recv_buff, &idx, -33, 33);

    float motor_leg1_j3 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg1_j3_dot = read_from2bytes(recv_buff, &idx, -200, 200);
    float motor_leg2_j3 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg2_j3_dot = read_from2bytes(recv_buff, &idx, -200, 200);
    float motor_leg3_j3 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg3_j3_dot = read_from2bytes(recv_buff, &idx, -200, 200);
    float motor_leg4_j3 = read_from2bytes(recv_buff, &idx, -M_PI, M_PI);
    float motor_leg4_j3_dot = read_from2bytes(recv_buff, &idx, -200, 200);

    lowState.imu.quaternion[0] = orientation_w;
    lowState.imu.quaternion[1] = orientation_x;
    lowState.imu.quaternion[2] = -orientation_z;
    lowState.imu.quaternion[3] = orientation_y;

    lowState.imu.gyroscope[0] = omega_lx;
    lowState.imu.gyroscope[1] = -omega_lz;
    lowState.imu.gyroscope[2] = omega_ly;
    lowState.imu.accelerometer[0] = acc_lx;
    lowState.imu.accelerometer[1] = -acc_lz;
    lowState.imu.accelerometer[2] = acc_ly;

    //   leg4   leg1    leg3    leg2
    //   hip thigh shank wheel

    lowState.motorState[0].q = lowState.motorState[0].q_raw = motor_leg4_j0;
    lowState.motorState[0].dq = lowState.motorState[0].dq_raw = motor_leg4_j0_dot;
    lowState.motorState[0].ddq = lowState.motorState[0].ddq_raw = 0;
    lowState.motorState[1].q = lowState.motorState[1].q_raw = -motor_leg4_j1;
    lowState.motorState[1].dq = lowState.motorState[1].dq_raw = -motor_leg4_j1_dot;
    lowState.motorState[1].ddq = lowState.motorState[1].ddq_raw = 0;
    lowState.motorState[2].q = lowState.motorState[2].q_raw = -motor_leg4_j2;
    lowState.motorState[2].dq = lowState.motorState[2].dq_raw = -motor_leg4_j2_dot;
    lowState.motorState[2].ddq = lowState.motorState[2].ddq_raw = 0;
    lowState.motorState[3].q = lowState.motorState[3].q_raw = -motor_leg4_j3;
    lowState.motorState[3].dq = lowState.motorState[3].dq_raw = -motor_leg4_j3_dot;
    lowState.motorState[3].ddq = lowState.motorState[3].ddq_raw = 0;

    lowState.motorState[4].q = lowState.motorState[4].q_raw = motor_leg1_j0;
    lowState.motorState[4].dq = lowState.motorState[4].dq_raw = motor_leg1_j0_dot;
    lowState.motorState[4].ddq = lowState.motorState[4].ddq_raw = 0;
    lowState.motorState[5].q = lowState.motorState[5].q_raw = -motor_leg1_j1;
    lowState.motorState[5].dq = lowState.motorState[5].dq_raw = -motor_leg1_j1_dot;
    lowState.motorState[5].ddq = lowState.motorState[5].ddq_raw = 0;
    lowState.motorState[6].q = lowState.motorState[6].q_raw = -motor_leg1_j2;
    lowState.motorState[6].dq = lowState.motorState[6].dq_raw = -motor_leg1_j2_dot;
    lowState.motorState[6].ddq = lowState.motorState[6].ddq_raw = 0;
    lowState.motorState[7].q = lowState.motorState[7].q_raw = -motor_leg1_j3;
    lowState.motorState[7].dq = lowState.motorState[7].dq_raw = -motor_leg1_j3_dot;
    lowState.motorState[7].ddq = lowState.motorState[7].ddq_raw = 0;

    lowState.motorState[8].q = lowState.motorState[8].q_raw = motor_leg3_j0;
    lowState.motorState[8].dq = lowState.motorState[8].dq_raw = motor_leg3_j0_dot;
    lowState.motorState[8].ddq = lowState.motorState[8].ddq_raw = 0;
    lowState.motorState[9].q = lowState.motorState[9].q_raw = -motor_leg3_j1;
    lowState.motorState[9].dq = lowState.motorState[9].dq_raw = -motor_leg3_j1_dot;
    lowState.motorState[9].ddq = lowState.motorState[9].ddq_raw = 0;
    lowState.motorState[10].q = lowState.motorState[10].q_raw = -motor_leg3_j2;
    lowState.motorState[10].dq = lowState.motorState[10].dq_raw = -motor_leg3_j2_dot;
    lowState.motorState[10].ddq = lowState.motorState[10].ddq_raw = 0;
    lowState.motorState[11].q = lowState.motorState[11].q_raw = -motor_leg3_j3;
    lowState.motorState[11].dq = lowState.motorState[11].dq_raw = -motor_leg3_j3_dot;
    lowState.motorState[11].ddq = lowState.motorState[11].ddq_raw = 0;

    lowState.motorState[12].q = lowState.motorState[12].q_raw = motor_leg2_j0;
    lowState.motorState[12].dq = lowState.motorState[12].dq_raw = motor_leg2_j0_dot;
    lowState.motorState[12].ddq = lowState.motorState[12].ddq_raw = 0;
    lowState.motorState[13].q = lowState.motorState[13].q_raw = -motor_leg2_j1;
    lowState.motorState[13].dq = lowState.motorState[13].dq_raw = -motor_leg2_j1_dot;
    lowState.motorState[13].ddq = lowState.motorState[13].ddq_raw = 0;
    lowState.motorState[14].q = lowState.motorState[14].q_raw = -motor_leg2_j2;
    lowState.motorState[14].dq = lowState.motorState[14].dq_raw = -motor_leg2_j2_dot;
    lowState.motorState[14].ddq = lowState.motorState[14].ddq_raw = 0;
    lowState.motorState[15].q = lowState.motorState[15].q_raw = -motor_leg2_j3;
    lowState.motorState[15].dq = lowState.motorState[15].dq_raw = -motor_leg2_j3_dot;
    lowState.motorState[15].ddq = lowState.motorState[15].ddq_raw = 0;

    // ctr++;
    // if (ctr >= 100)
    // {
    //     // float r2d = 180.0/M_PI;
    //     // std::cout<<"mcu time = "<<tmp_time_from_mcu<<",\tbattery = "<< BatteryVoltage<<std::endl;
    //     // std::cout<<"lx = "<<rockerBtn->lx<<",\try = "<<rockerBtn->ry<<",\trx = "<<rockerBtn->rx<<std::endl;
    //     // if(show_rc)
    //     if (0)
    //     {
    //         std::cout << "   rc: " << ((int)(rockerBtn->ly * 100) / 100.0) << "\t" << ((int)(rockerBtn->lx * 100) / 100.0);
    //         std::cout << "\t" << ((int)(rockerBtn->ry * 100) / 100.0) << "\t" << ((int)(rockerBtn->rx * 100) / 100.0) << std::endl;
    //     }

    //     {
    //         std::cout<<"                    roll, roll_offset.        vz = "<<robot_roll_deg<<",\t"<< current_roll_offset<<std::endl;
    //     }

    //     ctr = 0;
    // }
}

#endif // L4W4_SDK_HPP

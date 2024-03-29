// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROJECT_CUSTOMINTERFACE_H
#define PROJECT_CUSTOMINTERFACE_H

#include "leg_control_data_lcmt.hpp"
#include "motor_ctrl_lcmt.hpp"
#include "motor_ctrl_state_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

struct Robot_Data
{
	float q[12];              // 12个关节电机角度，弧度制
	float qd[12];             //电机角速度，弧度制
	float tau[12];            //电机扭矩 N.M
	float quat[4];            //机身姿态四元数，右手坐标系 w, x, y, z
	float rpy[3];             //机身姿态横滚、俯仰、偏航角 弧度制
	float acc[3];             //加速度计值
	float omega[3];           //角速度计值
	float ctrl_topic_interval;  //控制topic通信延迟
	int16_t err_flag;
};
struct Motor_Cmd
{
	//期望扭矩tau = tau_des + (q_des - q)*kp_des + (qd_des - qd)*kd_des
	float q_des[12];    // 12个关机电机期望角度，弧度制  0/3/6/9:-0.75~0.75  1/4:-1.257 ~ 4.363  7/10:-2.01 ~ 3.49  2/5/8/11:-2.478 ~ -0.506
	float qd_des[12];   //电机期望角速度，弧度制  +-12弧度/秒@24NM
	float kp_des[12];   //电机位置控制比例系数 0~200
	float kd_des[12];   //电机速度控制比例系数 0~10
	float tau_des[12];  //电机期望前馈扭矩  0/3/6/9:+-17N/M 1/2/4/5/7/8/10/11:+-24NM
};

class CustomInterface
{
public:
	CustomInterface(const double &loop_rate);
	void Spin();
	void Stop();

protected:
	virtual void UserCode() = 0;
	Robot_Data robot_data;
	Motor_Cmd motor_cmd;

private:
	double dt_;
	bool running_;
	bool all_thread_done_;
	bool mode_state;

	lcm::LCM _motor_ctrl_Lcm;
	lcm::LCM _motor_data_Lcm;
	lcm::LCM _robot_state_Lcm;
	lcm::LCM _motor_ctrl_state_Lcm;

	std::thread _motor_ctrl_state_LcmThread;
	std::thread _motor_data_LcmThread;
	std::thread _robot_state_LcmThread;
	std::thread _user_code_ControlThread;

	motor_ctrl_lcmt _motor_ctrl;

	std::string getLcmUrl_port(int64_t port, int64_t ttl);

	void motor_data_LcmThread()
	{
		while(running_)
		{
			_motor_data_Lcm.handleTimeout(1000);
		}
	}
	void robot_state_LcmThread()
	{
		while(running_)
		{
			_robot_state_Lcm.handleTimeout(1000);
		}
	}
	void motor_ctrl_state_LcmThread()
	{
		while(running_)
		{
			_motor_ctrl_state_Lcm.handleTimeout(1000);
		}
	}

	void handle_motor_ctrl_state_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const motor_ctrl_state_lcmt *msg);
	void handle_motor_data_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const leg_control_data_lcmt *msg);
	void handle_robot_state_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const state_estimator_lcmt *msg);

	void motor_cmd_send();
	void Control();
};  // CustomInterface
#endif
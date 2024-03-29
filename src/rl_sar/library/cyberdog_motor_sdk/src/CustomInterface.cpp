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

#include <CustomInterface.h>

std::string CustomInterface::getLcmUrl_port(int64_t port, int64_t ttl)
{
	assert(ttl >= 0 && ttl <= 255);
	return "udpm://239.255.76.67:" + std::to_string(port) + "?ttl=" + std::to_string(ttl);
}

CustomInterface::CustomInterface(const double &loop_rate)
		: _motor_data_Lcm(getLcmUrl_port(7667, 255)), _motor_ctrl_state_Lcm(getLcmUrl_port(7667, 255)),
		  _robot_state_Lcm(getLcmUrl_port(7669, 255)),
		  _motor_ctrl_Lcm(getLcmUrl_port(7667, 255))
{

	running_ = true;
	all_thread_done_ = false;
	mode_state = false;

	if(loop_rate > 0)
	{
		dt_ = 1.0 / loop_rate;
	}
	else
	{
		std::cout << "Loop rate should be more than zero! Set to default 500Hz." << std::endl;
		dt_ = 1.0 / 500;
	}

	_motor_ctrl_state_Lcm.subscribe("motor_ctrl_state", &CustomInterface::handle_motor_ctrl_state_LCM, this);
	_motor_data_Lcm.subscribe("leg_control_data", &CustomInterface::handle_motor_data_LCM, this);
	_robot_state_Lcm.subscribe("state_estimator", &CustomInterface::handle_robot_state_LCM, this);

	_motor_ctrl_state_LcmThread = std::thread(&CustomInterface::motor_ctrl_state_LcmThread, this);
	_motor_data_LcmThread = std::thread(&CustomInterface::motor_data_LcmThread, this);
	_robot_state_LcmThread = std::thread(&CustomInterface::robot_state_LcmThread, this);
	_user_code_ControlThread = std::thread(&CustomInterface::Control, this);
}

void CustomInterface::Control()
{
	auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
	int seconds = (int) dt_;
	int nanoseconds = (int) (1e9 * std::fmod(dt_, 1.f));
	int err_count = 0;

	itimerspec timerSpec;
	timerSpec.it_interval.tv_sec = seconds;
	timerSpec.it_value.tv_sec = seconds;
	timerSpec.it_value.tv_nsec = nanoseconds;
	timerSpec.it_interval.tv_nsec = nanoseconds;

	timerfd_settime(timerFd, 0, &timerSpec, nullptr);
	unsigned long long missed = 0;
	while(running_)
	{
		if(!mode_state)
		{
			sleep(1);
			std::cout << "Motor control mode has not been activated successfully" << std::endl;
			continue;
		}

		UserCode();

		if(robot_data.err_flag & 0x02)
		{
			if(err_count++ % 1000 == 0)
			{
				printf("\nErr: 0x02 Communicate lost over 500ms!\n");
				printf("The motor cmd has been disabled!!!\n");
				printf("Cyberdog locomotion ctrl process need restart after fix the communication!\n\n");
			}
		}
		else
		{
			motor_cmd_send();
		}

		int m = read(timerFd, &missed, sizeof(missed));
		(void) m;
	}
}

void CustomInterface::Spin()
{
	while(!all_thread_done_)
	{
		sleep(1.0);
	}

	printf("~ Exit ~\n");
}

void CustomInterface::Stop()
{
	running_ = false;
	_motor_ctrl_state_LcmThread.join();
	_motor_data_LcmThread.join();
	_robot_state_LcmThread.join();
	_user_code_ControlThread.join();
	all_thread_done_ = true;
}

void CustomInterface::motor_cmd_send()
{
	int sig[12] = {1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1};
	for(int i = 0; i < 12; i++)
	{
		_motor_ctrl.q_des[i] = motor_cmd.q_des[i] * sig[i];
		_motor_ctrl.qd_des[i] = motor_cmd.qd_des[i] * sig[i];
		_motor_ctrl.kp_des[i] = motor_cmd.kp_des[i];
		_motor_ctrl.kd_des[i] = motor_cmd.kd_des[i];
		_motor_ctrl.tau_des[i] = motor_cmd.tau_des[i] * sig[i];
	}
	_motor_ctrl_Lcm.publish("motor_ctrl", &_motor_ctrl);
}

void CustomInterface::handle_motor_ctrl_state_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                                  const motor_ctrl_state_lcmt *msg)
{
	(void) rbuf;
	(void) chan;
	robot_data.err_flag = msg->err_flag;
	robot_data.ctrl_topic_interval = msg->ctrl_topic_interval;
	mode_state = true;
}

void CustomInterface::handle_motor_data_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                            const leg_control_data_lcmt *msg)
{
	(void) rbuf;
	(void) chan;
	int sig[12] = {1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1};
	for(int i = 0; i < 12; i++)
	{
		robot_data.q[i] = msg->q[i] * sig[i];
		robot_data.qd[i] = msg->qd[i] * sig[i];
		robot_data.tau[i] = msg->tau_est[i] * sig[i];
	}
}

void CustomInterface::handle_robot_state_LCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                             const state_estimator_lcmt *msg)
{
	(void) rbuf;
	(void) chan;
	for(int i = 0; i < 3; i++)
	{
		robot_data.omega[i] = msg->omegaWorld[i];
		robot_data.rpy[i] = msg->rpy[i];
		robot_data.acc[i] = msg->aWorld[i];
	}
	for(int i = 0; i < 4; i++)
	{
		robot_data.quat[i] = msg->quat[i];
	}
}
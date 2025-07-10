/// @file main.cpp
/// @author your name (you@domain.com)
/// @brief 
/// @version 0.1
/// @date 2022-09-13 
/// @copyright Copyright (c) 2022


#include "udpsocket.hpp"
#include "udpserver.hpp"
#include "sender.h"
#include "dr_timer.h"
#include "receiver.h"
#include "motionexample.h"
#include <iostream>
#include <time.h>
#include <string.h>

using namespace std;

  bool is_message_updated_ = false; ///< Flag to check if message has been updated
  /**
   * @brief Callback function to set message update flag
   * 
   * @param code The code indicating the type of message received
   */
  void OnMessageUpdate(uint32_t code){
    if(code == 0x0906){
      is_message_updated_ = true;
    }
  }

int main(int argc, char* argv[]){
  DRTimer set_timer;
  double now_time,start_time;
  RobotCmd robot_joint_cmd;
  memset(&robot_joint_cmd, 0, sizeof(robot_joint_cmd));

  Sender* send_cmd          = new Sender("192.168.2.1",43893);              ///< Create send thread
  Receiver* robot_data_recv = new Receiver();                                 ///< Create a receive resolution
  robot_data_recv->RegisterCallBack(OnMessageUpdate);
  MotionExample robot_set_up_demo;                                            ///< Demos for testing can be deleted by yourself
  RobotData *robot_data = &robot_data_recv->GetState();

  robot_data_recv->StartWork();
  set_timer.TimeInit(1);                                                      ///< Timer initialization, input: cycle; Unit: ms
  send_cmd->RobotStateInit();                                                 ///< Return all joints to zero and gain control

  start_time = set_timer.GetCurrentTime();                                    ///< Obtain time for algorithm usage
  robot_set_up_demo.GetInitData(robot_data->joint_data,0.000);                ///< Obtain all joint states once before each stage (action)
  
  int time_tick = 0;
  while(1){
    if (set_timer.TimerInterrupt() == true){                                  ///< Time interrupt flag
      continue;
    }
    now_time = set_timer.GetIntervalTime(start_time);                         ///< Get the current time
    time_tick++;
    if(time_tick < 1000){
      robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*robot_data);     ///< Stand up and prepare for action
    } 
    if(time_tick == 1000){
      robot_set_up_demo.GetInitData(robot_data->joint_data,now_time);         ///< Obtain all joint states once before each stage (action)
    }
    if(time_tick >= 1000 ){
      robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*robot_data);        ///< Full stand up
    }
    if(time_tick >= 10000){
      send_cmd->ControlGet(ROBOT);                                            ///< Return the control right, input: ROBOT: Original algorithm control of the robot .  SDK: SDK control PS: over 50ms, no data set sent_ Send (cmd), you will lose control, you need to resend to obtain control
      break;
    }
    if(is_message_updated_){
      send_cmd->SendCmd(robot_joint_cmd);  
    }               
    cout << robot_data->imu.acc_x << endl;
  }
  return 0;
} 

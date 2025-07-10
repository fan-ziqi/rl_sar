/// @file receiver.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2023-03-17 
/// @copyright Copyright (c) 2023

#ifndef RECEIVER_H_
#define RECEIVER_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>

#include "robot_types.h"
#include "common/udpserver.hpp"

/// @brief This class is used for receiving data from the robot.
class Receiver {
  private:
    RobotData state_rec_;  // The received robot state data.

    /// @brief Keep receiving data.
    void Work();

    /// @brief CallBack_. 
    /// @param int Instruction type, only 0x0906.
    std::function<void(int)> CallBack_;
  public:

    /// @brief Registering Callbacks.
    void RegisterCallBack(std::function<void(int)> CallBack){
      CallBack_ = std::move(CallBack);
    }
    /// @brief Construct a new Receiver object.
    Receiver();

    /// @brief Start the receiving process.
    void StartWork();
    
    /// @brief Destroy the Receiver object.
    ~Receiver();

    /// @brief Get the received robot state data.
    /// @return RobotData& The reference of the received robot state data.
    RobotData& GetState();
};



#endif  ///< RECEIVER_H_

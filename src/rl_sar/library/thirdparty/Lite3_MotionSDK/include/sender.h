/// @file sender.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2023-03-17 
/// @copyright Copyright (c) 2023
 

#ifndef SENDER_H_
#define SENDER_H_

#include <iostream>
#include <stdint.h>
#include <array>
#include "common/command.h"
#include "common/udpsocket.hpp"
#include "robot_types.h"

#define SDK 2
#define ROBOT 1
/// @class Sender
/// @brief Class for sending RobotCmd through UDP socket.
class Sender {
  private:
    RobotCmd robot_cmd_; /**< The RobotCmd object to be sent. */
    UDPSocket* udp_socket_; /**< The UDP socket used for sending data. */

    /// @brief Send command to Lite.
    /// @param[in] command The Command object that has been executed.
    void CmdDone(Command& command);

  public:
    /// @brief Default constructor.
    /// Initialize Sender object with default IP and port.
    Sender();

    /// @brief Constructor with IP and port parameters.
    /// @param[in] ip The IP address to send data to.
    /// @param[in] port The port number to send data to.
    Sender(std::string ip = "192.168.1.120", uint16_t port = 43893);

    /// @brief Destructor.
    ~Sender();

    /// @brief Send RobotCmd through UDP socket.
    /// @param[in] robot_cmd The RobotCmd struct to be sent.
    void SendCmd(RobotCmd& robot_cmd);

    /// @brief Send control_get command with specified mode.
    /// @param[in] mode The mode of control_get command.You can write "SDK" or "ROBOT".
    void ControlGet(uint32_t mode);

    /// @brief Send a command to make all joints return to the zero position and acquisition of control.
    void AllJointBackZero(void);

    /// @brief Make all joints return to the zero position
    void RobotStateInit(void);

    /// @brief Set the value of a command.
    /// @param[in] code The code of the command.
    /// @param[in] value The value to be set.
    void SetCmd(uint32_t code, uint32_t value);
};



#endif  ///< PARSE_CMD_H_
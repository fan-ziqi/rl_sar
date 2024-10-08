#ifndef ROBOT_JOINT_CONTROLLER_H
#define ROBOT_JOINT_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "robot_msgs/MotorCommand.h"
#include "robot_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <math.h>

#define PosStopF (2.146E+9f)
#define VelStopF (16000.0f)

typedef struct
{
    uint8_t mode;
    double pos;
    double posStiffness;
    double vel;
    double velStiffness;
    double torque;
} ServoCommand;

namespace robot_joint_controller
{
    class RobotJointController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    private:
        hardware_interface::JointHandle joint;
        ros::Subscriber sub_command, sub_ft;
        control_toolbox::Pid pid_controller_;
        std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::MotorState>> controller_state_publisher_;

    public:
        std::string name_space;
        std::string joint_name;
        urdf::JointConstSharedPtr joint_urdf;
        realtime_tools::RealtimeBuffer<robot_msgs::MotorCommand> command;
        robot_msgs::MotorCommand lastCommand;
        robot_msgs::MotorState lastState;
        ServoCommand servoCommand;

        RobotJointController();
        ~RobotJointController();
        virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        virtual void starting(const ros::Time &time);
        virtual void update(const ros::Time &time, const ros::Duration &period);
        virtual void stopping();
        void setCommandCB(const robot_msgs::MotorCommandConstPtr &msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    };
}

#endif

/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_joint_controller.h"
#include <pluginlib/class_list_macros.h>

// #define rqtTune // use rqt or not

double clamp(double &value, double min, double max)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
    return value;
}

namespace robot_joint_controller
{

    RobotJointController::RobotJointController()
    {
        memset(&lastCommand, 0, sizeof(robot_msgs::MotorCommand));
        memset(&lastState, 0, sizeof(robot_msgs::MotorState));
        memset(&servoCommand, 0, sizeof(ServoCommand));
    }

    RobotJointController::~RobotJointController()
    {
        sub_ft.shutdown();
        sub_command.shutdown();
    }

    void RobotJointController::setCommandCB(const robot_msgs::MotorCommandConstPtr &msg)
    {
        lastCommand.q = msg->q;
        lastCommand.kp = msg->kp;
        lastCommand.dq = msg->dq;
        lastCommand.kd = msg->kd;
        lastCommand.tau = msg->tau;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(lastCommand);
    }

    // Controller initialization in non-realtime
    bool RobotJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name))
        {
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }

        // load pid param from ymal only if rqt need
#ifdef rqtTune
        // Load PID Controller using gains set on parameter server
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
            return false;
#endif

        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        joint = robot->getHandle(joint_name);

        // Start command subscriber
        sub_command = n.subscribe("command", 20, &RobotJointController::setCommandCB, this);

        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<robot_msgs::MotorState>(n, name_space + "/state", 1));

        return true;
    }

    void RobotJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
    }

    void RobotJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
    }

    void RobotJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
    }

    // Controller startup in realtime
    void RobotJointController::starting(const ros::Time &time)
    {
        double init_pos = joint.getPosition();
        lastCommand.q = init_pos;
        lastState.q = init_pos;
        lastCommand.dq = 0;
        lastState.dq = 0;
        lastCommand.tau = 0;
        lastState.tau_est = 0;
        command.initRT(lastCommand);

        pid_controller_.reset();
    }

    // Controller update loop in realtime
    void RobotJointController::update(const ros::Time &time, const ros::Duration &period)
    {
        double currentPos, currentVel, calcTorque;
        lastCommand = *(command.readFromRT());

        // set command data
        servoCommand.pos = lastCommand.q;
        positionLimits(servoCommand.pos);
        servoCommand.posStiffness = lastCommand.kp;
        if (fabs(lastCommand.q - PosStopF) < 0.00001)
        {
            servoCommand.posStiffness = 0;
        }
        servoCommand.vel = lastCommand.dq;
        velocityLimits(servoCommand.vel);
        servoCommand.velStiffness = lastCommand.kd;
        if (fabs(lastCommand.dq - VelStopF) < 0.00001)
        {
            servoCommand.velStiffness = 0;
        }
        servoCommand.torque = lastCommand.tau;
        effortLimits(servoCommand.torque);

        // rqt set P D gains
#ifdef rqtTune
        double i, i_max, i_min;
        getGains(servoCommand.posStiffness, i, servoCommand.velStiffness, i_max, i_min);
#endif

        currentPos = joint.getPosition();
        // currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.toSec());
        // calcTorque = computeTorque(currentPos, currentVel, servoCommand);
        currentVel = (currentPos - (double)lastState.q) / period.toSec();
        calcTorque = servoCommand.posStiffness * (servoCommand.pos - currentPos) + servoCommand.velStiffness * (servoCommand.vel - currentVel) + servoCommand.torque;
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        // lastState.tau_est = calcTorque;
        lastState.tau_est = joint.getEffort();

        // publish state
        if (controller_state_publisher_ && controller_state_publisher_->trylock())
        {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tau_est = lastState.tau_est;
            controller_state_publisher_->unlockAndPublish();
        }
    }

    // Controller stopping in realtime
    void RobotJointController::stopping() {}

    void RobotJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void RobotJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void RobotJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(robot_joint_controller::RobotJointController, controller_interface::ControllerBase);

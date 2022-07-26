#ifndef AGV_HW
#define AGV_HW
#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include "can_drive_motor.h"

#include <thread>

//#define SHOW_ORIGIN_CAN_TX


namespace agv_controller_ns
{
    class AgvInterface : public hardware_interface::RobotHW
    {
        public:
        AgvInterface(ros::NodeHandle& nh);
        AgvInterface();
        ~AgvInterface();

        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
        void read(const ros::Time& time, const ros::Duration& period);
        void write(const ros::Time& time, const ros::Duration& period);

    
        private:
        ros::NodeHandle nh_;
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface effort_joint_interface;
        hardware_interface::PositionJointInterface position_joint_interface;
        int num_joints;
        std::vector<std::string> joint_name;

        std::vector<double> wheel_position_state;
        std::vector<double> wheel_velocity_state;
        std::vector<double> wheel_effort_state;
        std::vector<double> wheel_effort_command;

        std::vector<double> turn_position_state;
        std::vector<double> turn_velocity_state;
        std::vector<double> turn_effort_state;
        std::vector<double> turn_position_command;
        Can_drive_motor* can_drive_motor_;

        std::thread* thread_;

    };

    

}

#endif
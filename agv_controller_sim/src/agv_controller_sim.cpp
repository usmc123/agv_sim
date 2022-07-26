#include "agv_controller_sim.h"
#include <pluginlib/class_list_macros.h>




namespace agv_controller_ns
{
    agv_controller_sim::agv_controller_sim(/* args */)
    {
    }
    
    agv_controller_sim::~agv_controller_sim()
    {
    }
    bool agv_controller_sim::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh)
    {
        std::vector<std::string> joint_names;
            if (!nh.getParam("joint",joint_names) ||joint_names.size()!=8)
            {
            ROS_ERROR(
            "Agv_Controller: Invalid or no joint_names parameters provided, aborting "
            "controller init!fuck!!!!!");
            return false;
            }
            ros::Duration time;
            time.sec=5;
            time.sleep();
            auto* turn_joint_interface=robot_hw->get<hardware_interface::PositionJointInterface>();
            // auto* joint_arm_interface=robot_hw->get<hardware_interface::PositionJointInterface>();
            auto* wheel_joint_interface=robot_hw->get<hardware_interface::EffortJointInterface>();
            
            
            ROS_ERROR("turn_joint_interface->getNames().size() = %d",turn_joint_interface->getNames().size());
            ROS_ERROR("wheel_joint_interface->getNames().size() = %d",wheel_joint_interface->getNames().size());
            for (uint8_t i = 0; i < 4; i++)
            {   
                
                joint_handles_.push_back(turn_joint_interface->getHandle(joint_names[i].c_str()));
                ROS_ERROR_STREAM("my name is "<<joint_handles_[i].getName());
            }
            for (uint8_t i = 0; i < 4; i++)
            {   
                
                joint_handles_.push_back(wheel_joint_interface->getHandle(joint_names[i+4].c_str()));
                ROS_ERROR_STREAM("my name is "<<joint_handles_[i+4].getName());
            }
        
            ROS_ERROR("joint_handles_.size() = %d",joint_handles_.size());
            for(int i = 0;i<8;i++)
            {
                ROS_ERROR("Check joint_handles_ = %s",joint_handles_[i].getName().c_str());
            }
            
            
            wheel_joint_state_.resize(4);
            turn_joint_state_.resize(4);

            ROS_ERROR("DEBOG2");
    return true;
    }
    void agv_controller_sim::update(const ros::Time&, const ros::Duration& period)
    {
        ROS_INFO("updata loaded");
    }
    void agv_controller_sim::starting(const ros::Time&)
    {
        ROS_INFO("starting  loaded");
    }


}


PLUGINLIB_EXPORT_CLASS(agv_controller_ns::agv_controller_sim,
                       controller_interface::ControllerBase)
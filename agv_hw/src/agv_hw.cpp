#include "agv_hw.h"
#include <pthread.h>
#define TEST_NO_PTR
#include <vector>
#include <iostream>




namespace agv_controller_ns
{
    AgvInterface::AgvInterface(ros::NodeHandle& nh)
    {

    }
    AgvInterface::AgvInterface()
    {
    }
    bool AgvInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        int ret = robot_hw_nh.getParam("/agv_sim/agv_hardware/joints", joint_name);//从参数服务器中获取name

        ROS_ERROR("getParam ret= %d",ret);
        
        num_joints = joint_name.size();
        for (size_t i = 0; i < num_joints; i++)
        {
            ROS_ERROR("jointname=%s",joint_name[i].c_str());
        }
        wheel_position_state.resize(4);
        wheel_velocity_state.resize(4);
        wheel_effort_state.resize(4);
        wheel_effort_command.resize(4);

        turn_position_state.resize(4);
        turn_velocity_state.resize(4);
        turn_effort_state.resize(4);
        turn_position_command.resize(4);

        can_drive_motor_ = new Can_drive_motor("/dev/can0");


        thread_ = new std::thread(&Can_drive_motor::thread_read,can_drive_motor_);//读取参数,开启接收数据线程
        
        can_drive_motor_->motor_init();//开启电机初始化程序

        if(ret!=0)
        {
            std::cout<<"pthread_create error :recive thread err code = "<<ret<<std::endl;
        }

        // 
    #ifndef TEST_NO_PTR
    for(int i=0; i<4; i++)
    {
        //State turn
        
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i].c_str(),
        can_drive_motor_->tt_motor_.at(i).get_position_state_ptr(), 
        can_drive_motor_->tt_motor_.at(i).get_vel_state_ptr(), 
        can_drive_motor_->tt_motor_.at(i).get_tor_state_ptr());

        joint_state_interface.registerHandle(jointStateHandle);
        ROS_INFO("joint_name[%d].c_str()=%s",i,jointStateHandle.getName().c_str());

        //position turn
        hardware_interface::JointHandle jointPositionHandle(joint_state_interface.getHandle(joint_name[i]),
         can_drive_motor_->tt_motor_.at(i).get_cmd_position_ptr());

         
        position_joint_interface.registerHandle(jointPositionHandle);
    }

    for (size_t i = 0; i < 4; i++)
    {
        //state of wheel
        hardware_interface::JointStateHandle wheel_jointStateHandle(joint_name[i+4].c_str(),
        can_drive_motor_->zl_motor_.at(i).get_position_state_ptr(),
        can_drive_motor_->zl_motor_.at(i).get_vel_state_ptr(),
        can_drive_motor_->zl_motor_.at(i).get_tor_state_ptr());
        joint_state_interface.registerHandle(wheel_jointStateHandle);

        //effort wheel
        hardware_interface::JointHandle jointEffortHandle(joint_state_interface.getHandle(joint_name[i+4]),
         can_drive_motor_->zl_motor_.at(i).get_cmd_position_ptr());
        effort_joint_interface.registerHandle(jointEffortHandle);
    }
    #else
    for(int i=0; i<4; i++)
    {
        //State turn
        
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i].c_str(),
        &turn_position_state[i], 
        &turn_velocity_state[i], 
        &turn_effort_state[i]);

        joint_state_interface.registerHandle(jointStateHandle);
        ROS_INFO("registerHandle joint_name[%d].c_str()=%s",i,jointStateHandle.getName().c_str());

        //position turn
        hardware_interface::JointHandle jointPositionHandle(joint_state_interface.getHandle(joint_name[i].c_str()),
        &turn_position_command[i]);

         
        position_joint_interface.registerHandle(jointPositionHandle);
    }


    for (size_t i = 0; i < 4; i++)
    {   
        //state of wheel
        hardware_interface::JointStateHandle wheel_jointStateHandle(joint_name[i+4].c_str(),
        &wheel_position_state[i],
        &wheel_velocity_state[i],
        &wheel_effort_state[i]);
        joint_state_interface.registerHandle(wheel_jointStateHandle);

        //effort wheel
        hardware_interface::JointHandle jointEffortHandle(joint_state_interface.getHandle(joint_name[i+4].c_str()),
        &wheel_effort_command[i]);
        effort_joint_interface.registerHandle(jointEffortHandle);
    }

    #endif


        ROS_WARN("joint_state_interface.getNames().size()=%d",joint_state_interface.getNames().size());
        ROS_WARN("effort_joint_interface.getNames().size()=%d",effort_joint_interface.getNames().size());
        ROS_WARN("position_joint_interface.getNames().size()=%d",position_joint_interface.getNames().size());




        registerInterface(&joint_state_interface);          //将类中的接口注册到ros中
        registerInterface(&position_joint_interface);
        registerInterface(&effort_joint_interface);
        

    

        return true;

    }
    void AgvInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        // ROS_INFO("turn_1 pos = %f",*this->can_drive_motor_->tt_motor_.at(0).get_position_state_ptr());
    }
    void AgvInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        for(int i =0;i<4;i++)
        {
            // ROS_WARN("write i= %d",i);
            this->can_drive_motor_->set_TT_motor_pos(i);
        }
        this->can_drive_motor_->set_ZL_motor_tor();
        
    }

    AgvInterface::~AgvInterface()
    {
        thread_->join();
    }

    
}

PLUGINLIB_EXPORT_CLASS(agv_controller_ns::AgvInterface, hardware_interface::RobotHW)
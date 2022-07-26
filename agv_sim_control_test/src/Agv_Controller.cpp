#include "Agv_Controller.h"
#include <pluginlib/class_list_macros.h>

// #define DEBUG_WX


namespace agv_controller_ns
{
    void Agv_Controller::doKeyctrl(const agv_sim_control_test::cmd_xyw::ConstPtr& cmd_xyz)
    {
        cmd_xyz_.vx=cmd_xyz->vx;
        cmd_xyz_.vy=cmd_xyz->vy;
        cmd_xyz_.vw=cmd_xyz->vw;
    }



    bool Agv_Controller::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh)
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
            
            joint_handles_.push_back(turn_joint_interface->getHandle(joint_names[i]));
            ROS_ERROR_STREAM("my name is "<<joint_handles_[i].getName());
        }
        for (uint8_t i = 0; i < 4; i++)
        {   
            
            joint_handles_.push_back(wheel_joint_interface->getHandle(joint_names[i+4]));
            ROS_ERROR_STREAM("my name is "<<joint_handles_[i+4].getName());
        }
       
        ROS_ERROR("joint_handles_.size() = %d",joint_handles_.size());

        // for (uint8_t i = 0; i < 6; i++)
        // {
        //     joint_handles_.push_back(joint_arm_interface->getHandle(joint_names[i]));
        //     ROS_ERROR_STREAM("my name is "<<joint_handles_[i+8].getName());
        // }
        
        // for (uint8_t i = 0; i < 4; i++)
        // {   
        //     joint_handles_.push_back(wheel_joint_interface->getHandle(joint_names[i+4]));
        // }
        agv_chassis=  new Chassis(1.062,1.0613);//float length,float width
        
        wheel_joint_state_.resize(4);
        turn_joint_state_.resize(4);

        ROS_ERROR("DEBOG2");
        
        sub = nh.subscribe<agv_sim_control_test::cmd_xyw>
        ("/cmd_xyw",10,&Agv_Controller::doKeyctrl,this,ros::TransportHints().reliable().tcpNoDelay());
    }


    void Agv_Controller::starting(const ros::Time&)
    {

        
        for (uint8_t i = 0; i < 4; i++)
        {
           turn_joint_init_pos_.push_back(joint_handles_[i].getPosition());
        }
        for (uint8_t i = 0; i < 4; i++)
        {
           wheel_joint_init_pos_.push_back(joint_handles_[i+4].getPosition());
           wheel_joint_state_[i].position=joint_handles_[i+4].getPosition();
           wheel_joint_state_[i].velocity=joint_handles_[i+4].getVelocity();
           wheel_joint_state_[i].effort=joint_handles_[i+4].getEffort();
        }
        agv_chassis->set_wheels_v_feedback(&wheel_joint_state_);

joint_handles_[0].setCommand(0);
joint_handles_[1].setCommand(0);
joint_handles_[2].setCommand(0);
joint_handles_[3].setCommand(0);
cmd_xyz_.vw=0;
cmd_xyz_.vx=0.0;
cmd_xyz_.vy=0.0;

        for (uint8_t i = 0; i < 4; i++)
        {
            
            joint_handles_[i+4].setCommand(0.0);
        }

        
        sleep(5);
    }
    void Agv_Controller::update(const ros::Time& time_now, const ros::Duration& period)
    {

        ROS_ERROR("DEBUG_3");
        // ROS_INFO("init_position turn1=%f,turn2=%f,turn3=%f,turn_4=%f",
        // turn_joint_init_pos_[0],turn_joint_init_pos_[1],turn_joint_init_pos_[2],turn_joint_init_pos_[3]); 

        static float time_last=0;

        // ROS_INFO("timecal=%f,calHZ=%f",time_now.toSec()-time_last,1/(time_now.toSec()-time_last));
        // time_last=time_now.toSec();


        #ifdef  DEBUG_JOINT_INFO
        ROS_INFO("now_position turn1=%f,turn2=%f,turn3=%f,turn_4=%f",
        joint_handles_[0].getPosition(),joint_handles_[1].getPosition(),joint_handles_[2].getPosition(),joint_handles_[3].getPosition());
        for(uint8_t i=0;i<4;i++)
        {
            ROS_INFO("now_position wheel%d=%f\n",i+1,joint_handles_[i+4].getPosition());
        }
        #endif 
        // ROS_INFO("period=%f,HZ=%f",period.toSec(),1/period.toSec());

        //获取行走电机速度反馈

        for (uint8_t i = 0; i < 4; i++)
        {
           wheel_joint_state_[i].position=joint_handles_[i+4].getPosition();
           wheel_joint_state_[i].velocity=(joint_handles_[i+4].getVelocity());

        //    wheel_joint_state_[i].velocity=(joint_handles_[i+4].getVelocity()/Rad_to_deg);
        //    ROS_WARN("joint_handles_[%d+4].getVelocity()=%f",i,wheel_joint_state_[i].velocity);
           wheel_joint_state_[i].effort=joint_handles_[i+4].getEffort();

           turn_joint_state_[i].position=joint_handles_[i].getPosition();
           turn_joint_state_[i].velocity=joint_handles_[i].getVelocity();
           turn_joint_state_[i].effort=joint_handles_[i].getEffort();
        }


        agv_chassis->set_wheels_v_feedback(&wheel_joint_state_);
        agv_chassis->set_turn_feedback(&turn_joint_state_);


        agv_chassis->Control(cmd_xyz_.vx,cmd_xyz_.vy,cmd_xyz_.vw);
         ROS_INFO("cmd_xyz_.vx=%f,cmd_xyz_.vy=%f,cmd_xyz_.vw=%f\n",cmd_xyz_.vx,cmd_xyz_.vy,cmd_xyz_.vw);
        auto temp_control = agv_chassis->get_cmd_motor();
        
        for (uint8_t i = 0; i < 4; i++)
        {
            #ifdef DEBUG_WX
            joint_handles_[i].setCommand(temp_control[i].motor_turn);
            // joint_handles_[i+4].setCommand(0.0);
            joint_handles_[i+4].setCommand(temp_control[i].motor_wheel);
            #else
            
            joint_handles_[i].setCommand(temp_control[i].motor_turn);
            // joint_handles_[i+4].setCommand(0.0);
            joint_handles_[i+4].setCommand(temp_control[i].motor_wheel);
            // ROS_ERROR("temp_control[%d].motor_turn=%f",i,temp_control[i].motor_turn);
            // ROS_ERROR("temp_control[%d].motor_wheel=%f",i,temp_control[i].motor_wheel);

            #endif
        }
        // for (uint8_t i = 0; i < 6; i++)
        // {
        //     joint_handles_[i+8].setCommand(0.0);
        // }
        

        // for(uint8_t i=0;i<4;i++)
        // {
        //     joint_handles_[i].setCommand(2);
            // joint_handles_[i+4].setCommand(1.5);
        // }
        
    }

    Agv_Controller::Agv_Controller()
    {
    }
}

PLUGINLIB_EXPORT_CLASS(agv_controller_ns::Agv_Controller,
                       controller_interface::ControllerBase)
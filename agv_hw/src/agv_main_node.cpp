#include "agv_hw.h"

#include <iostream>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>



int main(int argc, char *argv[])
{
    ros::init(argc,argv,"agv_hw");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    agv_controller_ns::AgvInterface hw;
    int ret = hw.init(nh,nh);

    ROS_INFO("hw init ret=%d",ret);

    controller_manager::ControllerManager cm(&hw,nh);

    

    sleep(1);
    
    ros::Duration period(1.0/125);
    ROS_INFO("2joint hw run");

    while (ros::ok())
    {
        hw.read(ros::Time::now(),period);
        cm.update(ros::Time::now(),period);
        hw.write(ros::Time::now(),period);
        period.sleep();
    }


    spinner.stop();
    return 0;
}

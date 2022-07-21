#include "Test_class.h"
#include "ros/ros.h"
#include "Chassis.h"



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "node_name");

    agv_controller_ns::B b_test;
    b_test.b_test();
    agv_controller_ns::Chassis fuck();
    // ros::spin();
    // ros::shutdown();
    return 0;
}

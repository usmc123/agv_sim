#ifndef DRIVE_TEST_H 
#define DRIVE_TEST_H

#include "JAKAZuRobot.h"

#include "jkerr.h"
#include "jktypes.h"
#include <ros/ros.h>
#include "agv_sim_control_test/cmd_xyw.h"

namespace drive_test
{
    class Drive_test
    {
        public:
        Drive_test(ros::NodeHandle* nh);
        ros::Subscriber sub;
        float vx=0;
        float vy=0;
        float vz=0;
        TorqueValue tor;

        RobotStatus robot_status;
        // FTxyz tor;


        void cb(const agv_sim_control_test::cmd_xyw::ConstPtr& cmd_xyz);

        CartesianPose Cartesiancommand;

        private:
        
        


    };


}










#endif
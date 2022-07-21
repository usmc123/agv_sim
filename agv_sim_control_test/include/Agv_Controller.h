#include <ros/node_handle.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>


#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <controller_interface/multi_interface_controller.h>
#include "Chassis.h"

#include "data_struct.h"
#include "Test_class.h"


#include "agv_sim_control_test/cmd_xyw.h"

namespace agv_controller_ns
{
    class Agv_Controller : 
    public controller_interface::MultiInterfaceController
    <hardware_interface::EffortJointInterface,
    hardware_interface::PositionJointInterface>
    {
    private:
        /* data */
        std::vector<hardware_interface::JointHandle> joint_handles_;
        std::vector<hardware_interface::JointHandle> arm_joint_handles_;
        std::vector<double> turn_joint_init_pos_;
        std::vector<double> wheel_joint_init_pos_;
        std::vector<joint_state> wheel_joint_state_;
        std::vector<joint_state> turn_joint_state_;
        ros::NodeHandle nh;
        agv_sim_control_test::cmd_xyw cmd_xyz_;
        
    Test_class* test;
    Chassis *agv_chassis;
    B* b_test;

    ros::Subscriber sub;

    public:
    void doKeyctrl(const agv_sim_control_test::cmd_xyw::ConstPtr& cmd_xyz);
    
     Agv_Controller();
    bool init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh);
    void starting(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);
    };
    
}
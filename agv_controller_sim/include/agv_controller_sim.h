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
#include <realtime_tools/realtime_buffer.h>



#include <controller_interface/multi_interface_controller.h>


#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <data_struct.h>



namespace agv_controller_ns
{
    class agv_controller_sim: 
    public controller_interface::MultiInterfaceController
    <hardware_interface::EffortJointInterface,
    hardware_interface::PositionJointInterface,
    hardware_interface::JointStateInterface>
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
    public:
        agv_controller_sim(/* args */);
        ~agv_controller_sim();

        bool init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh);
    void starting(const ros::Time&);
    void update(const ros::Time&, const ros::Duration& period);
        
    };
    

    


}
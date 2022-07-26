#include "drive_test.h"


#define TOR_CONTROL


namespace drive_test
{



Drive_test::Drive_test(ros::NodeHandle* nh)
{
    
    this->sub = nh->subscribe<agv_sim_control_test::cmd_xyw>("/cmd_xyw",1,&drive_test::Drive_test::cb,this,ros::TransportHints().reliable().tcpNoDelay());
    this->Cartesiancommand.tran.x=0;
    this->Cartesiancommand.tran.y =0;
    this->Cartesiancommand.tran.z =0;
    this->Cartesiancommand.rpy.rx=0;
    this->Cartesiancommand.rpy.ry=0;
    this->Cartesiancommand.rpy.rz=0;
}
int ret_all=0;
void drive_test::Drive_test::cb(const agv_sim_control_test::cmd_xyw::ConstPtr& cmd_xyz)
{
    this->Cartesiancommand.tran.x = cmd_xyz->vx/1.0f;
    this->Cartesiancommand.tran.y = cmd_xyz->vy/1.0f;
    this->Cartesiancommand.tran.z = cmd_xyz->vw/1.0f;
    this->tor.jTorque[3]= cmd_xyz->vx/10.0f+this->robot_status.robot_monitor_data.jointMonitorData[3].instTorq;
    this->tor.jTorque[4]= cmd_xyz->vy/10.0f+this->robot_status.robot_monitor_data.jointMonitorData[4].instTorq;

    this->tor.jTorque[5]= cmd_xyz->vw/10.0f+this->robot_status.robot_monitor_data.jointMonitorData[5].instTorq;
    //     this->tor.fx= cmd_xyz->vx*0.01f;
    // this->tor.fy= cmd_xyz->vy*0.01f;

    // this->tor.fz= cmd_xyz->vw*0.01f;
    ret_all++;
    // ROS_INFO("fuck %d",ret_all);
}









}//namespace


int main(int argc, char **argv)
{

    // setlocale(LC_ALL,"");
    ros::init(argc,argv,"jaka_drive_test");
    ros::NodeHandle n;

    JAKAZuRobot test;//创建接口
    int ret=test.login_in("192.168.1.10");//连接机器人
    if(ret==0)
    {
        ROS_INFO("连接成功");
    }else{ROS_ERROR("连接失败");}

    char vision[15];
        test.get_sdk_version(vision);
        ROS_INFO("SDKvision=%s",vision);
    // return 0;


    test.power_on();//机器人上电
    ROS_INFO("poweron");
    test.enable_robot();//机器人使能
    drive_test::Drive_test drive_test_(&n);


    for(int i=0;i<6;i++)
    {
    drive_test_.tor.jTorque[i]=0.0f;

    }
    ros::Rate rate(100);
    test.servo_move_enable(TRUE);
    #ifdef TOR_CONTROL
    test.torque_control_enable(TRUE);
    #endif
    // char logfilepath[]="/home/xjtu/learn_ws/src/jaka_drive/log/";
    // ret=test.set_SDK_filepath(logfilepath);
    
    
    // if(ret==0)
    // {ROS_INFO("设置log路径成功");}
    // else
    // {
    //     ROS_ERROR("log设置errcode=%d",ret);
    // }

    // ret=test.set_compliant_type(0,0);
    //     if(ret==0)
    // {ROS_INFO("设置力控成功");}
    // else
    // {
    //     ROS_ERROR("力控errcode=%d",ret);
    // }
    // test.enable_admittance_ctrl(1);

    TorqueValue tor_test;
    
    while(ros::ok())
    {
        
        // int ret= linear_move
        // (const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed);
        ros::spinOnce();
        // int ret= test.linear_move(&drive_test_.Cartesiancommand, INCR, TRUE, 50.0);

        // ROS_INFO("cmd_Cartesiancommand=-x=%f,y=%f,z=%f",
        // drive_test_.Cartesiancommand.tran.x,
        // drive_test_.Cartesiancommand.tran.y,
        // drive_test_.Cartesiancommand.tran.z);
   
        ret=test.get_robot_status(&drive_test_.robot_status);
        ROS_ERROR("nowTorque=joint4=%f,joint5=%f,joint6=%f-",
        drive_test_.robot_status.robot_monitor_data.jointMonitorData[3].instTorq,
        drive_test_.robot_status.robot_monitor_data.jointMonitorData[4].instTorq,
        drive_test_.robot_status.robot_monitor_data.jointMonitorData[5].instTorq);
        #ifdef TOR_CONTROL
        ROS_INFO("drive_test_.tor.jTorque=joint4=%f,joint5=%f,joint6=%f-",
        drive_test_.tor.jTorque[3],
        drive_test_.tor.jTorque[4],
        drive_test_.tor.jTorque[5]);

        // for(int i=0;i<6;i++)
        // {
        //     tor_test.jTorque[i]=drive_test_.robot_status.robot_monitor_data.jointMonitorData[i].instTorq;
        // }
        
        ret = test.torque_feedforward(tor_test, 1);
        #else
         int ret = test.servo_p(&drive_test_.Cartesiancommand,INCR);
        #endif
        // ret=test.set_compliance_condition(&drive_test_.tor);

        
        // for(int i=0;i<6;i++)
        // {
        //     // ROS_INFO("tor[%d]=%f",i,drive_test_.tor.jTorque[i]);
            
        // }
        ROS_INFO("ret=%d",ret);
        // errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num);
        rate.sleep();

    }

    #ifdef TOR_CONTROL
    
    test.torque_control_enable(FALSE);
    #endif
    test.servo_move_enable(FALSE);
    // test.enable_admittance_ctrl(0);

    test.disable_robot();
    test.power_off();

    // test.shut_down();


    return 0;
}
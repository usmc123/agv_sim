#ifndef CHASSIS_H
#define CHASSIS_H

#include "math.h"
#include "iostream"
#include "data_struct.h"
#include "vector"
#include "PID.h"
#include "ros/ros.h"


namespace agv_controller_ns
{
    // Velocity_vector_type Subtraction(Velocity_vector_type *a, Velocity_vector_type *b);//实现向量a-b
    // Velocity_vector_type add(Velocity_vector_type *a, Velocity_vector_type *b);//实现向量a+b

class Chassis
{
private:
	Wheel_control_type Wheel[4];//用于控制四个轮子运动
	Velocity_vector_type Wheel_Info[4];
    Velocity_vector_type Wheel_w_vector[4];
    
    std::vector<joint_state> *wheel_joint_state_now;
    std::vector<joint_state> *turn_joint_state_now;
	float Circle_rad;//正方形就是45°的弧度制
    float yaw_diff;
	float Chassis_yaw;
	float Gimbal_;
	int Car_Handle;
    int clientID;
    float width;
    float length;
    // float Circle_rad;//rad
    float V_speed_limit = 5;

    float R_center2wheel=0;

    float wheel_R=0.075;
    Pid pid_wheel_v2e_[4];
    Pid pid_turn_outer_[4];
    Pid pid_turn_inner_[4];
    
    /* data */
public:
	Chassis(float length,float width);

    Chassis();


    void setparam(float length,float width);
    // ~Chassis();
    // void Chassis_run();
	// void Chassis_GetOrientation(void);
	void Control(float x_speed, float y_speed, float w_speed);//外部接口输出速度分解,gimbal是云台偏离正方向的角度
	// float Control_direct(float x_speed, float y_speed, float w_speed);//直接输入三项速度，返回得到云台与底盘夹角作为setpoint
	// void Circle_Control(float w_speed);
    Velocity_vector_type vector_add(Velocity_vector_type *a, Velocity_vector_type *b);
    void vector2motorCMD(Wheel_control_type *Wheel);
    void set_wheels_v_feedback(std::vector<joint_state> *wheel_joint_state);
    void set_turn_feedback(std::vector<joint_state> *turn_joint_state_ptr);
    Wheel_control_type* get_cmd_motor();

};

}


#endif


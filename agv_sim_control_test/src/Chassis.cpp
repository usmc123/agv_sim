#include "Chassis.h"
#include "PID.h"
// #include <pluginlib/class_list_macros.h>

namespace agv_controller_ns
{
    //获取底盘位姿
float AngleFloats[3];
float PointFloats[3];
char StrTemp[20] = { 0 };
extern int move_flag;


Chassis::Chassis(float length,float width)
{
    Circle_rad=atan(length / width);
    // ROS_ERROR("deg=%f\t", (Circle_rad*Rad_to_deg));//没问题

    //初始化旋转角度
    Wheel_w_vector[0].yaw=Circle_rad;
    Wheel_w_vector[1].yaw=-Circle_rad;
    Wheel_w_vector[2].yaw=Circle_rad;
    Wheel_w_vector[3].yaw=-Circle_rad;

    R_center2wheel=sqrt(pow((length/2),2)+pow((width/2),2));
    for (uint8_t i = 0; i < 4; i++)
    {
        // pid_wheel_v2e_[i].Pid_init(500,0.0001,0.01,1000,2000);

        pid_wheel_v2e_[i].Pid_init(1,0.0,0.0,1000,2000);
        pid_turn_outer_[i].Pid_init(5, 0.0, 0.0, 3,3.14);
        pid_turn_inner_[i].Pid_init(0.5, 0.0, 0.0 ,10,15);
    }
}

Chassis::Chassis()
{

    Circle_rad=atan(1 / 1);
    ROS_ERROR("atan:%f\t", Circle_rad);

    //初始化旋转角度
    Wheel_w_vector[0].yaw=Circle_rad;
    Wheel_w_vector[1].yaw=-Circle_rad;
    Wheel_w_vector[2].yaw=Circle_rad;
    Wheel_w_vector[3].yaw=-Circle_rad;
    R_center2wheel=sqrt(pow((length/2),2)+pow((width/2),2));
}


void Chassis::Control(float x_speed, float y_speed, float w_speed)
{
    float car_V=sqrt(x_speed * x_speed + y_speed * y_speed);
    if (car_V > V_speed_limit) car_V = V_speed_limit;
    float car_yaw=0;
    car_yaw = atan2(y_speed, x_speed);//atan2前面除以后面,算出来是弧度值

    //   ROS_INFO("car_yaw=%f",(car_yaw*Rad_to_deg));
    
    
    //将vx和vy的速度计算出角度角度最小角度计算
    if((car_yaw*Rad_to_deg)>90)
    {car_yaw-=180/Rad_to_deg;
    car_V=-car_V;
    }
    if ((car_yaw*Rad_to_deg)<-90)
    {
        car_yaw+=180/Rad_to_deg;
        car_V=-car_V;
    }
    //  ROS_INFO("fuck---car_yaw=%f",(car_yaw*Rad_to_deg));

    for (uint8_t i = 0; i < 4; i++)
    {
        Wheel[i].Velocity_vector.yaw=car_yaw;
        Wheel[i].Velocity_vector.v=car_V;
    }

    //因为要保持俯视图视角一致性,所以没有把角度直接变为转向电机的角度
    
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     Wheel_w_vector[i].v=w_speed*R_center2wheel;
    // }
    float temp_w_speed=w_speed*R_center2wheel;
    Wheel_w_vector[0].v=temp_w_speed;
    Wheel_w_vector[1].v=-temp_w_speed;
    Wheel_w_vector[2].v=-temp_w_speed;
    Wheel_w_vector[3].v=temp_w_speed;
    //计算完了自旋向量的每个轮子的方向和速度,这里是俯视图,还没有转换为电机的真实速度,

    for (int i = 0; i < 4; i++) 
    {
        Wheel[i].Velocity_vector = vector_add(&(Wheel[i].Velocity_vector), &(Wheel_w_vector[i]));
        // ROS_WARN("Wheel[%d]turn = %f\n",i,Wheel[0].Velocity_vector.v);
    }//向量相加
    vector2motorCMD(Wheel);
    

    int err_all=0;
    for (uint8_t i = 0; i < 4; i++)
    {
        err_all+=fabs(pid_turn_outer_[i].get_err());
        // ROS_ERROR("pid_turn_outer_%d=%f",i,err_all);
    }
    float k_temp=0;
    if(err_all<0.01)
    {
        k_temp=1;
    }else 
    {
        k_temp=0;
    }
    // ROS_INFO("k_temp=%f",k_temp);
    
    

    // ROS_ERROR("Wheel0turn = %f\n",Wheel[0].Velocity_vector.yaw);
    for (uint8_t i = 0; i < 4; i++)
    {
        // ROS_ERROR("Wheel[%d]turn = %f\n",i,Wheel[0].Velocity_vector.v);
        // ROS_ERROR("wheel_joint_state_now->at(%d).velocity= %f\n",i,wheel_joint_state_now->at(i).velocity);
        Wheel[i].motor_wheel=k_temp*pid_wheel_v2e_[i].pid_cal(Wheel[i].Velocity_vector.v,wheel_joint_state_now->at(i).velocity);
        // ROS_ERROR("pid_wheel_v2e_%d err= %f,pout=%f,iout=%f,dout=%f,out=%f",
        // i,pid_wheel_v2e_[i].get_err(),pid_wheel_v2e_[i].get_pout(),pid_wheel_v2e_[i].get_iout(),pid_wheel_v2e_[i].get_dout(),pid_wheel_v2e_[i].get_out());
        

        // Wheel[i].motor_turn=Wheel[i].Velocity_vector.yaw;//位置控制,直接赋值

        pid_turn_outer_[i].pid_cal(Wheel[i].Velocity_vector.yaw,turn_joint_state_now->at(i).position);
        Wheel[i].motor_turn=pid_turn_inner_[i].pid_cal(pid_turn_outer_[i].get_out(),turn_joint_state_now->at(i).velocity);
        // ROS_INFO("turn_joint_state_now->at(i).position=%f",turn_joint_state_now->at(i).position);
        // ROS_INFO("Wheel[i].Velocity_vector.yaw=%f",Wheel[i].Velocity_vector.yaw);
        // ROS_INFO("pidoutter%d_out=%f,,,pidiner%d_out=%f turn%d =%f",i,pid_turn_outer_[i].get_out(),i,pid_turn_inner_[i].get_out(),i,Wheel[i].motor_turn);
    }
}


//Wheel_control_type
void Chassis::vector2motorCMD(Wheel_control_type *Wheel)
{   
    Wheel[0].Velocity_vector.v=-Wheel[0].Velocity_vector.v/wheel_R;//m/s转化为rad/s
    //第一个轮子正方向速度和电机正方向反着
    Wheel[1].Velocity_vector.v=Wheel[1].Velocity_vector.v/wheel_R;
    Wheel[2].Velocity_vector.v=Wheel[2].Velocity_vector.v/wheel_R;
    Wheel[3].Velocity_vector.v=-Wheel[3].Velocity_vector.v/wheel_R;
    //转化为了行走电机的速度
    for (uint8_t i = 0; i < 4; i++)
    {
        Wheel[i].Velocity_vector.yaw=-Wheel[i].Velocity_vector.yaw;
    }
    // for (size_t i = 0; i < 4; i++)
    // {
    //     ROS_WARN("Wheel%d=%f",i,Wheel[i].Velocity_vector.yaw);
    // }
}




Velocity_vector_type Chassis::vector_add(Velocity_vector_type *a, Velocity_vector_type *b)//实现向量a+b
{
 
	Velocity_vector_type temp;
	float x = a->v * cos(a->yaw) + b->v * cos(b->yaw);
	float y = a->v * sin(a->yaw) + b->v * sin(b->yaw);
	temp.v = sqrt(x * x + y * y);
	temp.yaw = atan2(y, x);
	//printf("x:%.5f,y:%.5f,yaw = :%.5f\n", x,y,R2DEG(temp.yaw));
	return temp;
}

void Chassis::set_wheels_v_feedback(std::vector<joint_state> *wheel_joint_state_ptr){

   this->wheel_joint_state_now=wheel_joint_state_ptr;
    
}

void Chassis::set_turn_feedback(std::vector<joint_state> *turn_joint_state_ptr){

   this->turn_joint_state_now=turn_joint_state_ptr;
    
}

void Chassis::setparam(float length,float width)
{
    Circle_rad=atan(length / width);
    ROS_ERROR("atan:%f\t", Circle_rad);

    //初始化旋转角度
    Wheel_w_vector[0].yaw=Circle_rad;
    Wheel_w_vector[1].yaw=-Circle_rad;
    Wheel_w_vector[2].yaw=Circle_rad;
    Wheel_w_vector[3].yaw=-Circle_rad;

    R_center2wheel=sqrt(pow((length/2),2)+pow((width/2),2));
}

Wheel_control_type* Chassis::get_cmd_motor()
{
    return this->Wheel;
}








}
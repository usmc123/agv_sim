#include"motor_func.h"


//1圈为32768


namespace agv_controller_ns
{
TT_motor::TT_motor()
{
    ;
}


TT_motor::~TT_motor()
{
    ;
}

void TT_motor::ecode2pos(const int32_t* ecode,double* pos)//弧度
{
    *pos = ((double)(*ecode)/TT_motorEcode)*2*3.1415926;
}

void TT_motor::pos2ecode(const double* pos,int32_t* ecode)
{
    *ecode = (int32_t)(*pos/(2*3.1415926)*TT_motorEcode);
}

void TT_motor::set_position_now(int32_t ecode)
{
    this->ecode2pos(&ecode,&this->position_now);
}

void TT_motor::set_vel_now(int32_t vel_)
{
    this->intvel2vel(&vel_,&this->vel_now);
}

void TT_motor::intvel2vel(const int32_t* vel_int,double* vel_now)
{
    *vel_now = (((double)(*vel_int))/0.1)*2*3.1415926/60.0;//转/min -》 rad/s
}


double* TT_motor::get_tor_state_ptr()
{
    return &this->tor_now;
}
double* TT_motor::get_position_state_ptr()
{
    return &this->position_now;
}

double* TT_motor::get_vel_state_ptr()
{
    return &this->vel_now;
}

double* TT_motor::get_cmd_position_ptr()
{
    return &this->position_d;
}


}

#include "ZL_motor.h"

namespace agv_controller_ns
{

ZL_motor::ZL_motor()
{

}



ZL_motor::~ZL_motor()
{

}
double* ZL_motor::get_cmd_tor_ptr()
{
    return &this->tor_d;
}
double* ZL_motor::get_cmd_vel_ptr()
{
    return &this->vel_d;
}


double* ZL_motor::get_position_state_ptr()
{
    return &this->position_now;
}

double* ZL_motor::get_tor_state_ptr()
{
    return &this->tor_now;
}
double* ZL_motor::get_vel_state_ptr()
{
    return &this->vel_now;
}
double* ZL_motor::get_cmd_position_ptr()
{
    return &this->position_d;
}

void ZL_motor::ecode2pos(const int32_t* ecode,double* pos)
{
    *pos=((double)(*ecode))/4096.0 *2*3.1415926;
}

void ZL_motor::set_position_now(int32_t ecode)
{
    ecode2pos(&ecode,&this->position_now);
}

void ZL_motor::set_vel_now(int32_t vel)
{
    this->vel_now=((double)vel)*0.1*2*3.1415926/60.0;
}


}
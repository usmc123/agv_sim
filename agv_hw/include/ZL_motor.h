#ifndef MOTOR_ZL_H
#define MOTOR_ZL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


namespace agv_controller_ns
{

class ZL_motor
{

public:
    ZL_motor(/* args */);
    ~ZL_motor();

void ecode2pos(const int32_t* ecode,double* pos);
void pos2ecode(const double* pos,int32_t* ecode);
void set_position_now(int32_t ecode);
double * get_position_state_ptr();
void intvel2vel(const int32_t* int_vel,double *vel);
void vel2intvel(const double *vel,int32_t *int_vel);

void set_vel_now(int32_t vel);
double* get_vel_state_ptr();
double* get_tor_state_ptr();
double* get_cmd_position_ptr();
double* get_cmd_vel_ptr();
double* get_cmd_tor_ptr();
private:
    /* data */

    double position_now;
    double vel_now;//rpm
    double tor_now;
    double position_d;
    double vel_d;
    double tor_d;

    int32_t ecode_now;
    int32_t ecode_d;

};

}



#endif
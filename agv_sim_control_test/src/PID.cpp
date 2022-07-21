#include "PID.h"
#include <ros/ros.h>

namespace agv_controller_ns
{
    Pid::Pid(float kp,float ki,float kd,float maxi_out, float max_out)
    {
        this->kp=kp;
        this->ki=ki;
        this->kd=kd;
        this->maxi_out=maxi_out;
        this->max_out=max_out;
        p_out=0;
        i_out=0;
        d_out=0;

    }
    Pid::Pid()
    {
        p_out=0;
        i_out=0;
        d_out=0;
    }
    float Pid::pid_cal(float target,float feedback)
    {
        err=(target-feedback);
        p_out=kp*err;
        i_out+=ki*err;
        if (fabs(i_out)>maxi_out)
        {
            i_out=maxi_out*(i_out>0?1:-1);
        }
        
        // (i_out>maxi_out) ? (i_out=maxi_out):i_out;
        d_out=kd*(err-last_err);
        last_err=err;


        out=p_out+i_out+d_out;
        // out>max_out?out=max_out:out;

        if (fabs(out)>max_out)
        {
            out=max_out*(out>0?1:-1);
        }

        // ROS_ERROR("out = %f",out);
        

        
        
        return out;
    }
    void Pid::Pid_init(float kp,float ki,float kd,float maxi_out, float max_out)
    {
        this->kp=kp;
        this->ki=ki;
        this->kd=kd;
        this->maxi_out=maxi_out;
        this->max_out=max_out;
        p_out=0;
        i_out=0;
        d_out=0;
    }
    float Pid::get_out()
    {
        return this->out;
    }
    float Pid::get_pout()
    {
        return this->p_out;
    }
    float Pid::get_err()
    {
        return this->err;
    }
    float Pid::get_iout()
    {
        return this->i_out;
    }
    float Pid::get_dout()
    {
        return this->d_out;
    }
    


}
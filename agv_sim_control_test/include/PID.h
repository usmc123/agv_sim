

#ifndef PID_H
#define PID_H
#include "iostream"



namespace agv_controller_ns
{
    class Pid
    {

    public:
        Pid();
        Pid(float kp,float ki,float kd,float maxi_out, float max_out);
        void Pid_init(float kp,float ki,float kd,float maxi_out, float max_out);
        float pid_cal(float target,float feedback);
        float get_out();
        float get_pout();
        float get_err();
        float get_iout();
        float get_dout();
    private:
        
        float kp,ki,kd,maxi_out,max_out;
        float last_out,last_err;
        float p_out,i_out,d_out,out;
        float err;
    };


}


#endif



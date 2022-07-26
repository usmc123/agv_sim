#include "Keyboard_control.h"
#include <ros/ros.h>



namespace agv_controller_ns
{
    int Keyboard_control::return_key_ask()
    {
        tcgetattr(0,&initial_settings);
        new_settings = initial_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0,&initial_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);
        this->in = getchar();
        tcsetattr(0,TCSANOW,&initial_settings);
        return in;
    }
    Keyboard_control::Keyboard_control()
    {;}
        Keyboard_control::~Keyboard_control()
    {;}






}

int main(int argc,char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle n;
    agv_controller_ns::Keyboard_control keyboard_control;
    agv_sim_control_test::cmd_xyw cmd_xyw;

    ros::Publisher pub = n.advertise<agv_sim_control_test::cmd_xyw>("/cmd_xyw",10);


    while (ros::ok())
    {
        // int key_ask = keyboard_control.return_key_ask();
        int key_ask = keyboard_control.get_keyboard_press_key();
        
        switch (key_ask)
        {
        case 119://w
            cmd_xyw.vx=1.0;
            break;
        case 115://s
            cmd_xyw.vx=-1.0;
            break;
        case 100://d
            cmd_xyw.vy=-1.0;
            break;
        case 97://a
            cmd_xyw.vy=1.0;
            break;
        case 113:
            cmd_xyw.vw=1;
        break;
        case 101:
            cmd_xyw.vw=-1;
            break;
            
        default:
        cmd_xyw.vw=0;
        cmd_xyw.vx=0;
        cmd_xyw.vy=0;
            break;
        }

        pub.publish(cmd_xyw);


        if (key_ask==3)
        {
            return 0;
            
        }
        

        printf("press the key is %d\n",key_ask);
        // printf("press the key is %d\n",key_ask);

    }

    return 0;
}
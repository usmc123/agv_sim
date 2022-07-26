#include <stdio.h>
#include <stdlib.h>
 #include <unistd.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <termios.h>  /*PPSIX 终端控制定义*/ 
#include <errno.h> /*错误号定义*/ 
#include "comm_service.h"



//宏定义
#define FALSE -1
#define TRUE 0


int main(int argc, char **argv)
{
    
    ros::init(argc, argv,"DJI_dbus_RX");
    ros::NodeHandle n;
    setlocale(LC_ALL,"");
    char buf[100];

    agv_sim_control_test::cmd_xyw cmd_xyw;

    ros::Publisher pub = n.advertise<agv_sim_control_test::cmd_xyw>("/cmd_xyw",1);
    
    
    dst_ccms_api::comm_service Usart_485;
    
    
    // int comm_service::CommInit(int speed, int flow_ctrl, int databits, int stopbits, int parity)
    char port[]="/dev/ttyS0";
    Usart_485.CommOpen(port);

    Usart_485.CommInit(115200,0,8,1,'N');

    

    ros::Rate rate(100);

    struct Command_Sbus cmd_sbus;
    bool send_flag=false;
    
    while (ros::ok())
    {
        /* code */
        uint8_t index=0;
        int ret=Usart_485.CommRecv(buf,1);
        index++;
        

            if ((uint8_t)buf[0]==0xA5)
            {
                ret=Usart_485.CommRecv(&buf[index],3);
                index+=3;
                if( Usart_485.Verify_CRC8_Check_Sum((uint8_t *)buf,4) )
                {

                    
                    uint16_t* len=(uint16_t*)(&buf[1]);

                    
                    ret=Usart_485.CommRecv(buf+index,2);

                    switch (*(uint16_t*)(buf+index))
                    {
 
                    case CMD_SBUS:
                        index+=2;
                         ret=Usart_485.CommRecv(buf+index,(*len)+2);
                        //  ROS_ERROR("fuck%d",(*len)+2);
                    
                        // for(int i=0;i<(*len)+8;i++)
                        // {
                        //     ROS_WARN("buf[%d]=%2x",i,(uint8_t)buf[i]);
                        // }

                        if (Usart_485.Verify_CRC16_Check_Sum((uint8_t *)buf,(*len)+8))
                        {
                            memcpy((void *)&cmd_sbus,(void *)(buf+index),(*len));
                            // ROS_WARN("mode=%d",*((uint8_t*)(buf+index)));
                            // ROS_WARN("x=%d",*((int32_t*)(buf+index+1)));
                            // ROS_WARN("y=%d",*((int32_t*)(buf+index+1+4)));
                            // ROS_WARN("z=%d",*((int32_t*)(buf+index+1+4+4)));



                            //  ROS_INFO("cmd_sbus:vx=%d,vy=%d,vw=%d\n",cmd_sbus.vx,cmd_sbus.vy,cmd_sbus.vw);
                            send_flag=true;
                        }
                        
                        break;
                    
                    default:
                        break;
                    }
                    
                }
            }
            

        //串口接收和赋值

        //
        if (send_flag==true && cmd_sbus.mode==MODE_NORMAL)
        {
     
            cmd_xyw.vx=cmd_sbus.vy/600.0f;
            cmd_xyw.vy=-cmd_sbus.vx/600.0f;
            cmd_xyw.vw=-cmd_sbus.vw/600.0f;
            pub.publish(cmd_xyw);
        }
        
        
        //  rate.sleep();
    }

    Usart_485.CommClose();
    
    


    
    return 0;
}


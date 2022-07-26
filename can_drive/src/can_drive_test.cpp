#include "can_drive_test.h"


#define STDDEV "can0"

unsigned long usleeptime = 1000;	/* 1000 ms */


// ls -l /dev/ttyUSB0



Can_drive_test::Can_drive_test(char* port)
{
    this->port = port;

    // fd = open(port, O_RDWR|O_NONBLOCK);
    fd = open(this->port, O_RDWR);

    ROS_INFO("openfd  = %d",fd);
    this->showCANStat(fd);

    Config_par_t  cfg;
    volatile Command_par_t cmd;

    cmd.cmd = CMD_STOP;
    ioctl(fd, CAN_IOCTL_COMMAND, &cmd);

    cfg.target = CONF_TIMING; 
    cfg.val1   = 1000;
    
    ioctl(fd, CAN_IOCTL_CONFIG, &cfg);

    cmd.cmd = CMD_START;
    ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    this->showCANStat(fd);
    this->turn_pos.resize(4);
    this->turn_tor.resize(4);
    this->turn_vel.resize(4);
    
    this->wheel_pos.resize(4);
    this->wheel_tor.resize(4);
    this->wheel_vel.resize(4);

}

Can_drive_test::~Can_drive_test()
{
    close(fd);
}

int Can_drive_test::send_msgs(int id ,uint8_t *buf)
{
    canmsg_t txmsg;
    txmsg.id=id;
    txmsg.flags = 0;
    txmsg.length = 8;
    memcpy((void*)txmsg.data,(void *)buf,8);


    int ret = write(fd, &txmsg, 1);

    if (ret == -1) 
        {
             perror("write error---");
		}
    else if (ret == 0) 
        {
            printf("transmit timed out\n");
		}

        return ret;
    
}


void Can_drive_test::showCANStat(int fd)
{
   CanStatusPar_t status;
   char *m;  
   ioctl(fd, CAN_IOCTL_STATUS, &status);
   switch(status.type) 
   {
      case  CAN_TYPE_SJA1000:
         m = "sja1000";
         break;
      default:
         m = "unknown";
         break;
   }

   printf(":: %s %4d %2d %2d %2d %2d %2d tx:%3d/%3d: rx:%3d/%3d:\n",
      m,
      status.baud,
      status.status,
      status.error_warning_limit,
      status.rx_errors,
      status.tx_errors,
      status.error_code,
      status.tx_buffer_size,
      status.tx_buffer_used,
      status.rx_buffer_size,
      status.rx_buffer_used);
}

int main(int argc,char** argv)
{
    char device[40];
    sprintf(device, "/dev/%s", STDDEV);
    
    
    ros::init(argc,argv,"can_test");
    ros::NodeHandle nh;


    Can_drive_test can(device);
    ros::Rate rate(1000);

    uint8_t buf[8];
    for (uint8_t i = 0; i < 8; i++)
    {
        buf[i]=i;
    }
    int fuck=1000;
    while(fuck--)
    {
        can.send_msgs(0x0601,buf);
        rate.sleep();
    }

    can.~Can_drive_test();
    return 0;
}

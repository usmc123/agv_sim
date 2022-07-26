#include "can_drive_motor.h"


#define STDDEV "can0"

unsigned long usleeptime = 1000;	/* 1000 ms */


// ls -l /dev/ttyUSB0


void Can_drive_motor::motor_init()
{

    
    ros::Duration time;
    time.nsec=100000000;
    uint8_t buf[8];
    buf[0]=0x10;
    buf[1]=0x10;
    int ret = 0;  
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(i+1,buf,2);
        time.sleep();
    }
    

    
    
        //开启所有电机的TPDO
    buf[0]=0x01;
    buf[1]=0x00;
    ret = send_msgs(0,buf,2);

    load_can_buf(buf,0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00);//位置操作模式
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }
    
    load_can_buf(buf,0x23,0x7a,0x60,0x00,0x00,0x00,0x00,0x00);//目标位置为0
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x23,0x81,0x60,0x00,0x20,0x4e,0x00,0x00);//速度20000转
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x80,0x00,0x00,0x00);//清除报警
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//伺服准备
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//伺服等待使能
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//立即更新命令1 更新位置点时也要发送这个命令
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x3f,0x00,0x00,0x00);//立即更新命令2 更新位置点时也要发送这个命令
    for (int i = 0; i < 4; i++)//转向电机初始化
    {
        ret = send_msgs(0x601+i,buf);
    }

    //上面是天太电机的初始化程序（转向电机）


    //下面是行走电机的初始化程序

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//初始化
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }
    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//初始化
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//初始化
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2b,0x0f,0x20,0x00,0x00,0x00,0x00,0x00);//设置异步模式
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2f,0x60,0x60,0x00,0x04,0x00,0x00,0x00);//设置电流模式
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2b,0x30,0x20,0x08,0x00,0x00,0x00,0x00);//右电机开抱闸
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

    load_can_buf(buf,0x2b,0x30,0x20,0x07,0x00,0x00,0x00,0x00);//左电机开抱闸
    for (int i = 0; i < 2; i++)//转向电机初始化
    {
        ret = send_msgs(0x605+i,buf);
    }

}

Can_drive_motor::Can_drive_motor(char* port)
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

    this->tt_motor_.resize(4);
    this->zl_motor_.resize(4);
}

Can_drive_motor::~Can_drive_motor()
{
    close(fd);
}

int Can_drive_motor::send_msgs(int id ,uint8_t *buf)
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

int Can_drive_motor::send_msgs(int id ,uint8_t *buf ,int num)
{
    canmsg_t txmsg;
    txmsg.id=id;
    txmsg.flags = 0;
    txmsg.length = num;
    memcpy((void*)txmsg.data,(void *)buf,num);


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

void Can_drive_motor::load_can_buf(uint8_t *buf,const uint8_t buf0,
const uint8_t buf1,
const uint8_t buf2,
const uint8_t buf3,
const uint8_t buf4,
const uint8_t buf5,
const uint8_t buf6,
const uint8_t buf7)
{
    *(buf)=buf0;
    *(buf+1)=buf1;
    *(buf+2)=buf2;
    *(buf+3)=buf3;
    *(buf+4)=buf4;
    *(buf+5)=buf5;
    *(buf+6)=buf6;
    *(buf+7)=buf7;
}


void Can_drive_motor::showCANStat(int fd)
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


void Can_drive_motor::thread_read()
{
    int ret = 0;
    canmsg_t rxmsg;
    while(ros::ok())
    {
        ret = read(fd,&rxmsg,1);
        #ifdef SHOW_ORIGIN_CAN_TX
        ROS_INFO("ret = %d rcvid = %4x rcv msg = %2x %2x %2x %2x %2x %2x %2x %2x",ret,rxmsg.id,rxmsg.data[0],rxmsg.data[1],rxmsg.data[2],rxmsg.data[3],rxmsg.data[4],rxmsg.data[5],rxmsg.data[6],rxmsg.data[7]);
        #endif
        int ecode_tmp,vel_tmp=0;
        switch (rxmsg.id)
        {
        case 0x181://turn1

            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->tt_motor_.at(0).set_position_now(ecode_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->tt_motor_.at(0).set_vel_now(vel_tmp);
            break;
        case 0x182://turn2
            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->tt_motor_.at(1).set_position_now(ecode_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->tt_motor_.at(1).set_vel_now(vel_tmp);
            break;
        case 0x183://turn3
            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->tt_motor_.at(2).set_position_now(ecode_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->tt_motor_.at(2).set_vel_now(vel_tmp);
            break;
        case 0x184://turn4
            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->tt_motor_.at(3).set_position_now(ecode_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->tt_motor_.at(3).set_vel_now(vel_tmp);
            break;
        case 0x185://wheel1 2
        //后面4位为1号
            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->zl_motor_.at(1).set_position_now(ecode_tmp);
            ecode_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->zl_motor_.at(0).set_position_now(ecode_tmp);
            break;
        case 0x285://wheel1 2
        //后面4位为1号
            vel_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->zl_motor_.at(1).set_vel_now(vel_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->zl_motor_.at(0).set_vel_now(vel_tmp);
            break;
        case 0x186://wheel3 4
            ecode_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->zl_motor_.at(2).set_position_now(ecode_tmp);
            ecode_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->zl_motor_.at(3).set_position_now(ecode_tmp);
            break;
        case 0x286://wheel3 4
        //后面4位为1号
            vel_tmp = (rxmsg.data[3]<<24) | (rxmsg.data[2]<<16) | (rxmsg.data[1]<<8) | (rxmsg.data[0]);
            this->zl_motor_.at(2).set_vel_now(vel_tmp);
            vel_tmp = (rxmsg.data[3+4]<<24) | (rxmsg.data[2+4]<<16) | (rxmsg.data[1+4]<<8) | (rxmsg.data[0+4]);
            this->zl_motor_.at(3).set_vel_now(vel_tmp);
        default:
            break;
        }
    }
}
int Can_drive_motor::set_TT_motor_pos(int motor_id)//rad
{   
    //地址返回的是低位字节
    uint8_t buf[8];
    int32_t ecode;
    // ROS_ERROR("set_TT_motor_pos motor_id=%d",motor_id);
    this->tt_motor_[motor_id].pos2ecode(this->tt_motor_.at(motor_id).get_cmd_position_ptr(),&ecode);
    buf[0]=0x23;
    buf[1]=0x7a;
    buf[2]=0x60;
    buf[3]=0x00;

    buf[7]=*((uint8_t*)(&ecode));
    buf[6]=*((uint8_t*)(&ecode)+1);
    buf[5]=*((uint8_t*)(&ecode)+2);
    buf[4]=*((uint8_t*)(&ecode)+3);

    send_msgs(motor_id+0x601,buf);
    return 0;
}

int Can_drive_motor::set_ZL_motor_tor()
{
    uint8_t buf[8];
    int32_t ecode;
    int tor_tmp = (int) (*this->zl_motor_.at(0).get_cmd_tor_ptr());
    // ROS_ERROR("tor_tmp = %d",tor_tmp);
    buf[0]=0x2b;
    buf[1]=0x71;
    buf[2]=0x60;
    buf[3]=0x02;//右侧电机 1号电机
    buf[7]=*((uint8_t*)(&tor_tmp));
    buf[6]=*((uint8_t*)(&tor_tmp)+1);
    buf[5]=*((uint8_t*)(&tor_tmp)+2);
    buf[4]=*((uint8_t*)(&tor_tmp)+3);
    send_msgs(0x605,buf);//1号电机,右
    
    tor_tmp = (int) (*this->zl_motor_.at(1).get_cmd_tor_ptr());
    buf[3]=0x01;//左侧电机 2号电机
    buf[7]=*((uint8_t*)(&tor_tmp));
    buf[6]=*((uint8_t*)(&tor_tmp)+1);
    buf[5]=*((uint8_t*)(&tor_tmp)+2);
    buf[4]=*((uint8_t*)(&tor_tmp)+3);
    send_msgs(0x605,buf);//2号电机,左

    tor_tmp = (int) (*this->zl_motor_.at(2).get_cmd_tor_ptr());
    buf[3]=0x01;//左侧电机 3号电机
    buf[7]=*((uint8_t*)(&tor_tmp));
    buf[6]=*((uint8_t*)(&tor_tmp)+1);
    buf[5]=*((uint8_t*)(&tor_tmp)+2);
    buf[4]=*((uint8_t*)(&tor_tmp)+3);
    send_msgs(0x606,buf);//3号电机,左

    tor_tmp = (int) (*this->zl_motor_.at(3).get_cmd_tor_ptr());
    buf[3]=0x02;//右侧电机 4号电机
    buf[7]=*((uint8_t*)(&tor_tmp));
    buf[6]=*((uint8_t*)(&tor_tmp)+1);
    buf[5]=*((uint8_t*)(&tor_tmp)+2);
    buf[4]=*((uint8_t*)(&tor_tmp)+3);
    send_msgs(0x606,buf);//4号电机,右

    return 0;

}

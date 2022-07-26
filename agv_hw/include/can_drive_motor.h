#ifndef CAN_DRIVE_TEST_H
#define CAN_DRIVE_TEST_H


#include "can4linux.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <vector>
#include "motor_func.h"
#include "ZL_motor.h"

using namespace agv_controller_ns;

class Can_drive_motor
{
private:
    char *port;
    CanStatusPar_t status;
    int fd=0;



public:
    Can_drive_motor(/* args */);
    Can_drive_motor(char* port);
    int send_msgs(int id ,uint8_t *buf);
    int send_msgs(int id ,uint8_t *buf ,int num);
    int recive_msgs(canmsg_t* rxmsg);
    void showCANStat(int fd);
    ~Can_drive_motor();


    //这里的id是电机号
    int get_ZL_motor_pos(int motor_id,double* pos);
    int get_ZL_motor_vel(int motor_id,double* vel);
    int get_ZL_motor_tor(int motor_id,double* tor);

    int get_TT_motor_pos(int motor_id,double* pos);
    int get_TT_motor_vel(int motor_id,double* vel);
    int get_TT_motor_tor(int motor_id,double* tor);

    int set_TT_motor_tor(int motor_id,double* tor);
    int set_TT_motor_pos(int motor_id);
    int set_ZL_motor_tor();
    int set_ZL_motor_vel(int motor_id,double* tor);

    void thread_read();

    void motor_init();

    void load_can_buf(uint8_t *buf,const uint8_t buf0,
const uint8_t buf1,
const uint8_t buf2,
const uint8_t buf3,
const uint8_t buf4,
const uint8_t buf5,
const uint8_t buf6,
const uint8_t buf7);

    std::vector <TT_motor> tt_motor_;
    std::vector <ZL_motor> zl_motor_;

};

















#endif

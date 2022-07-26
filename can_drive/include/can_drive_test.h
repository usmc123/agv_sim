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



class Can_drive_test
{
private:
    char *port;
    CanStatusPar_t status;
    int fd=0;
    /* data */
    std::vector<double> turn_pos;
    std::vector<double> turn_vel;
    std::vector<double> turn_tor;
    
    std::vector<double> wheel_pos;
    std::vector<double> wheel_vel;
    std::vector<double> wheel_tor;



public:
    Can_drive_test(/* args */);
    Can_drive_test(char* port);
    int send_msgs(int id ,uint8_t *buf);
    int recive_msgs(canmsg_t* rxmsg);
    void showCANStat(int fd);

    ~Can_drive_test();
};

















#endif

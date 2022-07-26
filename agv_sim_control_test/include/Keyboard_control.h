#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include <termio.h>
#include <stdio.h>

#include <unistd.h>
#include <iostream>
#include "agv_sim_control_test/cmd_xyw.h"


const int KEYCODE_W = 0x77;
const int KEYCODE_A = 0x61;
const int KEYCODE_S = 0x73;
const int KEYCODE_D = 0x64;
 
const int KEYCODE_A_CAP = 0x41;
const int KEYCODE_D_CAP = 0x44;
const int KEYCODE_S_CAP = 0x53;
const int KEYCODE_W_CAP = 0x57;



namespace agv_controller_ns
{
class Keyboard_control
{
private:
    /* data */
    int in=-1;
  struct termios initial_settings, new_settings;
public:
    int return_key_ask();
    Keyboard_control();
    ~Keyboard_control();

int get_keyboard_press_key() {
    kbhit();
    return readch();
    // printf("%02x \n", readch());
  };

   void init_keyboard() {
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VEOL] = 1;
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);
  }
 
  void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }
 
  int kbhit() {
    unsigned char ch;
    int nread;
 
    if (in != -1) return 1;
    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    if (nread == 1) {
      in = ch;
      return 1;
    }
    return 0;
  }
 
  int readch() {
    char ch;
    if (in != -1) {
      ch = in;
      in = -1;
      return ch;
    }
    read(0, &ch, 1);
    return ch;
  }
};



}


#endif
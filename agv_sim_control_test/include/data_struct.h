#ifndef DATA_STRUCT
#define DATA_STRUCT

// #define DEBUG_JOINT_INFO
#define PI 3.14159265359
#define Rad_to_deg (180/PI)

typedef struct{
    float v;//m/s
    float yaw;//rad/s
   }Velocity_vector_type;

typedef struct {
float motor_wheel, motor_turn;//电机控制命令
Velocity_vector_type Velocity_vector;//速度向量
Velocity_vector_type Velocity_vector_last;//上一次速度；
}Wheel_control_type;

typedef struct{
    float velocity;
    float position;
    float effort;
}joint_state;

#pragma pack(1)

#define CMD_SBUS (0x0001)
//#define MODE_STOP 0		//停止模式，
//#define MODE_NORMAL 1 //正常模式
//#define MODE_BACK_ZERO 2;//回零模式
enum{
MODE_STOP = 0,
MODE_NORMAL,
MODE_BACK_ZERO
};
struct Command_Sbus{
	uint8_t mode;

	//目前只用了低四位，高四位还可以利用一下
	//0:MODE_STOP停止模式，
	//1:MODE_NORMAL正常模式
	//2:MODE_BACK_ZERO
	int32_t vx;
	int32_t vy;
	int32_t vw;
};//13个字节


#pragma pack()


#endif
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "struct_typedef.h"
#include "referee.h"



//���̵���ٶȻ�PID

#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


//������ת����PID
#define CHASSIS_ROTATION_PID_KP 0.0f
#define CHASSIS_ROTATION_PID_KI 0.0f
#define CHASSIS_ROTATION_PID_KD 0.0f
#define CHASSIS_ROTATION_PID_MAX_OUT 0.0f
#define CHASSIS_ROTATION_PID_MAX_IOUT 0.0f



//������ת����PID
#define CHASSIS_ANGLE_PID_KP 40.0f
#define CHASSIS_ANGLE_PID_KI 0.0f
#define CHASSIS_ANGLE_PID_KD 0.0f
#define CHASSIS_ANGLE_PID_MAX_OUT 6.0f
#define CHASSIS_ANGLE_PID_MAX_IOUT 0.2f

typedef struct{
float x;
float y;
float w;
}location_t;

typedef enum{
	MOVE_CMD,
}AUTO_CMD;

typedef struct{
	AUTO_CMD cmd;
	}auto_pack_t;
#endif

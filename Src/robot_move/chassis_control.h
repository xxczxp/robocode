
#define AUTO_RECIVE_BUFFER_SIZE 30

#define X_PASS_LIMIT 0.05
#define Y_PASS_LIMIT 0.05
#define WZ_PASS_LIMIT 0.05

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "struct_typedef.h"
#include "referee.h"



//底盘电机速度环PID

#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_ROTATION_PID_KP 0.0f
#define CHASSIS_ROTATION_PID_KI 0.0f
#define CHASSIS_ROTATION_PID_KD 0.0f
#define CHASSIS_ROTATION_PID_MAX_OUT 0.0f
#define CHASSIS_ROTATION_PID_MAX_IOUT 0.0f



//底盘旋转跟随PID
#define CHASSIS_ANGLE_PID_KP 40.0f
#define CHASSIS_ANGLE_PID_KI 0.0f
#define CHASSIS_ANGLE_PID_KD 0.0f
#define CHASSIS_ANGLE_PID_MAX_OUT 6.0f
#define CHASSIS_ANGLE_PID_MAX_IOUT 0.2f

#define PI 3.1415926f

typedef struct{
float x;
float y;
float w;
}location_t;

typedef enum{
	MOVE_CMD,
	PUT_BALL_CMD
}AUTO_CMD;

typedef struct{
	int cmd;//AUTO_CMD
	int ball_num;
	int cup_num;
	int near;
	location_t target;
	}auto_pack_t;

	
extern QueueHandle_t auto_queue;
	
typedef enum{
	CMD_GET,
	MOVE,
	PUT_BALL,
	PUT_BALL_MOVE,
	STOP
}STEP_AUTO_STATE;
	
typedef enum{
	move_target,
	move_Sentry,
	release
}AUTO_INNER_STATE;





#endif

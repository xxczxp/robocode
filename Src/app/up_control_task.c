
#include "up_control_task.h"
#include "pwm.h"

#include "pid.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "Detect_Task.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 1.10537517e-4
#define OB_INDEX 0
#define PULL_INDEX 1
#define UP_MOTOR_NUM 3

PidTypeDef up_motor_speed_pid[UP_MOTOR_NUM]={PID_POSITION,M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
PidTypeDef up_motor_position_pid[UP_MOTOR_NUM]={PID_POSITION,100,0,0};

float motor_mearsure_bias[UP_MOTOR_NUM];


const motor_measure_t *motor_measure_ptr;

chassis_motor_t up_motor[UP_MOTOR_NUM];
float up_target[UP_MOTOR_NUM];

float up_motor_sign[UP_MOTOR_NUM];




steering_engine servo;

void up_init(){
	servo.port=A;
	motor_measure_ptr=get_Yaw_Gimbal_Motor_Measure_Point();

	
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	//DEBUG pid
	const static fp32 ob_position_pid[3] ={100,0,0 };
	const static fp32 pull_position_pid[3] = {100,0,0};
	
	
	for(int i=0;i<UP_MOTOR_NUM;i++){
		
		PID_Init(&up_motor_speed_pid[i],PID_POSITION,motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);
		up_motor[i].chassis_motor_measure=&motor_measure_ptr[i];
		motor_mearsure_bias[i]=up_motor[i].chassis_motor_measure->total_ecd;
	}
	
	//DEBUG pid
	PID_Init(&up_motor_position_pid[OB_INDEX],PID_POSITION,ob_position_pid,2.0f,0.2f);
	PID_Init(&up_motor_position_pid[PULL_INDEX],PID_POSITION,pull_position_pid,2.0f,0.2f);
	
}

void steer_open(void){
	change_pwm(&servo,STEER_OPEN_ANGLE);
}

void steer_close(){
	change_pwm(&servo,STEER_CLOSE_ANGLE);
}



void up_motor_speed_update(){
	for(int i=0;i<UP_MOTOR_NUM;i++){
		up_motor[i].speed = up_motor_sign[i] * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * up_motor[i].chassis_motor_measure->speed_rpm;
	}

}

void up_pid_cacu(void){
	for(int i=0;i<UP_MOTOR_NUM;i++){
		float speed=PID_Calc(up_motor_position_pid+i,up_motor[i].chassis_motor_measure->total_ecd*up_motor_sign[i]*CHASSIS_MOTOR_RPM_TO_VECTOR_SEN ,up_target[i]);
		PID_Calc(up_motor_speed_pid+i,up_motor[i].speed,up_target[i]);
	}

}


 
void up_task(void const *pvParameters){
	up_init();
	
	while(1){
		
		
		up_motor_speed_update();
		up_pid_cacu();
		
		
		if (toe_is_error(DBUSTOE))
						{
                CAN_CMD_CHASSIS(0, 0, 0, 0);
            }
            else
            {
               CAN_CMD_UP(up_motor_speed_pid[0].out,up_motor_speed_pid[1].out,0.0,0.0);
            }
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
	
	
}

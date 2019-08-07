
#include "up_control_task.h"
#include "pwm.h"

#include "pid.h"
#include "CAN_receive.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 1.10537517e-4

PidTypeDef ob_motor_speed_pid={PID_POSITION,M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
PidTypeDef ob_motor_position_pid={PID_POSITION,100,0,0};

float motor_mearsure_bias;

motor_measure_t ob_motor_measure;
const motor_measure_t *motor_measure_ptr;






steering_engine servo;

void up_init(){
	servo.port=A;
	motor_measure_ptr=get_Yaw_Gimbal_Motor_Measure_Point();
	ob_motor_measure=motor_measure_ptr[4];
	motor_mearsure_bias=ob_motor_measure.total_ecd;
}

void steer_open(void){
	change_pwm(&servo,STEER_OPEN_ANGLE);
}

void steer_close(){
	change_pwm(&servo,STEER_CLOSE_ANGLE);
}

float ob_pid_cacu(float target){
	float speed=PID_Calc(&ob_motor_position_pid,ob_motor_measure.total_ecd,target);
}

void up_task(void const *pvParameters){
	
	
	
}

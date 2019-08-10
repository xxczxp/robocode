#include "up_control_task.h"
#include "pwm.h"
#include <math.h>

#include "pid.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "Detect_Task.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 2.13081e-5
#define OB_INDEX 0
#define PULL_INDEX 1
#define UP_MOTOR_NUM 3
#define PI acos(-1)

PidTypeDef up_motor_speed_pid[UP_MOTOR_NUM]={PID_POSITION,M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
PidTypeDef up_motor_position_pid[UP_MOTOR_NUM]={PID_POSITION,100,0,0};

float motor_mearsure_bias[UP_MOTOR_NUM];

int LGBT = 0;
int task_finish = 0;

const motor_measure_t *motor_measure_ptr;

chassis_motor_t up_motor[UP_MOTOR_NUM];
float up_target[UP_MOTOR_NUM];

float up_motor_sign[UP_MOTOR_NUM];


extern auto_pack_t next_cmd;

steering_engine ball_serve;
steering_engine cup_serve;
steering_engine free_serve;

void up_init(void){
	
	up_motor_sign[0]=1;
	up_motor_sign[1]=1;
	up_motor_sign[2]=1;
	ball_serve.port = A;
	free_serve.port = B;
	cup_serve.port = C;
	motor_measure_ptr=get_Yaw_Gimbal_Motor_Measure_Point();

	
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	//DEBUG pid

	const static fp32 ob_position_pid[3] ={2,0,10 };
	const static fp32 pull_position_pid[3] = {1,0,0};

	
	
	for(int i=0;i<UP_MOTOR_NUM;i++){
		
		PID_Init(&up_motor_speed_pid[i],PID_POSITION,motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);

		up_motor[i].chassis_motor_measure=&motor_measure_ptr[i];
		motor_mearsure_bias[i]=up_motor[i].chassis_motor_measure->total_ecd;
	}
	
	//DEBUG pid
	PID_Init(&up_motor_position_pid[OB_INDEX],PID_POSITION,ob_position_pid,0.3f,0.2f);
	PID_Init(&up_motor_position_pid[PULL_INDEX],PID_POSITION,pull_position_pid,2.0f,0.2f);

	
}

void steer_open(void){
	change_pwm(&ball_serve,STEER_OPEN_ANGLE);
}

void steer_close(){
	change_pwm(&ball_serve,STEER_CLOSE_ANGLE);
}
void OPCL_task(void const *pvParameters){
	task_finish = 2;
	steer_open();
	vTaskDelay(500);
	steer_close();
	task_finish = 1;
	vTaskDelete(NULL);
}

void Timer_task(void const *pvParameters){
	LGBT = 1;
	vTaskDelay(2000);
	LGBT = 2;
	vTaskDelete(NULL);
}

void un_timer_task(void const *pvParameters){
	LGBT = 1;
	vTaskDelay(2000);
	LGBT = 0;
	vTaskDelete(NULL);
}
	
	
void cup_out_task(void const *pvParameters){
	int i = 0;
	while(i < next_cmd.cup_num){
	change_pwm(&cup_serve,45);
	vTaskDelay(100);
	change_pwm(&cup_serve,180);
	vTaskDelay(100);
	change_pwm(&cup_serve, 90);
	i++;
	}
	vTaskDelete(NULL);
}

void free_ball_task(void const *pvParameters){
	int i = 0;
	while(i < next_cmd.cup_num){
	change_pwm(&free_serve, 45);
	vTaskDelay(100);
	change_pwm(&cup_serve,90);
	i++;
	}
	vTaskDelete(NULL);
}


void up_motor_speed_update(){
	for(int i=0;i<UP_MOTOR_NUM;i++){
		up_motor[i].speed = up_motor_sign[i] * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * up_motor[i].chassis_motor_measure->speed_rpm;
	}

}

void up_pid_cacu(){
	for(int i=0;i<UP_MOTOR_NUM;i++){
		float speed=PID_Calc(up_motor_position_pid+i, up_motor[i].chassis_motor_measure->total_ecd*up_motor_sign[i]*CHASSIS_MOTOR_RPM_TO_VECTOR_SEN, up_target[i]);
		PID_Calc(up_motor_speed_pid+i,up_motor[i].speed,speed);
		
	}

}
void trans_ball_task(void const *pvParameters){
  up_target[0] += next_cmd.ball_num * PI /2;
	
}

 
void up_task(void const *pvParameters){
	up_init();	
	while(1){
		
		
		up_motor_speed_update();
		up_pid_cacu();
		
		
		if (toe_is_error(DBUSTOE))
						{
                CAN_CMD_UP(0, 0, 0, 0);
            }
            else
            {
               CAN_CMD_UP(up_motor_speed_pid[0].out,up_motor_speed_pid[1].out,0.0,0.0);
            }
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
	
	
}

#include "up_control_task.h"
#include "pwm.h"
#include <math.h>
#include "freeRTOS.h"
#include "task.h"

#include "pid.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "Detect_Task.h"


#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 2.13081e-5
#define ARG_3510 1
#define OB_INDEX 0
#define PULL_INDEX 1
#define UP_MOTOR_NUM 3

#define GET_CUP_EN 0xCF
#define PUT_CUP_EN 0xC60
//#define PI 3.1415926

PidTypeDef up_motor_speed_pid[UP_MOTOR_NUM]={PID_POSITION,M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
PidTypeDef up_motor_position_pid[UP_MOTOR_NUM]={PID_POSITION,100,0,0};

float motor_mearsure_bias[UP_MOTOR_NUM];

int LGBT = 0;
int task_finish = 0;
int cup_free = 0;

const motor_measure_t *motor_measure_ptr;

chassis_motor_t up_motor[UP_MOTOR_NUM];
float up_target[UP_MOTOR_NUM];

float up_motor_sign[UP_MOTOR_NUM];


extern auto_pack_t next_cmd;

steering_engine ball_serve;
steering_engine cup_freer;
steering_engine ball_puter;
steering_engine clip_freer;
steering_engine position_controler;
void up_init(void){
	
	up_motor_sign[0]=1*CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
	up_motor_sign[1]=1*ARG_3510;
	up_motor_sign[2]=1;
	ball_serve.port = A;
  cup_freer.port = B;
  ball_puter.port = C;
  clip_freer.port = D;
  position_controler.port = E;
	motor_measure_ptr=get_Yaw_Gimbal_Motor_Measure_Point();

	
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	const static fp32 roll_motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	//DEBUG pid

	const static fp32 ob_position_pid[3] ={2,0,10 };
	const static fp32 pull_position_pid[3] = {100,10,0};

	
	
	for(int i=0;i<UP_MOTOR_NUM;i++){
		
		PID_Init(&up_motor_speed_pid[i],PID_POSITION,motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);

		up_motor[i].chassis_motor_measure=&motor_measure_ptr[i];
		motor_mearsure_bias[i]=up_motor[i].chassis_motor_measure->total_ecd;
	}
	
	PID_Init(&up_motor_speed_pid[1],PID_POSITION,roll_motor_speed_pid,M3510_MOTOR_SPEED_PID_MAX_OUT,M3510_MOTOR_SPEED_PID_MAX_IOUT);
	
	//DEBUG pid
	PID_Init(&up_motor_position_pid[OB_INDEX],PID_POSITION,ob_position_pid,0.3f,0.2f);
	PID_Init(&up_motor_position_pid[PULL_INDEX],PID_POSITION,pull_position_pid,2500,2000);

	
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
	

void up_s_task(void const *pvParameters){
change_pwm(&position_controler,130);
vTaskDelete(NULL);
}

xTaskHandle cup_put;
xTaskHandle prepare_cup;
xTaskHandle free_cup;

void cup_prepare_task(void const *pvParameters){
  up_target[1] = PI;
	change_pwm(&position_controler,80);
	vTaskDelay(300);
	change_pwm(&cup_freer,0);
	vTaskDelay(1200);
	change_pwm(&cup_freer,90);
	change_pwm(&clip_freer, 55);
	change_pwm(&ball_puter,0);
	vTaskDelay(500);
	change_pwm(&ball_puter, 65);
	vTaskDelete(NULL);
}
	
void cup_put_task(void const *pvParameters){
	xTaskCreate((TaskFunction_t)up_s_task, "steering engine up", 128, NULL, 1, cup_put);
	up_target[1] = PI/2;
	change_pwm(&clip_freer, 120);
	vTaskDelete(NULL);
}

void cup_free_task(int* num){
	while(num>0){
xTaskCreate((TaskFunction_t)cup_prepare_task, "cup prepare", 128, NULL, 1 ,prepare_cup);
xTaskCreate((TaskFunction_t)cup_put_task, "cup  free", 128, NULL, 1, free_cup);
num--;
	}
	cup_free = 1;
	vTaskDelete(NULL);
}


int last_3510_en=0;

void up_motor_speed_update(){
	for(int i=0;i<UP_MOTOR_NUM;i++){
		up_motor[i].speed = up_motor_sign[i]  * up_motor[i].chassis_motor_measure->speed_rpm;
	}
	
	up_motor[1].speed=(up_motor[1].chassis_motor_measure->total_ecd-last_3510_en)/UP_CONTROL_TIME_MS;
	last_3510_en=up_motor[1].chassis_motor_measure->total_ecd;

	

}

void up_pid_cacu(){
	
		float speed;
	
	for(int i=0;i<UP_MOTOR_NUM;i++){
	speed=PID_Calc(up_motor_position_pid+i, up_motor[i].chassis_motor_measure->total_ecd*up_motor_sign[i], up_target[i]);
		PID_Calc(up_motor_speed_pid+i,up_motor[i].speed,speed);
	}
	
	up_motor_speed_pid[1].out=PID_Calc(up_motor_position_pid+1, up_motor[1].chassis_motor_measure->total_ecd*up_motor_sign[1], up_target[1]);
		
	

}
void trans_ball_task(int* num){
  while(num>0){
	up_target[0] += next_cmd.ball_num * PI /2;
	num--;
	}
	vTaskDelete(NULL);
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
		vTaskDelay(UP_CONTROL_TIME_MS);
	}
	
	
}

int timer_state_sign;

xTaskHandle p_timer_handle;

void timer_delay_task(void const *pvParameters){
	vTaskDelay(*(uint32_t*)pvParameters);
	timer_state_sign=1;
	vTaskDelete(NULL);
}
uint32_t mtime;
void timer_start(int time){
	timer_state_sign=0;
	if(eTaskGetState( p_timer_handle) != eRunning && eTaskGetState( p_timer_handle) != eReady && eTaskGetState( p_timer_handle) != eSuspended )
		;
	else
		vTaskDelete(p_timer_handle);
	xTaskCreate((TaskFunction_t)timer_delay_task,"timer_peng",128,&time,osPriorityHigh,p_timer_handle);
}

//int timer_is_finish(){
//	return timer_state_sign;
//}



#include "pwm.h"
#include "up_control.h"

steering_engine servo;

void up_init(){
	servo.port=A;
}

void steer_open(void){
	change_pwm(&servo,STEER_OPEN_ANGLE);
}

void steer_close(){
	change_pwm(&servo,STEER_CLOSE_ANGLE);
}




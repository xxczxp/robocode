#ifndef PWM_H
#define PWM_H
#include "main.h"

typedef struct{
	char name;
	TIM_HandleTypeDef  htim;
	int  channal;
}steering_engine;

extern void steering_engine_init(steering_engine *item);

extern void change_pwm(steering_engine *item, double degree);

#endif

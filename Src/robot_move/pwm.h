#ifndef PWM_H
#define PWM_H
#include "main.h"

typedef struct{
	char name;
	TIM_HandleTypeDef  htim;
	uint32_t  channel;
}steering_engine;

extern void steering_engine_init(steering_engine *item);
extern void pwm_all_init(void);
extern void change_pwm(steering_engine *item, float degree);

#endif

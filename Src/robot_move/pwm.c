#include "main.h"
#include "stm32_hal_legacy.h"

//#define __HAl_TIM_SetCompare
extern TIM_HandleTypeDef htim4;

void set_pwm (void){

HAL_TIM_Base_Start(&htim4);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
int i = 0, j = 0;
while(1){
	if (j < 1){
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,  1000);
	j++;
	}
	else{
	j =0;
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,2500);
	}
	osDelay(1000);
}

}

#include "main.h"
#include "stm32_hal_legacy.h"
#include "pwm.h"


extern TIM_HandleTypeDef htim4;

void steering_engine_init(steering_engine *item){
	HAL_TIM_Base_Start(&(item->htim));

	switch (item -> channal){

		case 1 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_1);

		case 2 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_2);

		case 3 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_3);

		case 4 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_4);

			}
}


void change_pwm(steering_engine *item, double degree){

	uint8_t channal;

	switch (item -> channal){
		case 1 : channal = TIM_CHANNEL_1;

		case 2 : channal = TIM_CHANNEL_2;

		case 3 : channal = TIM_CHANNEL_3;

		case 4 : channal = TIM_CHANNEL_4;

	}
	degree = degree/0.09 + 500;
	__HAL_TIM_SetCompare(&(item->htim), channal, degree);


}

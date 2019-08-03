#include "main.h"
#include "stm32_hal_legacy.h"
#include "pwm.h"
//#include "bsp_buzzer.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;



void pwm_all_init(void){
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim8);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	
}

void change_pwm(steering_engine *item, float degree){
	degree = degree/0.09 + 500;
	
	switch(item->port)
	{
		case A : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, degree); break;
		
		case B : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, degree); break;
		
		case C : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, degree); break;
		
		case D : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, degree); break;
		
		case E : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, degree); break;
		
		case F : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, degree); break;
		
		case G : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, degree); break;
		
		case H : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, degree); break;
		
		case S : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, degree); break;
		
		case T : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, degree); break;
	
		case U : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, degree); break;
		
		case V : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, degree); break;
		
		case W : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, degree); break;
		
		case X : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, degree); break;
		
		case Y : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, degree); break;
		
		case Z : 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, degree); break;
	
		default:   ;
	
		}
	
}

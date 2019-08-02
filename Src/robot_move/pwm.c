
#include "main.h"
#include "stm32_hal_legacy.h"
#include "pwm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

//#define __HAl_TIM_SetCompare
//extern TIM_HandleTypeDef htim4;

void steering_engine_init(steering_engine *item){
	HAL_TIM_Base_Start(&(item->htim));

//	switch (item -> channel){
//
//		case 1 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_1);break;
//
//		case 2 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_2);break;
//
//		case 3 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_3);break;
//
//		case 4 : HAL_TIM_PWM_Start(&(item -> htim), TIM_CHANNEL_4);break;
//
//
//			}
	HAL_TIM_PWM_Start(&(item -> htim), item->channel);

}

void pwm_all_init(void){
	steering_engine pwminit;
	pwminit.htim=htim1;
	pwminit.channel=TIM_CHANNEL_1;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_2;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_3;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_4;
	steering_engine_init(&pwminit);

	pwminit.htim=htim2;
	pwminit.channel=TIM_CHANNEL_1;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_2;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_3;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_4;
	steering_engine_init(&pwminit);

	pwminit.htim=htim3;
	pwminit.channel=TIM_CHANNEL_1;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_2;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_3;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_4;
	steering_engine_init(&pwminit);

	pwminit.htim=htim4;
	pwminit.channel=TIM_CHANNEL_1;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_2;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_3;
	steering_engine_init(&pwminit);
	pwminit.channel=TIM_CHANNEL_4;
	steering_engine_init(&pwminit);
}

void change_pwm(steering_engine *item, float degree){

//	uint32_t schannel;
//
//	switch (item -> channel){
//		case 1 : schannel = TIM_CHANNEL_1;break;
//
//		case 2 : schannel = TIM_CHANNEL_2;break;
//
//		case 3 : schannel = TIM_CHANNEL_3;break;
//
//		case 4 : schannel = TIM_CHANNEL_4;break;
//
//
//
//	}
	degree = degree/0.09 + 500;
	__HAL_TIM_SetCompare(&(item->htim), item -> channel, degree);

}

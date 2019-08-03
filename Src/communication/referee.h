//my recive 
#define RECIVE_TERM_SIZE 129
#define RECIVE_BUFFER_SIZE 5

#ifndef REFEREE_H
#define REFEREE_H

//裁判信息解读与透传数据帧封装程序
#include "bsp_remote_control.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"



typedef float auto_t;

typedef struct {
	int8_t is_new;
	auto_t x;
	auto_t y;
	auto_t z;
	auto_t roll;
	auto_t pitch;
	auto_t wx;
	auto_t wy;
	auto_t wz;
} apriltap_data_t;

extern void referee_task(void const * argument);
extern SemaphoreHandle_t apriltag_handle;
extern apriltap_data_t apriltap_data;

extern uint16_t referee_data_solve(uint8_t *frame);

#endif

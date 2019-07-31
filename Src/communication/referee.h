#ifndef REFEREE_H
#define REFEREE_H

//裁判信息解读与透传数据帧封装程序
#include "bsp_remote_control.h"

//my recive 
#define RECIVE_TERM_SIZE 129
#define RECIVE_BUFFER_SIZE 256

extern void referee_task(void const * argument);


extern uint16_t referee_data_solve(uint8_t *frame);

#endif

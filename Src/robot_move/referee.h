#ifndef REFEREE_H
#define REFEREE_H

//裁判信息解读与透传数据帧封装程序
#include "bsp_remote_control.h"


extern void init_referee_struct_data(void);
extern uint16_t referee_data_solve(uint8_t *frame);

#endif

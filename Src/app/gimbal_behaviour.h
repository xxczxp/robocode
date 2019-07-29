#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_CALI,           //��̨У׼
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);




#endif

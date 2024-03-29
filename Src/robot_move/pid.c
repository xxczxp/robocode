/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include <stdio.h>
#include "pid.h"
#include "main.h"
#include "math.h"
#include "chassis_control.h"
#include "light_matrix.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}



float result[2];
void PID_Calc_L(PidTypeDef *pid_x, PidTypeDef *pid_y, location_t *target, location_t *current)
{
    if ((pid_x == NULL || pid_y == NULL) || (pid_x->mode == PID_LOCATION && pid_y->mode == PID_LOCATION ))
    {
//        printf("pid is NULL !");
    }    
		Mat RotationT ;
		Mat MoveV ;
		Mat Tresult ; 
	float k[3][6];
		
		MatInit(&RotationT, 2, 2,k[0]);
		MatInit(&MoveV, 2, 1,k[1]);
		MatInit(&Tresult, 2, 1,k[2]);
		
		RotationT.element[0][0]=  cos(current -> w);
		RotationT.element[0][1]= -sin(current -> w);
		RotationT.element[1][0]=  sin(current -> w);
		RotationT.element[1][1]=  cos(current -> w);
		
		MoveV.element[0][0] = target -> x  - current -> x;
		MoveV.element[1][0] =  target -> y  - current -> y;
		
		MatMul(&RotationT, &MoveV, &Tresult);
//		float transform_x, transform_y;
//		transform_x = Tresult.element[0][0];
//		transform_y = Tresult.element[1][0];
		
		result[0] = PID_Calc(pid_x, current -> x, target -> x);
    result[1] = PID_Calc(pid_y, current -> y, target -> y);
	
		}

		
void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

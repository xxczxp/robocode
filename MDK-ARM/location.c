#include "chassis_control.h"
#include "chassis_task.h"
#include "PID.h"

#include "main.h"
#include "arm_math.h"
#include "string.h"
#include "protocol.h"
#include "cmsis_os.h"

#define FRONT 0;
#define RIGHT 1;
#define LEFT -1;

extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern chassis_move_t chassis_move;
typedef struct{
int x;
int y;
int wz;
}Location;

void chassis_motor_location_control (Location *current, Location *target){

if (current == NULL || target == NULL){
	return;
}

Lx = target->x - current->x;
Ly = target->y - current->y;
Lz = target->z - current->z;

PidTypeDef pid_x_set;
PidTypeDef pid_y_set;
PidTypeDef pid_z_set;
PidTypeDef pid_x;
PidTypeDef pid_y;
PidTypeDef pid_z;

char i = "x";
int speed_x;
int speed_y;
int speed_z;

for(int q = 0; q < 4; q++){
		if ( i == "x"){
			
			
			
			if (Lx == 0){
				;
			}
			
			else {
				speed_x_set = PID_Calc(pid_x_set, current->x, target->x);
			  speed_x = PID_Calc(pid_x,chassis_move->vx, speed_x_set);
				i == "y";
			}
			
		}
		
		
		
		else if ( i == "y"){
			
			if (Ly == 0){
				;
			}
			
			else {
				speed_y_set = PID_Calc(pid_y_set, current->y, target->y);
			  speed_y = PID_Calc(pid_y,chassis_move->vy, speed_y_set);
				i == "z";
			}
		}
		
		
		
		else if ( i == "z"){
			
			if (Lz == 0){
				;
			}
			
			else {
				speed_z_set = PID_Calc(pid_z_set, current->z, target->z);
			  speed_z = PID_Calc(pid_z,chassis_move->vz, speed_z_set);
			}
			
			
			
		}
		
	}
chassis_move -> vx_set = speed_x;
chassis_move -> vy_set = speed_y;
chassis_move -> vz_set = speed_x;

}







//fp32 PID_LocationX(PidTypeDef *pid, int target, int current)
//{
//	if (pid == NULL){
//		
//		return;
//		
//	}
//	target = target;
//	current = current;
//	
//	pid->error[2] = pid->error[1];
//  pid->error[1] = pid->error[0];
//  pid->error[0] = target - current;
//	
//	if (pid -> mode = PID_LOCATION){
//		
//		pid->Pout = pid->Kp * pid->error[0];
//    pid->Iout += pid->Ki * pid->error[0];
//    pid->Dbuf[2] = pid->Dbuf[1];
//    pid->Dbuf[1] = pid->Dbuf[0];
//    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//    pid->Dout = pid->Kd * pid->Dbuf[0];
//    LimitMax(pid->Iout, pid->max_iout);
//    pid->out = pid->Pout + pid->Iout + pid->Dout;
//    LimitMax(pid->out, pid->max_out);	
//	
//	}
//	
//	
//	

//}
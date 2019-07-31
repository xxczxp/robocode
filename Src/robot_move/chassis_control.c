#include "chassis_control.h"
#include "chassis_task.h"
#include "PID.h"

#include "main.h"
#include "arm_math.h"
#include "string.h"
#include "protocol.h"
#include "cmsis_os.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.00005f
#define AB 0.5f

PidTypeDef auto_x;
PidTypeDef auto_y;
PidTypeDef auto_wz;
extern uint8_t chassis_odom_pack_solve(
  float x,
  float y,
  float odom_yaw,
  float vx,
  float vy,
  float vw,
  float gyro_z,
  float gyro_yaw);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern int	result[2];
location_t current;
location_t target;



//底盘运动数据
 chassis_move_t chassis_move;
 uint8_t usb_tx[128];

  // auto_control unpacked data
extern chassis_ctrl_info_t ch_auto_control_data;

void chassis_motor_speed_update(chassis_move_t *chassis_move_update)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }
}

void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0]=-(vx_set+vy_set+wz_set*AB);
	wheel_speed[1]=vx_set-vy_set-wz_set*AB;
	wheel_speed[2]=vx_set+vy_set-wz_set*AB;
	wheel_speed[3]=-vx_set+vy_set-wz_set*AB;
    
    
}

int32_t v[4]={0,0,0,0};
int32_t sv[4]={0,0,0,0};
int32_t dv[4]={0,0,0,0};

static fp32 distance_x = 0.0f, distance_y = 0.0f, distance_wz = 0.0f;
void chassis_distance_calc_task(void const * argument)
{
		
		float x,y,theta,s[4];
	
    
    while(1)
    {
				for(int i=0;i<4;i++){
			s[i]=chassis_move.motor_chassis[i].speed;
		}
				for(int i=0;i<4;i++){
			sv[i]=v[i];
			v[i]=chassis_move.motor_chassis[i].chassis_motor_measure->total_ecd;
					dv[i]=v[i]-sv[i];
		}
			x=(-dv[0]+ dv[1]+dv[2]-dv[3])/4;
			y=(dv[0]- dv[1]+dv[2]-dv[3])/4;   
			theta=(dv[0]- dv[1]-dv[2]+dv[3])/(4*AB);  
				
		chassis_move.vx=(-s[0]+ s[1]+s[2]-s[3])/4;
				chassis_move.vy=(s[0]- s[1]+s[2]-s[3])/4;   
				chassis_move.wz=(s[0]- s[1]-s[2]+s[3])/(4*AB); 
				distance_x+=(cos(theta)*x-sin(theta)*y);
				distance_y+=(sin(theta)*x+sin(theta)*y);
				distance_wz+=theta;
								
		
        osDelay(1);
    }
}


void chassis_distance_send_task(void const * argument)
{
    
    
    while(1)
    {
        uint8_t send_len;
        //send_len = chassis_odom_pack_solve( distance_x, distance_y, distance_wz, chassis_move.vx, chassis_move.vy, chassis_move.wz, chassis_move.chassis_gyro_z, chassis_move.chassis_yaw);
        osDelay(10);
    }
    
}




void chassis_normal_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
	}



location_t target;
location_t current;
PidTypeDef PID_Lx ;
PidTypeDef PID_Ly ;
PidTypeDef PID_Lw ;


void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
<<<<<<< Updated upstream
	//闭位置环
   current.x = distance_x;
	 current.y = distance_y;
	 current.w = distance_wz;
	 
	*vx_set = PID_Calc_L(&PID_Lx, current.x, target.x);
	*vy_set = PID_Calc_L(&PID_Ly, current.y, target.y);
	*wz_set = PID_Calc_L(&PID_Lw, current.w, target.w);
=======
	
	current.x = distance_x;
	current.y = distance_y;
	current.w = distance_wz;
	PID_Calc_L(&auto_x, &auto_y, &target, &current);
	*vx_set =result[0];
	*vy_set =result[1];
	*wz_set =PID_Calc(&auto_wz, current.w, target.w);
>>>>>>> Stashed changes
	
    return;
}



void chassis_PID_init(void)
{
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    const static fp32 chassis_rotation_pid[3] = {CHASSIS_ROTATION_PID_KP, CHASSIS_ROTATION_PID_KI, CHASSIS_ROTATION_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_angle_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

    uint8_t i;


    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {
        PID_Init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }


    //初始化旋转PID
    PID_Init(&chassis_move.chassis_rotation_pid, PID_POSITION, chassis_rotation_pid, CHASSIS_ROTATION_PID_MAX_OUT, CHASSIS_ROTATION_PID_MAX_IOUT);

    PID_Init(&chassis_move.chassis_angle_pid, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);


}






#include "chassis_control.h"
#include "chassis_task.h"
#include "PID.h"
#include "math.h"

#include "up_control_task.h"
#include "INS_Task.h"
#include "main.h"
#include "arm_math.h"
#include "string.h"
#include "protocol.h"
#include "cmsis_os.h"
#include  "chassis_behaviour.h"
#include "referee.h"
#include "semphr.h"
#include "kalman.h"

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 1.10537517e-4
#define AB /*0.25f*/ 0.0405
#define WHEEL_R 0.0072f
#define ARG /*(CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * WHEEL_R)*/ 4.089931e-4
#define Pi acos(-1)


PidTypeDef auto_x = {0, 0.5, 7e-10, 0.0005, 1, 1};
PidTypeDef auto_y = {0, 0.0f, 0.0f, 0.0f, 1, 1};
PidTypeDef auto_wz = {0, 0.5, 7e-10, 0.005, 1, 1};

extern SemaphoreHandle_t apriltag_handle;

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
extern float	result[2];
location_t current;
location_t target = {0, 0, 0};



//�����˶�����
 chassis_move_t chassis_move;
 uint8_t usb_tx[128];


  // auto_control unpacked data
extern chassis_ctrl_info_t ch_auto_control_data;

//real auto_control
QueueHandle_t auto_queue;
void step_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

void chassis_motor_speed_update(chassis_move_t *chassis_move_update)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ����ٶȣ����ٶ����ٶȵ�PID΢��
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

float v[4]={0,0,0,0};
float sv[4]={0,0,0,0};
float dv[4]={0,0,0,0};

float encode_sign[4];

kalman_filter_t pos_kalman[3];
feiman_filter_t wz_feiman;

extern rm_imu_data_t rm_imu_data;
float imu_last;
const float *imu_angle;

#define ODOM_V 0.001
#define APRIL_V 0.0005
#define GYRO_V 0.005
#define ANGLE_USE 0

fp32 distance_x = 0.0f, distance_y = 0.0f, distance_wz = 0.0f;

void angle_sim(const float target,float *var){
	while((*var-target)>Pi)
		*var-=2*Pi;
	while((*var-target)<-Pi)
		*var+=2*Pi;
	
}

void kalman_use(void){
	if(xSemaphoreTake(apriltag_handle,0)==pdTRUE)
		{
			if(apriltap_data.is_new==1){
				apriltap_data.is_new=0;
//				distance_x=distance_x*0.5+apriltap_data.x*0.5;
//				distance_x=distance_y*0.5+apriltap_data.y*0.5;
//				distance_x=distance_wz*0.5+apriltap_data.wz*0.5;
				angle_sim(imu_angle[ANGLE_USE]-imu_last+distance_wz,&distance_wz);
				distance_wz=feiman_update(&wz_feiman,distance_wz,imu_angle[ANGLE_USE]-imu_last+distance_wz);
				distance_x=kalman_update(&pos_kalman[0],distance_x,apriltap_data.x);
				distance_y=kalman_update(&pos_kalman[1],distance_y,apriltap_data.y);
				angle_sim(apriltap_data.wz,&distance_wz);
				distance_wz=kalman_update(&pos_kalman[2],distance_wz,apriltap_data.wz);

			}
			
			else{
				pos_kalman[0].v_pre+=pos_kalman[0].v_noise_pre;
			pos_kalman[1].v_pre+=pos_kalman[1].v_noise_pre;
				angle_sim(imu_angle[ANGLE_USE]-imu_last+distance_wz,&distance_wz);
			distance_wz=feiman_update(&wz_feiman,distance_wz,imu_angle[ANGLE_USE]-imu_last+distance_wz);
				
			}
			
			xSemaphoreGive(apriltag_handle);
		}
		else {
			pos_kalman[0].v_pre+=pos_kalman[0].v_noise_pre;
			pos_kalman[1].v_pre+=pos_kalman[1].v_noise_pre;
			angle_sim(imu_angle[ANGLE_USE]-imu_last+distance_wz,&distance_wz);
			distance_wz=feiman_update(&wz_feiman,distance_wz,imu_angle[ANGLE_USE]-imu_last+distance_wz);
		}
}


void chassis_distance_calc_task(void const * argument)
{
	for(int i=0;i<3;i++){
		pos_kalman[i].v_noise_pre=ODOM_V;
		pos_kalman[i].v_noise_get=APRIL_V;
		pos_kalman[i].v_pre=0;
	}
	pos_kalman[2].v_noise_pre=0.0;
	wz_feiman.v_noise_1=ODOM_V;
	wz_feiman.v_noise_2=GYRO_V;
	wz_feiman.v_1=0;
	wz_feiman.v_2=0;
	imu_angle=get_INS_angle_point();
	imu_last=imu_angle[ANGLE_USE];
	
	
	
		float x,y,theta,s[4];
	distance_x=0;
	distance_y=0;
	distance_wz=0;
	encode_sign[0]=-1;
	encode_sign[1]=1;
	encode_sign[2]=1;
	encode_sign[3]=-1;

    while(1)
    {
		chassis_move.vx=(s[0]+ s[1]+s[2]+s[3])/4;
		chassis_move.vy=(s[0]- s[1]+s[2]-s[3])/4;
		chassis_move.wz=(s[0]- s[1]-s[2]+s[3])/(4*AB);

		
				for(int i=0;i<4;i++){
			s[i]=chassis_move.motor_chassis[i].speed;
		}
				for(int i=0;i<4;i++){
			sv[i]=v[i];
			v[i]=(float)chassis_move.motor_chassis[i].chassis_motor_measure->total_ecd;
					dv[i]=(v[i]-sv[i])*ARG*encode_sign[i];
		}

			x=(dv[0]+ dv[1]+dv[2]+dv[3])/4;
			y=(dv[0]- dv[1]+dv[2]-dv[3])/4;
			theta=(dv[0]- dv[1]-dv[2]+dv[3])/(4*AB);

			
				distance_x+=(arm_cos_f32(distance_wz)*x-arm_sin_f32(distance_wz)*y);
				distance_y+=(arm_sin_f32(distance_wz)*x+arm_cos_f32(distance_wz)*y);
				distance_wz+=theta;
		
		kalman_use();
		
		imu_last=imu_angle[ANGLE_USE];
			
		//if(switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))

        osDelay(5);
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


extern chassis_ctrl_info_t ch_auto_control_data;
void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {

        return;
    }


	current.x = distance_x;
	current.y = distance_y;
	current.w = 0;
	
		
		double dis=sqrt(distance_x*distance_x+distance_y*distance_y);
		double b=distance_x/dis;
	  double right_degree  = acos(distance_x/dis);
		if(asin(distance_y/dis)<0)
			right_degree=2*Pi-right_degree;
		
		target.x=3;
		
		
	double adjustment_dis_degee =distance_wz;
		

	double delta_degree = right_degree-adjustment_dis_degee;
		while(delta_degree>PI)
			delta_degree-=2*Pi;
		while(delta_degree<-Pi)
			delta_degree+=2*Pi;
		
	PID_Calc_L(&auto_x, &auto_y, &target, &current);
	*vx_set = 0.5*PID_Calc(&auto_x,dis,target.x);
	*vy_set = 0.6;
	*wz_set = PID_Calc(&auto_wz, distance_wz,distance_wz+delta_degree);
	
		// step_auto_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	
}

static unsigned char ucParameterToPass;
auto_pack_t next_cmd;
int state=CMD_GET;
int inner_state = move_target; 
int is_create = 0;
void step_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector){
	static auto_pack_t pack;
	xTaskHandle C_O_T;
  xTaskHandle T_B_T;
	
	switch(state){
		case CMD_GET:
		{ 
			if(xQueueReceive(auto_queue,&pack,0)==pdPASS)
			{		 
				xQueuePeek(auto_queue,&next_cmd,10);
				if(pack.cmd==MOVE_CMD && next_cmd.cmd == PUT_BALL_CMD){
					state=MOVE; 
				  if(is_create == 0){
						xTaskCreate(cup_out_task,"cup_out_task" , 512, &ucParameterToPass, 1, &C_O_T);	
						xTaskCreate(trans_ball_task, "trans_ball_task", 512, &ucParameterToPass, 1, &T_B_T);
						is_create = 1;
					}
					else{
						return;
					}
				}
				else if(pack.cmd==MOVE_CMD && next_cmd.cmd != PUT_BALL_CMD){
					state=MOVE;
				}
				else if(pack.cmd==PUT_BALL_CMD){
					state=PUT_BALL;
				}
				target.x = pack.target.x*0.93;
				target.y = pack.target.y*0.93;
			}
		}
			case MOVE:
		{
			if((fabs(distance_x-target.x)<X_PASS_LIMIT)&&(fabs(distance_y-target.y)<Y_PASS_LIMIT) && (fabs(distance_wz-target.w)<WZ_PASS_LIMIT))
			{
				
				state=CMD_GET;
				*vx_set=0;
				*vy_set = 0;
				*wz_set = 0;
			}
			else{
				float werr,xerr,yerr;
				current.x=distance_x;
				current.y=distance_y;
				current.w=distance_wz;
				
				xerr=pack.target.x-distance_x;
				yerr=pack.target.y-distance_y;
				
				float dis=sqrt(xerr*xerr+yerr*yerr);
				float right_degree  = (float)acos(xerr/dis);
				if(asin(yerr/dis)<0)
					right_degree=2*Pi-right_degree;
				
				pack.target.w=right_degree;
				
				werr=pack.target.w-distance_wz;
				while(werr>Pi){
					werr-=2*Pi;
				}
				while(werr<-Pi){
					werr+=2*Pi;
				}
				current.w=pack.target.w+werr;
				
				PID_Calc_L(&auto_x, &auto_y, &pack.target, &current);
				*vx_set =result[0];
				*vy_set =result[1];
				*wz_set=PID_Calc(&auto_wz,current.w,target.w);
			}
		}break;
		
		case  PUT_BALL_CMD:
		{
			switch (inner_state){
				case move_target :{
				
				
				
				}
			
			
			
			
			
			
			}
		
		
		
		
		
		
		}
	}
}


void chassis_PID_init(void){
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    const static fp32 chassis_rotation_pid[3] = {CHASSIS_ROTATION_PID_KP, CHASSIS_ROTATION_PID_KI, CHASSIS_ROTATION_PID_KD};
    //������ת��pidֵ
    const static fp32 chassis_angle_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};

    uint8_t i;


    //��ʼ��PID �˶�
    for (i = 0; i < 4; i++)
    {
        PID_Init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }


    //��ʼ����תPID
    PID_Init(&chassis_move.chassis_rotation_pid, PID_POSITION, chassis_rotation_pid, CHASSIS_ROTATION_PID_MAX_OUT, CHASSIS_ROTATION_PID_MAX_IOUT);

    PID_Init(&chassis_move.chassis_angle_pid, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);


}

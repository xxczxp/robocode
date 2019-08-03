#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "semphr.h"

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
  
//USB底层发送函数，直接操作硬件
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//计算式输入数据
communicate_class_input_data_t communicate_input_data;
//计算式输出结果
communicate_class_output_data_t communicate_output_data;

extern QueueHandle_t referee_send_queue;



//实现RM协议的序列化过程
void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len)
{
    // 定义至少128字节大小缓存数组
    static uint8_t send_buf[RECIVE_TERM_SIZE];
    uint16_t index = 0;
    // 定义帧头结构体
    frame_header_struct_t referee_send_header;
    
	//**buf[0] record the length of the frame
	index++;
	
    // 初始化对应帧头结构体
    referee_send_header.SOF = HEADER_SOF;
    referee_send_header.data_length = len;
    referee_send_header.seq++;
    
    // 生成CRC8校验
    Append_CRC8_Check_Sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    
    memcpy(send_buf + index, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(send_buf + index, (void*)&cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);

    // 填充数据包
    memcpy(send_buf + index, (void*)buf, len);
    index += len;

    // 生成CRC16校验
    Append_CRC16_Check_Sum(send_buf+1, REF_HEADER_CRC_CMDID_LEN + len);
    index += sizeof(uint16_t);
    
	//**buf[0] record the length of the frame
	send_buf[0]=(uint8_t)(index&0xFF);
	
    xQueueSend(referee_send_queue, send_buf, 10);
    // 调用底层发送函数
//    CDC_Transmit_FS(send_buf, index);
}

uint8_t chassis_odom_pack_solve(
  float x,
  float y,
  float odom_yaw,
  float vx,
  float vy,
  float vw,
  float gyro_z,
  float gyro_yaw){
	  
	  //distance_x, distance_y, distance_wz, chassis_move.vx, chassis_move.vy, chassis_move.wz, chassis_move.chassis_gyro_z, chassis_move.chassis_yaw
	  int index=0;
	  static uint8_t send_buf[RECIVE_TERM_SIZE];
	  memcpy(send_buf+index,&x,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&y,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&odom_yaw,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&vx,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&vy,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&vw,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&gyro_z,sizeof(float));
	  index+=sizeof(float);
	  memcpy(send_buf+index,&gyro_yaw,sizeof(float));
	  index+=sizeof(float);
	  referee_send_data(CHASSIS_ODOM_CMD_ID,send_buf,index);
		return index+sizeof(float)+REF_HEADER_CRC_CMDID_LEN;
	  
	  
  }

//TODO：实现计算式运算和结果上发
//计算式信息包反序列化已经实现
//通过communicate_input_data结构体可以直接获取计算式信息
//operate '+' '-' '*' '/'
void communicate_class_solve(void)
{
    //TODO 查询操作符
    //TODO计算底层计算结果
    switch(communicate_input_data.operate)
    {
        case '+':
        {
            communicate_output_data.result = communicate_input_data.data1 + communicate_input_data.data2;
            goto return_result;
        }
        case '-':
        {
            communicate_output_data.result = communicate_input_data.data1 - communicate_input_data.data2;
            goto return_result;
        }
        case '*':
        {
            communicate_output_data.result = communicate_input_data.data1 * communicate_input_data.data2;
            goto return_result;
        }
        case '/':
        {
            communicate_output_data.result = communicate_input_data.data1 / communicate_input_data.data2;
            goto return_result;
        }
        return_result:
        {
            //上发计算结果
            referee_send_data(CLASS_COM_CMD_ID, &communicate_output_data, 4);
            break;
        }
        default :
        {
            break;
        }
    }
}


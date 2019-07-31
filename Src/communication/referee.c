#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "cmsis_os.h"

//计算式输入数据
extern communicate_class_input_data_t communicate_input_data;
//计算式输出结果
extern communicate_class_output_data_t communicate_output_data;

//USB接收FIFO初始化
void usb_fifo_init(void);

void referee_send_task(void const * argument);
    
// auto_control unpacked data
chassis_ctrl_info_t ch_auto_control_data;

//USB FIFO控制结构体
fifo_s_t usb_fifo;
//USB FIFO环形缓存区
uint8_t usb_fifo_buf[512];
//RM协议解包控制结构体
unpack_data_t referee_unpack_obj;
//RM协议反序列化函数
void referee_unpack_fifo_data(void);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//RM协议序列化函数
void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len);

uint16_t communicate_result_pack(uint8_t *buf, communicate_class_output_data_t output);
void communicate_class_solve(void);

frame_header_struct_t referee_receive_header;
TimerHandle_t referee_send_handle;
QueueHandle_t referee_send_queue;

//RM协议解析函数，系统自动调用
//-hk-<任务>建立referee_send_task任务及消息队列，
void referee_task(void const * argument)
{
    usb_fifo_init();
    xTaskCreate((TaskFunction_t)referee_send_task, "send_task", 512, NULL, 0, &referee_send_handle);
    referee_send_queue = xQueueCreate(RECIVE_BUFFER_SIZE, RECIVE_TERM_SIZE);
    while(1)
    {
			//-hk-解包裁判数据，正确则存于referee_unpack_obj中并发送
      referee_unpack_fifo_data();
      osDelay(10);
    }
}
//-hk-<任务>通过消息队列发送数据，但此usb_tx_buf中未放置需发送的数据
void referee_send_task(void const * argument)
{
	uint8_t usb_tx_buf[RECIVE_TERM_SIZE];

	while(1)
	{
		if(xQueueReceive( referee_send_queue, usb_tx_buf, 10 ) == pdPASS)
		{
			CDC_Transmit_FS(usb_tx_buf+1, usb_tx_buf[0]);
		}
	}
}

//USB FIFO初始化函数
void usb_fifo_init(void)
{
  fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
}

//USB接收中断
void usb_receiver(uint8_t *buf, uint32_t len)
{
  fifo_s_puts(&usb_fifo, (char*)buf, len);
}

//RM协议反序列化
//-hk-解包裁判数据，正确则存于referee_unpack_obj中并发送
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&usb_fifo) )
  {
    byte = fifo_s_get(&usb_fifo);
    switch(p_obj->unpack_step)
    {
      //查找帧头
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      //获取数据长度低字节
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      //获取数据长度高字节
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      //记录协议包序列号
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      //校验帧头CRC8
      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      //校验整帧CRC16
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            //成功接收一正确的信息包
						//-hk-解算裁判数据并发送
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      //解包失败重新寻找帧头
      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

//-hk-解算裁判数据并发送
uint16_t referee_data_solve(uint8_t *frame)
{
    uint8_t index = 0;
    uint16_t cmd_id = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    switch (cmd_id)
    {
        //接收计算式命令码对应信息包
        case CLASS_COM_CMD_ID:
        {
            memcpy(&communicate_input_data, frame + index, sizeof(communicate_class_input_data_t));
            //调用计算式处理函数
            communicate_class_solve();
            break;
        }
		
				//DEBUG recive hehe	
				case CHASSIS_CTRL_CMD_ID:
				{
					memcpy(&ch_auto_control_data.vx,frame + index,sizeof(float));
					index+=sizeof(float);
					memcpy(&ch_auto_control_data.vy,frame + index,sizeof(float));
					index+=sizeof(float);
					memcpy(&ch_auto_control_data.vw,frame + index,sizeof(float));
					index+=sizeof(float);
					break;
				}
		
        default:
        {
            break;
        }
    }

    index += referee_receive_header.data_length + 2;
    return index;
}

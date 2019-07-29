#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

//RMЭ������������
typedef enum
{
  GAME_STATUS_CMD_ID = 0x0001,
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
  CLASS_COM_CMD_ID = 0xFFFF,
} referee_data_cmd_id_tpye;

//RMЭ��֡ͷ�ṹ��
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

//RMЭ�鷴���л�����ö��
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//RMЭ�鷴���л��ṹ��
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

//����ʽ��Ϣ��
typedef struct
{
  float data1;
  uint8_t operate;
  float data2;
}communicate_class_input_data_t;

//������������
typedef struct
{
  float result;
}communicate_class_output_data_t;

/***************����ͨ����Ϣ�����Эͬ�γ���δʹ��****************/
//RM����Ӫ������������Ϣ��
typedef struct
{
  struct castle_energy
  {
    uint8_t energy[2];
  }castle_energy[7];

  struct region_occupy
  {
    uint8_t status : 2; // 0 = no one, 1 = weak, 2 = strong
    uint8_t belong : 2; // 0 = no one, 1 = red, 2 = blue
    uint8_t have_robot : 2; // same as belong // resv
    uint8_t resv : 2;
  }region_occupy[9][7];

  uint8_t car_location[2];
  uint8_t round_remain_tick;
  uint8_t round_remain_cnt;
  uint8_t round_team;
  int16_t realtime_score[2];
} summer_camp_info_t;

//�����ٶȿ�����Ϣ��
typedef struct
{
  float vx;
  float vy;
  float vw;
}  chassis_ctrl_info_t;

//��̼Ʒ�������
typedef struct
{
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
}  chassis_odom_info_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H

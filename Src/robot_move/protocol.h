#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#pragma pack(push, 1)
typedef enum
{
    GAME_STATUS_CMD_ID = 0x0001,
    CHASSIS_ODOM_CMD_ID = 0x0101,
    CHASSIS_CTRL_CMD_ID = 0x0102,
    CLASS_COM_CMD_ID = 0x0201,

} referee_data_cmd_id_tpye;


typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{
    struct castle_energy
    {
        uint8_t energy[2];
    } castle_energy[7];

    struct region_occupy
    {
        uint8_t status : 2; // 0 = no one, 1 = weak, 2 = strong
        uint8_t belong : 2; // 0 = no one, 1 = red, 2 = blue
        uint8_t have_robot : 2; // same as belong // resv
        uint8_t resv : 2;
    } region_occupy[9][7];

    uint8_t car_location[2];
    uint8_t round_remain_tick;
    uint8_t round_remain_cnt;
    uint8_t round_team;
    int16_t realtime_score[2];
} summer_camp_info_t;

typedef struct
{
    float vx;
    float vy;
    float vw;
}  chassis_ctrl_info_t;

typedef struct
{
    float x;
    float y;
    float odom_yaw;
    float vx;
    float vy;
    float vw;
    float gyro_z;
    float gyro_yaw;
}  chassis_odom_info_t;

#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H

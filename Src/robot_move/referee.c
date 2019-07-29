#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;
summer_camp_info_t summer_camp_info;
chassis_odom_info_t chassis_odom_info;
chassis_ctrl_info_t chassis_ctrl_info;
uint8_t rx_buf[512];

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&summer_camp_info, 0, sizeof(summer_camp_info_t));
    memset(&chassis_odom_info, 0, sizeof(chassis_odom_info_t));
    memset(&chassis_ctrl_info, 0, sizeof(chassis_ctrl_info_t));
}

uint16_t referee_data_solve(uint8_t *frame)
{
    uint8_t index = 0;
    uint16_t cmd_id = 0;
    if(*frame != HEADER_SOF)
    {
        return 0;
    }

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    if (( !Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t))) || ( !Verify_CRC16_Check_Sum(frame, referee_receive_header.data_length + 9)))
    {
        return 0;
    }
    else
    {
        memcpy(&cmd_id, frame + index, sizeof(uint16_t));
        index += sizeof(uint16_t);
        switch (cmd_id)
        {
        case GAME_STATUS_CMD_ID:
        {
            memcpy(&summer_camp_info, frame + index, sizeof(summer_camp_info_t));
            break;
        }
        case CHASSIS_CTRL_CMD_ID:
        {
            memcpy(&chassis_ctrl_info, frame + index, sizeof(chassis_ctrl_info_t));
            break;
        }
        case CLASS_COM_CMD_ID:
        {
            memcpy(rx_buf, frame + index, referee_receive_header.data_length);
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
}

void frame_header_pack_solve(uint8_t *buf, uint16_t data_length)
{
    referee_send_header.SOF = HEADER_SOF;
    referee_send_header.data_length = data_length;
    referee_send_header.seq++;
    Append_CRC8_Check_Sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    memcpy(buf, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
}
uint8_t chassis_odom_pack_solve(uint8_t *buf,
                                float x,
                                float y,
                                float odom_yaw,
                                float vx,
                                float vy,
                                float vw,
                                float gyro_z,
                                float gyro_yaw)
{
    uint8_t index;
    uint16_t cmd_id;

    index = 0;

    frame_header_pack_solve(buf + index, sizeof(chassis_odom_info_t));
    index += sizeof(frame_header_struct_t);

    cmd_id = CHASSIS_ODOM_CMD_ID;
    memcpy(buf + index, &cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);

    chassis_odom_info.x = x;
    chassis_odom_info.y = y;
    chassis_odom_info.odom_yaw = odom_yaw;
    chassis_odom_info.vx = vx;
    chassis_odom_info.vy = vy;
    chassis_odom_info.vw =vw;
    chassis_odom_info.gyro_z = gyro_z;
    chassis_odom_info.gyro_yaw = gyro_yaw;
    memcpy(buf + index, &chassis_odom_info, sizeof(chassis_odom_info_t));
    index += sizeof(chassis_odom_info_t);


    Append_CRC16_Check_Sum(buf, sizeof(frame_header_struct_t) +2 + sizeof(chassis_odom_info_t) + 2);
    index += sizeof(uint16_t);

    return index;
}



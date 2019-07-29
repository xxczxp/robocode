#include "referee_usart_task.h"
#include "main.h"
#include "referee.h"
#include "string.h"
#include "protocol.h"
#include "cmsis_os.h"

extern void  MX_USB_DEVICE_Init(void);
static void search_frame_sof(uint8_t *frame, uint16_t total_len);
uint8_t usb_rx_buf[512];
uint32_t usb_len = 0;

void referee_solve_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
        search_frame_sof(usb_rx_buf, usb_len);
        usb_len = 0;
        osDelay(1);
    }
}

void usb_receiver(uint8_t *buf, uint32_t len)
{
    memcpy(usb_rx_buf,buf, len);
    usb_len = len;
}

static void search_frame_sof(uint8_t *frame, uint16_t total_len)
{
    uint16_t i;
    uint16_t index;
    for(i = 0; i < total_len; )
    {
        if(*frame == HEADER_SOF)
        {
            index = referee_data_solve(frame);
            i += index;
            frame += index;
        }
        else
        {
            i++;
            frame++;
        }
    }

}




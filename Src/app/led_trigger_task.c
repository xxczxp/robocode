#include "led_trigger_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "bsp_power_ctrl.h"
#include "Detect_Task.h"

#include "usb_device.h"


#define user_is_error() toe_is_error(errorListLength)

void led_trigger_task(void const * argument)
{
    MX_USB_DEVICE_Init(); 
    while(1)
    {

        if (!user_is_error())
        {
            led_green_on();
        }
        vTaskDelay(500);
        led_green_off();
        vTaskDelay(500);
    }

}

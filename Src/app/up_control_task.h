
#define STEER_CLOSE_ANGLE 127
#define STEER_OPEN_ANGLE 110

extern void OPCL_task(void const *pvParameters);
extern void cup_out_task(void const *pvParameters);
extern void free_ball_task(void const *pvParameters);
extern void trans_ball_task(void const *pvParameters);
extern void Timer_task(void const *pvParameters);
extern void un_timer_task(void const *pvParameters);

#ifndef UP_CONTROL_H
#define UP_CONTROL_H

extern void up_task(void const *pvParameters);


#endif

#ifndef PWM_H
#define PWM_H
#include "main.h"
typedef enum interface_enum_t{
A, B, C, D, E, F, G, H, S, T, U, V, W, X, Y, Z
}  interface_t;

typedef struct{
	interface_t port;
}steering_engine;

extern void steering_engine_init(steering_engine *item);
extern void pwm_all_init(void);
extern void change_pwm(steering_engine *item, float degree);

#endif

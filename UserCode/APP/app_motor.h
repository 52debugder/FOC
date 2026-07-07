#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__ 

#include "stm32f1xx_hal.h"
#include "foc.h"

void app_foc_mainloop(void);
void app_foc_init(uint8_t motor_num, const foc_hal_t *hal_interface);
uint32_t App_GetMicroseconds(void);

#endif

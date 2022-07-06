#ifndef __BSP_BEEP_H__
#define __BSP_BEEP_H__

#include "main.h"


#define BEEP_STATE_OFF       0
#define BEEP_STATE_ON_ALWAYS 1
#define BEEP_STATE_ON_DELAY  2


#define BEEP_ON()        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, SET)
#define BEEP_OFF()       HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, RESET)


void Beep_Timeout_Close_Handle(void);
void Beep_On_Time(uint16_t time);


#endif

#ifndef __BSP_PWM_SERVO_H__
#define __BSP_PWM_SERVO_H__

#include "gpio.h"

#define MAX_PWM_SERVO   4


#define USE_SERVO_J1
#define USE_SERVO_J2
#define USE_SERVO_J3
#define USE_SERVO_J4


#define SERVO_1_HIGH()  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET)
#define SERVO_1_LOW()   HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET)

#define SERVO_2_HIGH()  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_SET)
#define SERVO_2_LOW()   HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET)

#define SERVO_3_HIGH()  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET)
#define SERVO_3_LOW()   HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET)

#define SERVO_4_HIGH()  HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_SET)
#define SERVO_4_LOW()   HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_RESET)



void PwmServo_Init(void);
void PwmServo_Set_Angle(uint8_t index, uint8_t angle);
void PwmServo_Set_Angle_All(uint8_t angle_s1, uint8_t angle_s2, uint8_t angle_s3, uint8_t angle_s4);
void PwmServo_Handle(void);


#endif

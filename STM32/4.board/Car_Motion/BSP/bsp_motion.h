#ifndef BSP_MOTION_H_
#define BSP_MOTION_H_

#include "stdint.h"

#define ENABLE_REAL_WHEEL    (0)
#define YAW_ADJUST           (1)
#define YAW_NO_ADJUST        (0)

// 轮子一整圈的位移，单位为mm
// The displacement of a wheel in one complete turn, Unit: mm 
#define MECANUM_MINI_CIRCLE_MM       (204.203f)
// 底盘电机间距之和的一半
// Half of the sum of the chassis motor spacing
#define MECANUM_MINI_APB             (164.555f)


// 停止模式，STOP_FREE表示自由停止，STOP_BRAKE表示刹车。
// Stop mode, STOP_FREE: stop freely, STOP_BRAKE: brake
typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;

// The speed structure of the car  小车的速度结构体
typedef struct _car_data
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;


void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);

void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);

void Motion_Handle(void);

void Motion_Get_Speed(car_data_t* car);
float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);


#endif /* BSP_MOTION_H_ */

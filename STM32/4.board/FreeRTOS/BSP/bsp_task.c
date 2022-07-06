/*
 * bsp_task.c
 *
 *  Created on: Mar 5, 2022
 *      Author: Administrator
 */
#include "bsp.h"

int enable_beep = 0;

// LED light task entity function  LED灯任务实体函数
void Task_Entity_LED(void)
{
    while (1)
    {
        // The indicator lights up every 100 milliseconds  指示灯每隔100毫秒亮一次
        LED_TOGGLE();
        osDelay(100);
    }
}

// Buzzer task entity function  蜂鸣器任务实体函数
void Task_Entity_Beep(void)
{
    while (1)
    {
        if (enable_beep)
        {
            // The buzzer goes off every 200 milliseconds  蜂鸣器每200毫秒响一次
            BEEP_ON();
            osDelay(100);
            BEEP_OFF();
            osDelay(100);
        }
        else
        {
            BEEP_OFF();
            osDelay(100);
        }
    }
}

// Key task entity function  按键任务实体函数
void Task_Entity_Key(void)
{
    while (1)
    {
        if (Key1_State(1) == KEY_PRESS)
        {
            // Button controls the buzzer switch  按键控制蜂鸣器开关
            enable_beep = !enable_beep;
        }
        osDelay(10);
    }
}

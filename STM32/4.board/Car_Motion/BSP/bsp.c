#include "bsp.h"

// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
static void Bsp_Led_Show_State_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
		LED_TOGGLE();
	}
}


// The peripheral device is initialized  外设设备初始化
void Bsp_Init(void)
{
	Beep_On_Time(50);
	Motor_Init();
	Encoder_Init();
	PID_Param_Init();
}

int car_state = 0;
// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{
	Motion_Handle();
	
	if (Key1_State(1))
	{
		
		Beep_On_Time(50);
		if (car_state == 0)
		{
			Motion_Ctrl(500, 0, 0);
			car_state = 1;

		}
		else
		{
			Motion_Stop(STOP_BRAKE);
			car_state = 0;
		}
	}
	Bsp_Led_Show_State_Handle();
	Beep_Timeout_Close_Handle();
	HAL_Delay(10);
}

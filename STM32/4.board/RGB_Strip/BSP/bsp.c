#include "bsp.h"


#define INTERVAL     50

// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
void Bsp_Led_Show_State_Handle(void)
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
	RGB_Init();
}


// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{
	static int count = 0;
	static uint8_t index = RGB_CTRL_ALL;
	count++;
	if (count == 1*INTERVAL)
	{
		// RED 亮红色
		RGB_Set_Color(index, 0xff, 0x00, 0x00);
		RGB_Update();
	}
	else if (count == 2*INTERVAL)
	{
		// GREEN 亮绿色
		RGB_Set_Color(index, 0x00, 0xff, 0x00);
		RGB_Update();
	}
	else if (count == 3*INTERVAL)
	{
		// BLUE 亮蓝色
		RGB_Set_Color(index, 0x00, 0x00, 0xff);
		RGB_Update();
		count = 0;
	}

	// Detect button down events   检测按键按下事件
	if (Key1_State(KEY_MODE_ONE_TIME))
	{
		Beep_On_Time(50);
	}

	Bsp_Led_Show_State_Handle();
	Beep_Timeout_Close_Handle();
	HAL_Delay(10);
}


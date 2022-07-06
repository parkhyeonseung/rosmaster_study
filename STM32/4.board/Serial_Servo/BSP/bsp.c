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
	USART1_Init();
	USART3_Init();
}

uint8_t servo_id = 0x01;

// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{
	// Detect button down events   检测按键按下事件
	if (Key1_State(KEY_MODE_ONE_TIME))
	{
		Beep_On_Time(50);
		static int press = 0;
		press++;
		printf("press:%d\n", press);

		UartServo_Get_Angle(servo_id);
		HAL_Delay(12);
		if (press%2)
		{
			UartServo_Ctrl(servo_id, 1000, 500);
		}
		else
		{
			UartServo_Ctrl(servo_id, 3000, 500);
		}
	}

	UartServo_Rx_Parse();
	Bsp_Led_Show_State_Handle();
	Beep_Timeout_Close_Handle();
	HAL_Delay(10);
}

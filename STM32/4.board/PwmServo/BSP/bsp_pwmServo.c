#include "bsp_pwmServo.h"
#include "bsp.h"

uint16_t g_pwm_pulse = 0;

uint8_t g_pwm_angle[MAX_PWM_SERVO] = {90, 90, 90, 90};
uint16_t g_angle_num[MAX_PWM_SERVO] = {149, 149, 149, 149};

// 角度转化为脉冲数, angle= [0, 180]
// The Angle is converted to the number of pulses, angle= [0, 180]
static uint16_t PwmServo_Angle_To_Pulse(uint8_t angle)
{
	uint16_t pulse = (angle * 11 + 500) / 10;
	return pulse;
}

// PWM舵机控制，在定时器中调用，模拟输出PWM信号
// PWM steering gear control, in the timer call, analog output PWM signal
void PwmServo_Handle(void)
{
	g_pwm_pulse++;

#ifdef USE_SERVO_J1
	if (g_pwm_pulse <= g_angle_num[0])
		SERVO_1_HIGH();
	else
		SERVO_1_LOW();
#endif

#ifdef USE_SERVO_J2
	if (g_pwm_pulse <= g_angle_num[1])
		SERVO_2_HIGH();
	else
		SERVO_2_LOW();
#endif

#ifdef USE_SERVO_J3
	if (g_pwm_pulse <= g_angle_num[2])
		SERVO_3_HIGH();
	else
		SERVO_3_LOW();
#endif

#ifdef USE_SERVO_J4
	if (g_pwm_pulse <= g_angle_num[3])
		SERVO_4_HIGH();
	else
		SERVO_4_LOW();
#endif

	if (g_pwm_pulse >= 2000)
		g_pwm_pulse = 0;
}

// Initialize the steering gear  舵机初始化
void PwmServo_Init(void)
{
	for (int i = 0; i < MAX_PWM_SERVO; i++)
	{
		g_pwm_angle[i] = 90;
		g_angle_num[i] = PwmServo_Angle_To_Pulse(g_pwm_angle[i]);
	}
}

// 设置pwm舵机角度，index=0~MAX_PWM_SERVO-1，angle为0-180
// Set the PWM servo Angle, index=0~MAX_PWM_SERVO, Angle to 0-180
void PwmServo_Set_Angle(uint8_t index, uint8_t angle)
{
	if (index >= MAX_PWM_SERVO)
		return;
	if (angle > 180)
		return;
	g_pwm_angle[index] = angle;
	g_angle_num[index] = PwmServo_Angle_To_Pulse(angle);
}

// 设置全部pwm舵机的角度
// Set the Angle of all PWM steering gear
void PwmServo_Set_Angle_All(uint8_t angle_s1, uint8_t angle_s2, uint8_t angle_s3, uint8_t angle_s4)
{
	if (angle_s1 <= 180)
	{
		g_pwm_angle[0] = angle_s1;
		g_angle_num[0] = PwmServo_Angle_To_Pulse(angle_s1);
	}

	if (angle_s2 <= 180)
	{
		g_pwm_angle[1] = angle_s2;
		g_angle_num[1] = PwmServo_Angle_To_Pulse(angle_s2);
	}

	if (angle_s3 <= 180)
	{
		g_pwm_angle[2] = angle_s3;
		g_angle_num[2] = PwmServo_Angle_To_Pulse(angle_s3);
	}

	if (angle_s4 <= 180)
	{
		g_pwm_angle[3] = angle_s4;
		g_angle_num[3] = PwmServo_Angle_To_Pulse(angle_s4);
	}
}

// Timer interrupts the callback function  定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim7.Instance)
	{
		PwmServo_Handle();
	}
}


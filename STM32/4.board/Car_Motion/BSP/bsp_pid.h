#ifndef BSP_PID_H_
#define BSP_PID_H_

#include "stdint.h"

#define PID_DEF_KP      (0.8f)
#define PID_DEF_KI      (0.06f)
#define PID_DEF_KD      (0.5f)

typedef struct _pid_t
{
    float target_val;               //目标值
    float pwm_output;        		//PWM输出值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值

    float err_next;                 //定义下一个偏差值, 增量式
    float integral;          		//定义积分值，位置式
} motor_pid_t;

typedef struct _motor_data_t
{
    float speed_mm_s[4];        // Input value, encoder calculation speed 输入值，编码器计算速度
    float speed_pwm[4];         // Output value, PID calculates PWM value 输出值，PID计算出PWM值
    int16_t speed_set[4];       // Speed setting value  速度设置值
} motor_data_t;


void PID_Param_Init(void);
void PID_Calc_Motor(motor_data_t* motor);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);


#endif /* BSP_PID_H_ */

/*
 * bsp_rgb.c
 *
 *  Created on: Jan 24, 2022
 *      Author: Administrator
 */

#include "bsp.h"

// 硬件spi模拟ws2812时序（用spi的3位数据模拟ws2812的一位数据）
// 要求SPI的通信频率为2.25M，传输一位数据的时间约为444纳秒（ns）
// The hardware SPI simulates the WS2812 timing sequence (using spi's 3-bit data to simulate WS2812's one-bit data). 
// The spi communication frequency is required to be 2.25m and the transmission time of one-bit data is about 444 nanoseconds (ns).  
// 444ns   888ns
//  __
// |  |_|   0b110  high level
//  _   
// | |__|   0b100  low level
#define TIMING_ONE           0x06
#define TIMING_ZERO          0x04


// Store the color information of the light bar  储存灯条的颜色信息
ws2812_t g_ws2812 = {0};

// transmitter data  发送数据
static void WS2812_Send_Data(uint8_t *buf, uint16_t buf_size)
{
	HAL_SPI_Transmit_DMA(&hspi3, buf, buf_size);
}

// 设置单个RGB灯颜色值，index=[0, MAX_RGB-1]，RGB=[0x00000000, 0x00FFFFFF]
// Set single RGB light color value, index=[0, MAX_RGB-1], RGB=[0x00000000, 0x00FFFFFF] 
static void WS2812_Set_Color_One(uint8_t index, uint32_t RGB)
{
    if (index >= MAX_RGB) return;
    uint8_t i;
    uint32_t TempR = 0, TempG = 0, TempB = 0;

    for(i = 0; i < 8; i++)
    {
        (RGB & 0x00010000) == 0 ? (TempR |= (TIMING_ZERO<<(i*3))) : (TempR |= (TIMING_ONE<<(i*3)));
        (RGB & 0x00000100) == 0 ? (TempG |= (TIMING_ZERO<<(i*3))) : (TempG |= (TIMING_ONE<<(i*3)));
        (RGB & 0x00000001) == 0 ? (TempB |= (TIMING_ZERO<<(i*3))) : (TempB |= (TIMING_ONE<<(i*3)));
        RGB >>= 1;
    }
    for (i = 0; i < 3; i++)
    {
        g_ws2812.Strip[index].RGB.R[i] = TempR >> (16-8*i);
        g_ws2812.Strip[index].RGB.G[i] = TempG >> (16-8*i);
        g_ws2812.Strip[index].RGB.B[i] = TempB >> (16-8*i);
    }
}


// Initializes the indicator bar  初始化灯条
void RGB_Init(void)
{
	RGB_Clear();
	RGB_Update();
}

// 刷新RGB灯条颜色。下方函数调用修改RGB颜色后，必须调用此函数更新显示。
// Refresh RGB light bar color. This function must be called to update the display after the RGB color is modified by the function call below.  
void RGB_Update(void)
{
    WS2812_Send_Data((uint8_t*)&g_ws2812.Strip[0].Buff, 9*MAX_RGB);
}

// 设置颜色，index=[0, MAX_RGB-1]控制对应灯珠颜色, index=0xFF控制所有灯珠颜色。
// Set the color, index=[0, max_RGB-1] controls the corresponding bead color, index=0xFF controls all the bead color.
void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = r << 16 | g << 8 | b;
    RGB_Set_Color_U32(index, color);
}

// 设置RGB灯条颜色值，index=[0, MAX_RGB-1]控制对应灯珠颜色, index=255控制所有灯珠颜色。
// Set the RGB bar color value, index=[0, max_RGB-1] controls the corresponding bead color, index=255 controls all the bead color.
void RGB_Set_Color_U32(uint8_t index, uint32_t color)
{
    if (index < MAX_RGB)
    {
        WS2812_Set_Color_One(index, color);
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint16_t i = 0; i < MAX_RGB; i++)
        {
            WS2812_Set_Color_One(i, color);
        }
    }
}

// Clear color (off)  清除颜色（熄灭）
void RGB_Clear(void)
{
    for (uint8_t i = 0; i < MAX_RGB; i++)
    {
        WS2812_Set_Color_One(i, 0);
    }
}

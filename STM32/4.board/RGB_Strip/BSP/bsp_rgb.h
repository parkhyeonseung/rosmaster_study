/*
 * bsp_rgb.h
 *
 *  Created on: Jan 24, 2022
 *      Author: Administrator
 */

#ifndef BSP_RGB_H_
#define BSP_RGB_H_

#include "stdint.h"

#define RGB_CTRL_ALL    0xFF
#define MAX_RGB         14

typedef struct
{
    union 
    {
        uint8_t Buff[9];
        struct 
        {
            uint8_t G[3]; // G First
            uint8_t R[3]; // R Second
            uint8_t B[3]; // B Third
        } RGB;
    } Strip[MAX_RGB];
} ws2812_t;


void RGB_Init(void);
void RGB_Update(void);

void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void RGB_Set_Color_U32(uint8_t index, uint32_t color);
void RGB_Clear(void);


#endif /* BSP_RGB_H_ */

/******************************** END *****************************************/

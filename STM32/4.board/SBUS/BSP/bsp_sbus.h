/*
 * bsp_sbus.h
 *
 *  Created on: Mar 8, 2022
 *      Author: Administrator
 */

#ifndef BSP_SBUS_H_
#define BSP_SBUS_H_

#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS       0x00


void SBUS_Reveive(uint8_t data);
void SBUS_Handle(void);


#endif /* BSP_SBUS_H_ */

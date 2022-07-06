
#include "bsp_mpu9250.h"
#include "bsp.h"

int16_t aacx, aacy, aacz;	 // Accelerometer raw data  加速度传感器原始数据
int16_t gyrox, gyroy, gyroz; // Gyroscope raw data      陀螺仪原始数据
int16_t magx, magy, magz;	 // Magnetometer raw data   磁力计原始数据

// 拉低AD0引脚，让MPU6500的ID为0x68
// Lower the AD0 pin so that the ID of the MPU6500 is 0x68
void MPU_ADDR_CTRL(void)
{
	HAL_GPIO_WritePin(MPU_AD0_GPIO_Port, MPU_AD0_Pin, GPIO_PIN_RESET);
}

// 初始化MPU9250, 返回值:0,成功, 其他,错误代码
// Initialize MPU9250, return value :0, success, other, error code
uint8_t MPU9250_Init(void)
{
	MPU_ADDR_CTRL();
	MPU_IIC_Init();
	MPU_Delay_ms(10);

	uint8_t res = 0;
	// Reset MPU9250 //复位MPU9250
	MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X80);
	// Delay 100 ms //延时100ms
	MPU_Delay_ms(100);
	// Wake mpu9250 //唤醒MPU9250
	MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00);

	// Gyroscope sensor  陀螺仪传感器,±500dps=±500°/s ±32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
	MPU_Set_Gyro_Fsr(1);
	// Acceleration sensor 加速度传感器,±2g=±2*9.8m/s^2 ±32768 accel/32768*19.6=accel/1671.84
	MPU_Set_Accel_Fsr(0);
	// Set the sampling rate to 50Hz //设置采样率50Hz
	MPU_Set_Rate(50);

	// Turn off all interrupts //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR, MPU_INT_EN_REG, 0X00);
	// The I2C main mode is off //I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR, MPU_USER_CTRL_REG, 0X00);
	// Close the FIFO //关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR, MPU_FIFO_EN_REG, 0X00);
	// The INT pin is low, enabling bypass mode to read the magnetometer directly
	// INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
	MPU_Write_Byte(MPU9250_ADDR, MPU_INTBP_CFG_REG, 0X82);
	// Read the ID of MPU9250  读取MPU9250的ID
	res = MPU_Read_Byte(MPU9250_ADDR, MPU_DEVICE_ID_REG);
	printf("MPU6500 Read ID=0x%02X\n", res);
	// Check whether the device ID is correct 判断器件ID是否正确
	if (res == MPU6500_ID1 || res == MPU6500_ID2)
	{
		// Set CLKSEL,PLL X axis as reference //设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X01);
		// Acceleration and gyroscope both work //加速度与陀螺仪都工作
		MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT2_REG, 0X00);
		// Set the sampling rate to 50Hz //设置采样率为50Hz
		MPU_Set_Rate(50);
	}
	else
		return 1;
	// Read AK8963ID 读取AK8963ID
	res = MPU_Read_Byte(AK8963_ADDR, MAG_WIA);
	printf("AK8963 Read ID=0x%02X\n", res);
	if (res == AK8963_ID)
	{
		// Set AK8963 to single measurement mode 设置AK8963为单次测量模式
		MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11);
	}
	else
		return 2;
	return 0;
}

//设置MPU9250陀螺仪传感器满量程范围:fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功, 其他,设置失败
// Set the full range of the MPU9250 gyroscope sensor:fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// return value :0, the setting succeeds, other, the setting fails
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}

// 设置MPU9250加速度传感器满量程范围：fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功， 其他,设置失败
// Set the full range of the MPU9250 acceleration sensor: FSR :0,±2G; 1,±4g; 2,±8g; 3,±16g
// return value :0, the setting succeeds, other, the setting fails
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

// 设置MPU9250的数字低通滤波器，返回值:0,设置成功， 其他,设置失败
// Set the digital low-pass filter of the MPU9250. The return value is 0. The setting succeeds
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU9250_ADDR, MPU_CFG_REG, data); //设置数字低通滤波器
}

// 设置MPU9250的采样率(假定Fs=1KHz)， rate:4~1000(Hz)，返回值:0,设置成功 ， 其他,设置失败
// Set the sampling rate of MPU9250 (assuming Fs=1KHz), rate:4~1000(Hz),
// return value :0, the setting succeeds, other, the setting fails
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, data);
	return MPU_Set_LPF(rate / 2);
}

// 读取陀螺仪值(原始值), 返回值:0,成功, 其他,错误代码
// Read gyroscope value (original value), return value :0, success, other, error code
uint8_t MPU_Get_Gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}

// 读取加速度值(原始值), 返回值:0,成功, 其他,错误代码
// Read acceleration value (original value), return value :0, success, other, error code
uint8_t MPU_Get_Accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}

// 读取磁力计值(原始值)，返回值:0,成功，其他,错误代码
// Read magnetometer value (original value), return value :0, success, other, error code
uint8_t MPU_Get_Magnetometer(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(AK8963_ADDR, MAG_XOUT_L, 6, buf);
	if (res == 0)
	{
		*mx = ((uint16_t)buf[1] << 8) | buf[0];
		*my = ((uint16_t)buf[3] << 8) | buf[2];
		*mz = ((uint16_t)buf[5] << 8) | buf[4];
	}
	// AK8963每次读完以后都需要重新设置为单次测量模式
	// AK8963 needs to be reset to single measurement mode after each reading
	MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11);
	return res;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Millisecond delay function  毫秒级延时函数
void MPU_Delay_ms(uint16_t time)
{
	HAL_Delay(time);
}

// IIC连续写，buf为要写的数据地址。返回值:0,正常，其他,错误代码
// IIC continuous write, buF is the address of the data to be written.  Return value :0, normal, otherwise, error code
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0);
	if (MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	for (i = 0; i < len; i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if (MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC连续读, 数据保存到buf中。返回值:0,正常, 其他,错误代码
// IIC reads continuously and saves data to BUF.  Return value :0, normal, otherwise, error code
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0);
	if (MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1);
	MPU_IIC_Wait_Ack();
	while (len)
	{
		if (len == 1)
			*buf = MPU_IIC_Read_Byte(0);
		else
			*buf = MPU_IIC_Read_Byte(1);
		len--;
		buf++;
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC写一个字节, 返回值:0,正常, 其他,错误代码
// IIC writes a byte, return value :0, normal, otherwise, error code
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0);
	if (MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(data);
	if (MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}

// IIC读一个字节, 返回读到的数据
// IIC reads one byte and returns the read data
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1);
	MPU_IIC_Wait_Ack();
	res = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();
	return res;
}

// Read and print the data  读取并打印数据
void MPU9250_Read_Data_Handle(void)
{
	// Get accelerometer data  得到加速度传感器数据
	MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
	// Get the gyroscope data  得到陀螺仪数据
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	// Get magnetometer data   得到磁力计数据
	MPU_Get_Magnetometer(&magx, &magy, &magz);

	// 为了打印不太快，每10个数据打印一次。
	// In order not to print too fast, print every 10 pieces of data
	static uint8_t show = 0;
	show++;
	if (show > 10)
	{
		show = 0;
		printf("accel:%d, %d, %d\n", aacx, aacy, aacz);
		printf("gyro:%d, %d, %d\n", gyrox, gyroy, gyroz);
		printf("mag:%d, %d, %d\n", magx, magy, magz);
	}
}

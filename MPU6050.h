/*
 * MPU6050.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Enes Tayfun CICEK
 *
 * 	Copyright (C) 2021 enescicek.com
 *
 * 	This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 *	of the GNU General Public License version 3 as published by the Free Software Foundation.
 * 	This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 * 	or indirectly by this software, read more about this on the GNU General Public License.
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
							               //Kullandiginiz kartin temel header dosyasini yazmalisiniz, burada "stm32l4xx.hal.h" dir.
#include "stm32l4xx_hal.h"  //You can choose which STM32 family you are using
#include <math.h>

extern I2C_HandleTypeDef hi2c1; // Kullandiginiz I2C birimini girmelisiniz. Örn; hi2c1, hi2c2....
#define I2C_UNIT &hi2c1			// Kullandiginiz I2C birimini girmelisiniz. Örn; &hi2c1, &hi2c2....

#define MPU6050_ADDRESS 0x68<<1 // 7 Bit I2C modu kullanildigi icin 8 bitlik adres 1 bit sola otelenmistir

typedef enum{
	SMPRT_DIV		=	0x19,
	CONFIG			=	0x1A,
	GYRO_CONFIG	 	=	0x1B,
	ACCEL_CONFIG	= 	0x1C,
	ACCEL_XOUT_H 	=	0x3B,
	ACCEL_XOUT_L 	=	0x3C,
	ACCEL_YOUT_H 	=	0x3D,
	ACCEL_YOUT_L 	=	0x3E,
	ACCEL_ZOUT_H 	=	0x3F,
	ACCEL_ZOUT_L 	=	0x40,
	TEMP_OUT_H 		=	0x41,
	TEMP_OUT_L 		=	0x42,
	GYRO_XOUT_H 	=	0x43,
	GYRO_XOUT_L 	=	0x44,
	GYRO_YOUT_H 	=	0x45,
	GYRO_YOUT_L 	=	0x46,
	GYRO_ZOUT_H 	=	0x47,
	GYRO_ZOUT_L 	=	0x48,
	PWR_MGMT_1 		=	0x6B,
	WHO_AM_I		=	0x75,

}Mpu6050_Addres;

/*
 * 	Gyro_Scale jiroskop olcum araligini belirleyen parametredir.
 *  FS_SEL_250 secenegi -250 derece/saniye ile +250 derece/saniye arasinda acisal hiz olcumu yapar
 */
typedef enum{
	FS_SEL_250		=	0x00,
	FS_SEL_500		=	0x01,
	FS_SEL_1000		=	0x02,
	FS_SEL_2000		=	0x03,
}Gyro_Scale;
/*
 * 	Accel_Scale ivmeolcer olcum araligini belirleyen parametredir.
 *  AFS_SEL_2G secenegi -2G  ile +2G arasinda ivme olcumu yapar.
 */
typedef enum{
	AFS_SEL_2G		=	0x00,
	AFS_SEL_4G		=	0x01,
	AFS_SEL_8G		=	0x02,
	AFS_SEL_16G		=	0x03,
}Accel_Scale;
/*
 * 	Filter_Scale jiroskop ve ivmeolcer verilerini filtrelen,
 * 		 dijital alcak geçiren bir filtrenin gecirme araligini belirleyen bir parametredir.
 *  DLPF_CFG_0 secenegi ivmeolcere 260 Hz, jiroskopa 256 Hz filtre uygular.
 *  Dokuman Sayfa No: 13
 */
typedef enum{
	DLPF_CFG_0		=	0x00,
	DLPF_CFG_1		=	0x01,
	DLPF_CFG_2		=	0x02,
	DLPF_CFG_3		=	0x03,
	DLPF_CFG_4		=	0x04,
	DLPF_CFG_5		=	0x05,
	DLPF_CFG_6		=	0x06,
	DLPF_CFG_7		=	0x07,
}Filter_Scale;

typedef struct{
	uint8_t 		buffer[14],
					control,
					reg_add;
	float			gyro_sens;

}Mpu6050;

class MPU6050{
public:
	Mpu6050 mpu6050;

	void Init(uint8_t gyro_select, uint8_t accel_select, uint8_t filter_select);
	void Imu_Read();
	void Imu_Raw_Values(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp);
	void Imu_Calibration(int iter, float* cax, float* cay, float* caz, float* cgx, float* cgy, float* cgz);

};


#endif /* INC_MPU6050_H_ */

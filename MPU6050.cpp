/*
 * 	MPU6050.cpp
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

#include "MPU6050.h"


void MPU6050::Init(uint8_t gyro_select, uint8_t accel_select, uint8_t filter_select){

	HAL_I2C_Mem_Read(I2C_UNIT, MPU6050_ADDRESS, WHO_AM_I, 1, &mpu6050.control, 1, 100);

	if(mpu6050.control == (MPU6050_ADDRESS>>1)){

		//PowerManagement 1 Register
		mpu6050.reg_add = 0x00;
		HAL_I2C_Mem_Write(I2C_UNIT, MPU6050_ADDRESS, PWR_MGMT_1, 1, &mpu6050.reg_add, 1, HAL_MAX_DELAY);

		//Gyro Configuration
		mpu6050.reg_add = gyro_select<<3;
		HAL_I2C_Mem_Write(I2C_UNIT, MPU6050_ADDRESS, GYRO_CONFIG, 1, &mpu6050.reg_add, 1, HAL_MAX_DELAY);

		//Accelerometer Configuration
		mpu6050.reg_add = accel_select<<3;
		HAL_I2C_Mem_Write(I2C_UNIT, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &mpu6050.reg_add, 1, HAL_MAX_DELAY);

		//Gyro Digital Filter
		mpu6050.reg_add = filter_select<<3;
		HAL_I2C_Mem_Write(I2C_UNIT, MPU6050_ADDRESS, CONFIG, 1, &mpu6050.reg_add, 1, HAL_MAX_DELAY);

		switch(gyro_select){
		case 0:
			mpu6050.gyro_sens = 131;
			break;
		case 1:
			mpu6050.gyro_sens = 65.5;
			break;
		case 2:
			mpu6050.gyro_sens = 32.8;
			break;
		case 3:
			mpu6050.gyro_sens = 16.4;
			break;
		}

		HAL_Delay(10);
	}

}


void MPU6050::Imu_Read(){
	mpu6050.reg_add = 0x3B;
	HAL_I2C_Master_Transmit(I2C_UNIT,MPU6050_ADDRESS,&mpu6050.reg_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(I2C_UNIT,MPU6050_ADDRESS,mpu6050.buffer,14,HAL_MAX_DELAY);
}

void MPU6050::Imu_Calibration(int iter, float* cax, float* cay, float* caz, float* cgx, float* cgy, float* cgz){
	int16_t ax, ay, az, gx, gy,gz,temp;
	for(int i=0; i<iter; i++){
		Imu_Raw_Values(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		*cax += ax;
		*cay += ay;
		*caz += az;
		*cgx += gx;
		*cgy += gy;
		*cgz += gz;
	}
	*cax /= iter;
	*cay /= iter;
	*caz /= iter;
	*cgx /= iter;
	*cgy /= iter;
	*cgz /= iter;
}

void MPU6050::Imu_Raw_Values(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp){

	Imu_Read();
	*ax		= (mpu6050.buffer[0] << 8 | mpu6050.buffer[1]);
	*ay 	= (mpu6050.buffer[2] << 8 | mpu6050.buffer[3]);
	*az 	= (mpu6050.buffer[4] << 8 | mpu6050.buffer[5]);
	*temp 	= (mpu6050.buffer[6] << 8 | mpu6050.buffer[7]);
	*gx 	= (mpu6050.buffer[8] << 8 | mpu6050.buffer[9])/mpu6050.gyro_sens;
	*gy 	= (mpu6050.buffer[10] << 8 | mpu6050.buffer[11])/mpu6050.gyro_sens;
	*gz 	= (mpu6050.buffer[12] << 8 | mpu6050.buffer[13])/mpu6050.gyro_sens;
}

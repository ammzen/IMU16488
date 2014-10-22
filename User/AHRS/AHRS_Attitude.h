#ifndef __AHRS_ATTITUDE_H
#define __AHRS_ATTITUDE_H

#include "stm32f10x.h"

void init_quaternion(void);
void factored_quaternion(float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Get_Attitude(void);
void get_compass_bias(void);
void compass_calibration(void);
void Read_Mag(void);
void gyro_calibration(void);
void GetData(void);
void GetRawData(void);
float GET_NOWTIME(void);


#endif

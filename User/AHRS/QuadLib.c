#include "QuadLib.h"


Quaternion quad_times(Quaternion a, Quaternion b)
{
	Quaternion temp;
	temp.q0 = a.q0 * b.q0 - a.q1 * b.q1 - a.q2 * b.q2 - a.q3 * b.q3;
	temp.q1 = a.q0 * b.q1 + a.q1 * b.q0 + a.q2 * b.q3 - a.q3 * b.q2;
	temp.q2 = a.q0 * b.q2 - a.q1 * b.q3 + a.q2 * b.q0 + a.q3 * b.q1;
	temp.q3 = a.q0 * b.q3 + a.q1 * b.q2 - a.q2 * b.q1 + a.q3 * b.q0;
	quad_norm(&temp);
	return temp;
}

Quaternion quad_add(Quaternion a, Quaternion b)
{
	Quaternion temp;
	temp.q0 = a.q0 + b.q0;
	temp.q1 = a.q1 + b.q1;
	temp.q2 = a.q2 + b.q2;
	temp.q3 = a.q3 + b.q3;
	quad_norm(&temp);
	return temp;
}

Quaternion quad_minus(Quaternion a, Quaternion b)
{
	Quaternion temp;
	temp.q0 = a.q0 - b.q0;
	temp.q1 = a.q1 - b.q1;
	temp.q2 = a.q2 - b.q2;
	temp.q3 = a.q3 - b.q3;
	quad_norm(&temp);
	return temp;
}

void quad_norm(Quaternion *a)
{
	float recipNorm;
	recipNorm = invSqrt(a->q0 * a->q0 + a->q1 * a->q1 + a->q2 * a->q2 + a->q3 * a->q3);
	a->q0 *= recipNorm;
	a->q1 *= recipNorm;
	a->q2 *= recipNorm;
	a->q3 *= recipNorm;
}

EulerAngle quad2euler(Quaternion a)
{
	EulerAngle temp;
	temp.yaw = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
	temp.pitch = asin(-2 * (q1*q3 - q0*q2));
	temp.roll = atan2(2 * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
	return temp;
}

Quaternion euler2quad(EulerAngle a)
{
	Quaternion temp;
	temp.q0 = cos(0.5*a.Roll)*cos(0.5*a.Pitch)*cos(0.5*a.Yaw) + sin(0.5*a.Roll)*sin(0.5*a.Pitch)*sin(0.5*a.Yaw);  //w
	temp.q1 = sin(0.5*a.Roll)*cos(0.5*a.Pitch)*cos(0.5*a.Yaw) - cos(0.5*a.Roll)*sin(0.5*a.Pitch)*sin(0.5*a.Yaw);  //x   绕x轴旋转是pitch
	temp.q2 = cos(0.5*a.Roll)*sin(0.5*a.Pitch)*cos(0.5*a.Yaw) + sin(0.5*a.Roll)*cos(0.5*a.Pitch)*sin(0.5*a.Yaw);  //y   绕y轴旋转是roll
	temp.q3 = cos(0.5*a.Roll)*cos(0.5*a.Pitch)*sin(0.5*a.Yaw) - sin(0.5*a.Roll)*sin(0.5*a.Pitch)*cos(0.5*a.Yaw);  //z   绕z轴旋转是Yaw
	quad_norm(&temp);
	return temp;
}


/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
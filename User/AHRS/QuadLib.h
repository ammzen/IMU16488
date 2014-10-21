#ifndef __QUADLIB_H
#define __QUADLIB_H

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}Quaternion;

typedef struct
{
	float yaw;
	float pitch;
	float roll;
}EulerAngle;


Quaternion quad_times(Quaternion a, Quaternion b);
Quaternion quad_add(Quaternion a, Quaternion b);
Quaternion quad_minus(Quaternion a, Quaternion b);
/* quad_norm(&a) */
void quad_norm(Quaternion *a);
/** 转动顺序为Z—Y-X **/
EulerAngle quad2euler(Quaternion a);
Quaternion euler2quad(EulerAngle a);

#endif

#include "AHRS_Attitude.h"
#include "bsp_spi_adis.h"
#include "bsp_usart1.h"
#include "Kalman.h"
#include "QuadLib.h"
#include "bsp_led.h"
#include <math.h>

__IO uint16_t *p;
__IO float * TRUEVALUE;
__IO float DELTANG[3];
__IO float accel[3];
__IO float mag[3];
__IO float gyro[3];
float q0, q1, q2, q3;
float vx,vy,vz,px,py,pz;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float halfT;
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
float maxMagX = 0;
float minMagX = 0;
float maxMagY = 0;
float minMagY = 0;
float maxMagZ = 0;
float minMagZ = 0;
float MXgain = 1;
float MYgain = 1;
float MZgain = 1;
float MXoffset = 0;
float MYoffset = 0;
float MZoffset = 0;

typedef struct
{
	float AXoffset;
	float AYoffset;
	float AZoffset;
	float Params[3][3]; 
}AccCalibParams;

AccCalibParams AccCalib;

extern __IO float Pitch, Roll, Yaw;
extern __IO int KEYDOWN;

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
#define Kp 2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = 0.1;								// 2 * proportional gain (Kp)

void init_quaternion()
{
	float init_Yaw, init_Pitch, init_Roll;

	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	init_ax = *(TRUEVALUE + 0);
	init_ay = *(TRUEVALUE + 1);
	init_az = *(TRUEVALUE + 2);
	p = SPI_READ_MAGN();
	TRUEVALUE = CONVERT_MAGN(p);
	init_mx = *(TRUEVALUE + 0);
	init_my = *(TRUEVALUE + 1);
	init_mz = *(TRUEVALUE + 2);

	init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.295780转化为角度
	init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
//	init_Yaw  = atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
//	                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//AHRSupdata 类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度				            
//	init_Roll =  atan2(init_ay, init_az);    //算出的单位是弧度，如需要观察则应乘以57.295780转化为角度
//	init_Pitch=  atan2(init_ax, init_az);              //init_Pitch = asin(ay / 1);      
//	init_Yaw  =  -atan2(init_my, init_mx);//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度				            
    init_Yaw  = -atan2(init_my*cos(init_Pitch) - init_mz*sin(init_Pitch),
                       init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch));//MadgwickAHRS类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度				            
	//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
	q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
	q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
	q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
	q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw
//	q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
//	q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
//	q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
//	q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

	init_Roll  = init_Roll * 57.295780;	 //弧度转角度
	init_Pitch = init_Pitch * 57.295780;
	init_Yaw   = init_Yaw * 57.295780;
	if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //将Yaw的范围转成0-360
	if(init_Yaw > 360){init_Yaw = init_Yaw - 360;} 	    
	printf("由初始化四元数得到:%9.6f,%9.6f,%9.6f,%9.6f,yaw,pitch,roll:%8.3f,%8.3f,%8.3f\n\r",q0,q1,q2,q3, init_Yaw, init_Pitch, init_Roll);
//    Yaw = init_Yaw;
//    Pitch = init_Pitch;
//    Roll = init_Roll;
}

void factored_quaternion(float ax, float ay, float az, float mx, float my, float mz)
{
	// theta->Pitch rotate by Y; phi->Roll rotate by X; psi->Yaw rotate by Z;
	float sin_theta, cos_theta, sin_phi, cos_phi, sin_psi, cos_psi;
	float sin_half_theta, cos_half_theta, sin_half_phi, cos_half_phi, sin_half_psi, cos_half_psi;
	// q_e->Pitch(Elevation); q_r->Roll(Roll/Bank); q_a->Yaw(Azimuth); q_est->all three rotation
	Quaternion q_e, q_r, q_a, q_est;
	// mag_b->magnetic field measurement vector in the body coordinate
	// mag_e->rotate mag_b into intermediate Earth coordinate
	// nx ny->the known local normalized magnetic field vector
	Quaternion mag_b, mag_e, acc_g;
	// Singularity Avoidance in Impletation
	Quaternion q_est_alt, q_alpha, mag_b_offset, acc_g_offset;
	float nx, ny, Nx, Ny, Mx, My;
   	int cos_theta_tiny_tag = 0;
    EulerAngle euler;
	
	float recipNorm;
	// Normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;   

	// Normalise magnetometer measurement
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;
    	
    // Convert Sensor coordinate to North-East-Down
	az = -az;
	mz = -mz;

	// Elevation Quaternion
	sin_theta = ax;
	cos_theta = sqrt(1 - sin_theta*sin_theta);
	sin_half_theta = (sin_theta >= 0 ? 1:-1) * sqrt(0.5 * (1 - cos_theta));
	cos_half_theta = sqrt(0.5 * (1 + cos_theta));
	q_e.q0 = cos_half_theta;
	q_e.q1 = 0;
	q_e.q2 = sin_half_theta;
	q_e.q3 = 0;

	acc_g = quad_init(0, ax, ay, az);
	q_alpha = quad_init(0.9848, 0 , 0.1736, 0);
	acc_g_offset = quad_times(quad_times(q_alpha, acc_g), quad_invert(q_alpha));
	mag_b_offset = quad_times(quad_times(q_alpha, mag_b), quad_invert(q_alpha));

	// Roll Quaternion
	if (cos_theta < 0.1)
	{
		sin_phi = 0;
		cos_phi = 1;
//		sin_theta = acc_g_offset.q1;
//		cos_theta = sqrt(1 - sin_theta*sin_theta);
//		sin_phi = -acc_g_offset.q2 / cos_theta;
//		cos_phi = -acc_g_offset.q3 / cos_theta;
//		cos_theta_tiny_tag = 1;
	}
	else
	{
		sin_phi = -ay / cos_theta;
		cos_phi = -az / cos_theta;
	}
	sin_half_phi = (sin_phi >= 0 ? 1:-1) * sqrt(0.5 * (1 - cos_phi));
	cos_half_phi = sqrt(0.5 * (1 + cos_phi));
	q_r.q0 = cos_half_phi;
	q_r.q1 = sin_half_phi;
	q_r.q2 = 0;
	q_r.q3 = 0;

	// Azimuth Quaternion
	mag_b.q0 = 0;
	mag_b.q1 = mx;
	mag_b.q2 = my;
	mag_b.q3 = mz;
	mag_e = quad_times(quad_times(q_r, mag_b), quad_invert(q_r));
	mag_e = quad_times(quad_times(q_e, mag_e), quad_invert(q_e));

	recipNorm = invSqrt(mag_e.q1 * mag_e.q1 + mag_e.q2 * mag_e.q2);
	Mx = mag_e.q1 * recipNorm; 
	My = mag_e.q2 * recipNorm; 
	Nx = -0.1556;
	Ny =  0.9878;
	// Nx =  0;
	// Ny =  1;
	cos_psi =  Mx * Nx + My * Ny;
	sin_psi = -My * Nx + Mx * Ny;
	sin_half_psi = (sin_psi >= 0 ? 1:-1) * sqrt(0.5 * (1 - cos_psi));
	cos_half_psi = sqrt(0.5 * (1 + cos_psi));
	q_a.q0 = cos_half_psi;
	q_a.q1 = 0;
	q_a.q2 = 0;
	q_a.q3 = sin_half_psi;

	// Three rotations represent the orientation
	q_est = quad_times(quad_times(q_a, q_e), q_r);
	if (cos_theta_tiny_tag)
	{
		q_est = quad_times(q_est, q_alpha);
	}
    euler = quad2euler(q_est);
    q0 = q_est.q0;
    q1 = q_est.q1;
    q2 = q_est.q2;
    q3 = q_est.q3;
    Yaw = euler.yaw * 57.295780;
	if(Yaw < 0)  {Yaw = Yaw + 360;}      //将Yaw的范围转成0-360
	if(Yaw > 360){Yaw = Yaw - 360;} ;
    Pitch = euler.pitch * 57.295780;
    Roll = euler.roll * 57.295780;

}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//    printf("aX=%7.2f,aY=%7.2f,aZ=%7.2f,mX=%7.2f,mY=%7.2f,mZ=%7.2f,gX=%7.2f,gY=%7.2f,gZ=%7.2f\r\n",ax, ay, az, mx, my, mz, gx, gy, gz);

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * (1.0f / sampleFreq);
//	q1 += qDot2 * (1.0f / sampleFreq);
//	q2 += qDot3 * (1.0f / sampleFreq);
//	q3 += qDot4 * (1.0f / sampleFreq);
//    halfT=GET_NOWTIME();
    q0 += qDot1 * halfT *2;
	q1 += qDot2 * halfT *2;
	q2 += qDot3 * halfT *2;
	q3 += qDot4 * halfT *2;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
    
    /* Madgwick 论文上的转换公式*/
//    Yaw   = atan2(2*q1q2 + 2*q0q3, 2*(q0q0 + q1q1) - 1) * 57.295780; //偏航角，绕z轴转动	
//    if(Yaw < 0 ){Yaw = Yaw + 360;}
//	if(Yaw > 360 ){Yaw = Yaw - 360;}
//	Roll = asin(2*(q0q2 - q1q3)) * 57.295780; 	 
//    Pitch  = atan2(2*(q2q3 + q0q1), 2*(q0q0 + q3q3) - 1) * 57.295780;   

    /* AHRSupdate 上的转换公式*/
    Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
    Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动
    

	//printf("四元数收敛过程：Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*方便之后的程序使用，减少计算时间*/
   //auxiliary variables to reduce number of repeated operations，
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
//   // integral error scaled integral gain
//   exInt = exInt + ex*Ki;		
//   eyInt = eyInt + ey*Ki;
//   ezInt = ezInt + ez*Ki;
//   // adjusted gyroscope measurements
//   gx = gx + Kp*ex + exInt;
//   gy = gy + Kp*ey + eyInt;
//   gz = gz + Kp*ez + ezInt;

//   halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise，四元数更新算法
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z
        
/*Y轴指向正北，由四元数计算出Pitch  Roll  Yaw，只需在需要PID控制时才将四元数转化为欧拉角
乘以57.295780是为了将弧度转化为角度*/
	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
    Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.295780是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //偏航角，绕z轴转动

//	printf("halfT=%f  \n\r", halfT);
//    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}

void Get_Attitude(void)
{
    //init_quaternion();
//    factored_quaternion(init_ax, init_ay, init_az, init_mx, init_my, init_mz);
    MadgwickAHRSupdate(init_gx/57.29578, init_gy/57.29578, init_gz/57.29578, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
    //AHRSupdate(init_gx/57.29578, init_gy/57.29578, init_gz/57.29578, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
//    halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半
//    printf("Time: %10.4f", halfT * 2);
}

/**
 * 初始化各个传感器的校准系数
 */
void init_calibparams(void)
{
	AccCalib.AXoffset = 0;
	AccCalib.AYoffset = 0;
	AccCalib.AZoffset = 0;
	AccCalib.Params[0][0] = 1;
	AccCalib.Params[0][1] = 0;
	AccCalib.Params[0][2] = 0;
	AccCalib.Params[1][0] = 0;
	AccCalib.Params[1][1] = 1;
	AccCalib.Params[1][2] = 0;
	AccCalib.Params[2][0] = 0;
	AccCalib.Params[2][1] = 0;
	AccCalib.Params[2][2] = 1;
	/*
	|S_ax   K_ax1  K_ax2|   K_ax1、K_ay1、K_az1 是安装误差系数
	|K_ay1  S_ay   K_ay2|   S_ax、S_ay、S_az 是刻度因数
	|K_az1  K_az2  S_az |	AXoffset、AYoffset、AZoffset 是零偏值
	 */
}

void get_compass_bias(void)
{
  Read_Mag();

  if(init_mx > maxMagX)
  maxMagX = init_mx;
  if(init_mx < minMagX)
  minMagX = init_mx;

  if(init_my > maxMagY)
  maxMagY = init_my;
  if(init_my < minMagY)
  minMagY = init_my;

  if(init_mz > maxMagZ)
  maxMagZ = init_mz;
  if(init_mz < minMagZ)
  minMagZ = init_mz;
//  printf("\n\r  maxMagX=%f, minMagX=%f, maxMagY=%f, minMagY=%f, maxMagZ=%f, minMagZ=%f ", 
//          maxMagX, minMagX, maxMagY, minMagY, maxMagZ, minMagZ);  
}
/*******************************************************************************
磁力计空间校准
*******************************************************************************/
void compass_calibration(void)
{ //将有最大响应的轴的增益设为1
  if(((maxMagX - minMagX) >= (maxMagY - minMagY)) && ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
  {
    MXgain = 1.0;
	MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
	MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);
	MXoffset = -0.5 * (maxMagX + minMagX);
	MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
	MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);	 
  }
  if(((maxMagY - minMagY) > (maxMagX - minMagX)) && ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
  {
    MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
	MYgain = 1.0;
	MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
	MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
	MYoffset = -0.5 * (maxMagY + minMagY);
	MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);    
  }
  if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) && ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
  {
    MXgain = (maxMagZ - minMagZ) / (maxMagX - minMagX);
	MYgain = (maxMagZ - minMagZ) / (maxMagY - minMagY);
	MZgain = 1.0;
	MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
	MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
	MZoffset = -0.5 * (maxMagZ + minMagZ);    
  }
  printf(" \n\r  MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f", 
          MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);         
}
/**
 * 加速度计的误差校准
 * 将传感器依次按照要求的6个姿态放置，采集相应的数据，计算加速度计的零偏及安装误差系数
 */
void get_acc_bias(void)
{
	float Ax[6], Ay[6], Az[6];
	printf("\n\r即将进行加速度计校准程序，请按提示进行操作\n\r");

	printf("\n\r将加速度计的X、Y、Z轴按照 东 天 南 放置，放置好后按2键采集数据\n\r");
	while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[0] = *(TRUEVALUE + 0);
	Ay[0] = *(TRUEVALUE + 1);
	Az[0] = *(TRUEVALUE + 2);
	printf("\n\r将加速度计的X、Y、Z轴按照 东 北 天 放置，放置好后按2键采集数据\n\r");
	KEYDOWN = 0;
    while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
    p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[1] = *(TRUEVALUE + 0);
	Ay[1] = *(TRUEVALUE + 1);
	Az[1] = *(TRUEVALUE + 2);
	printf("\n\r将加速度计的X、Y、Z轴按照 地 东 南 放置，放置好后按2键采集数据\n\r");
	KEYDOWN = 0;
	while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[2] = *(TRUEVALUE + 0);
	Ay[2] = *(TRUEVALUE + 1);
	Az[2] = *(TRUEVALUE + 2);
	printf("\n\r将加速度计的X、Y、Z轴按照 西 地 南 放置，放置好后按2键采集数据\n\r");
	KEYDOWN = 0;
	while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[3] = *(TRUEVALUE + 0);
	Ay[3] = *(TRUEVALUE + 1);
	Az[3] = *(TRUEVALUE + 2);
	printf("\n\r将加速度计的X、Y、Z轴按照 天 西 南 放置，放置好后按2键采集数据\n\r");
	KEYDOWN = 0;
	while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[4] = *(TRUEVALUE + 0);
	Ay[4] = *(TRUEVALUE + 1);
	Az[4] = *(TRUEVALUE + 2);
	printf("\n\r将加速度计的X、Y、Z轴按照 南 西 地 放置，放置好后按2键采集数据\n\r");
	KEYDOWN = 0;
	while(!KEYDOWN)  //按键2按下则停止校准
	{}
	LED2_TOGGLE;
    Delay_ms(1000);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	Ax[5] = *(TRUEVALUE + 0);
	Ay[5] = *(TRUEVALUE + 1);
	Az[5] = *(TRUEVALUE + 2);

	printf("\n\r加速度计校准完毕\n\r");
	AccCalib.AXoffset = (Ax[0]+Ax[1]+Ax[3]+Ax[5]) / 4;
	AccCalib.AYoffset = (Ay[1]+Ay[2]+Ay[4]+Ay[5]) / 4;
	AccCalib.AZoffset = (Az[0]+Az[2]+Az[3]+Az[4]) / 4;
	AccCalib.Params[0][0] = (Ax[2]-Ax[4]) / 2;
	AccCalib.Params[0][1] = (Ax[3]-Ax[0]) / 2;
	AccCalib.Params[0][2] = (Ax[5]-Ax[1]) / 2;
	AccCalib.Params[1][0] = (Ay[2]-Ay[4]) / 2;
	AccCalib.Params[1][1] = (Ay[3]-Ay[0]) / 2;
	AccCalib.Params[1][2] = (Ay[5]-Ay[1]) / 2;
	AccCalib.Params[2][0] = (Az[2]-Az[4]) / 2;
	AccCalib.Params[2][1] = (Az[3]-Az[0]) / 2;
	AccCalib.Params[2][2] = (Az[5]-Az[1]) / 2;
	printf("\n\r校准矩阵如下：\n\r");
	printf("%f    %f    %f\n\r%f    %f    %f\n\r%f    %f    %f\n\r",
	AccCalib.Params[0][0], AccCalib.Params[0][1], AccCalib.Params[0][2], 
	AccCalib.Params[1][0], AccCalib.Params[1][1], AccCalib.Params[1][2], 
	AccCalib.Params[2][0], AccCalib.Params[2][1], AccCalib.Params[2][2]);
	printf("\n\rAXoffset=%f, AYoffset=%f, AZoffset=%f\n\r", AccCalib.AXoffset, AccCalib.AYoffset, AccCalib.AZoffset);
	/*
	-0.996969    0.006343    -0.015940
	-0.001398    -0.999080    -0.006122
	-0.011063    0.015296    -0.998357
	
	AXoffset=-0.002932, AYoffset=-0.002300, AZoffset=-0.002448
	 */
}
void gyro_calibration(void)
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendTwoByte(0x8003);
  SPI_FLASH_SendTwoByte(0x8201);
  SPI_FLASH_SendTwoByte(0x8300);
  SPI_FLASH_SendTwoByte(0x8000);
  SPI_FLASH_CS_HIGH();
}
/*******************************************************************************
磁力计数据读取	
*******************************************************************************/
void Read_Mag(void)
{
	p = SPI_READ_MAGN();
	TRUEVALUE = CONVERT_MAGN(p);
	mag[0] = *(TRUEVALUE + 0);
	mag[1] = *(TRUEVALUE + 1);
	mag[2] = *(TRUEVALUE + 2);

	init_mx = mag[0];					
    init_my = mag[1];
    init_mz = mag[2];
}

void GetData(void)
{
    halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半

	p = SPI_READ_GYRO();
	TRUEVALUE = CONVERT_GYRO(p);
	init_gx = *(TRUEVALUE + 0);
	init_gy = *(TRUEVALUE + 1);
	init_gz = *(TRUEVALUE + 2);
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	init_ax = *(TRUEVALUE + 0);
	init_ay = *(TRUEVALUE + 1);
	init_az = *(TRUEVALUE + 2);
	init_ax = AccCalib.AXoffset + AccCalib.Params[0][0] * init_ax + AccCalib.Params[0][1] * init_ay + AccCalib.Params[0][2] * init_az;
	init_ay = AccCalib.AYoffset + AccCalib.Params[1][0] * init_ax + AccCalib.Params[1][1] * init_ay + AccCalib.Params[1][2] * init_az;
	init_az = AccCalib.AZoffset + AccCalib.Params[2][0] * init_ax + AccCalib.Params[2][1] * init_ay + AccCalib.Params[2][2] * init_az;
	p = SPI_READ_MAGN();
	TRUEVALUE = CONVERT_MAGN(p);
	init_mx = *(TRUEVALUE + 0) * MXgain + MXoffset;
	init_my = *(TRUEVALUE + 1) * MYgain + MYoffset;
	init_mz = *(TRUEVALUE + 2) * MZgain + MZoffset;
//    init_mx = 0;
//	init_my = 0;
//	init_mz = 0;
}
void GetRawData()
{
    DELTANG[0] = 0;
	DELTANG[1] = 0;
	DELTANG[2] = 0;
	p = SPI_READ_DELTANG();
	TRUEVALUE = CONVERT_DELTANG(p);
	DELTANG[0] = *(TRUEVALUE+0);
	DELTANG[1] = *(TRUEVALUE+1);
	DELTANG[2] = *(TRUEVALUE+2);        
	printf("DELTANG[0]=%.5f   DELTANG[1]=%.5f   DELTANG[2]=%.5f \r\n", *(TRUEVALUE+0), *(TRUEVALUE+1), *(TRUEVALUE+2));
	p = SPI_READ_GYRO();
	TRUEVALUE = CONVERT_GYRO(p);
	gyro[0] = *(TRUEVALUE + 0);
	gyro[1] = *(TRUEVALUE + 1);
	gyro[2] = *(TRUEVALUE + 2);
	printf("GYRO[0]=%.5f   GYRO[1]=%.5f   GYRO[2]=%.5f \r\n", *(TRUEVALUE+0), *(TRUEVALUE+1), *(TRUEVALUE+2));
	p = SPI_READ_ACCL();
	TRUEVALUE = CONVERT_ACCL(p);
	accel[0] = *(TRUEVALUE + 0);
	accel[1] = *(TRUEVALUE + 1);
	accel[2] = *(TRUEVALUE + 2);
	printf("ACCL[0]=%.5f   ACCL[1]=%.5f   ACCL[2]=%.5f \r\n", *(TRUEVALUE+0), *(TRUEVALUE+1), *(TRUEVALUE+2));
	p = SPI_READ_MAGN();
	TRUEVALUE = CONVERT_MAGN(p);
	mag[0] = *(TRUEVALUE + 0);
	mag[1] = *(TRUEVALUE + 1);
	mag[2] = *(TRUEVALUE + 2);
	printf("MAGN[0]=%.5f   MAGN[1]=%.5f   MAGN[2]=%.5f \r\n", *(TRUEVALUE+0), *(TRUEVALUE+1), *(TRUEVALUE+2));
}

//返回当前TIM3的计数值
float GET_NOWTIME(void)
{
    float temp=0 ;
	static uint32_t now=0; // 采样周期计数 单位 us

 	now = TIM3->CNT;//读高16位时间
   	TIM3->CNT=0;
    //TIM_Cmd(TIM3, DISABLE);
	temp = (float)now / 2000000.0f;          //换算成秒，再除以2得出采样周期的一半

	return temp;
}

//计算位移及速度
void Get_Position()
{
    float acc_sum;
    Quaternion acc_sensor, acc_geo, attitude;
    acc_sensor = quad_init(0, init_ax, init_ay, init_az);
    attitude = quad_init(q0, q1, q2, q3);
	acc_geo = quad_times(quad_times(attitude, acc_sensor), quad_invert(attitude));
    acc_sum = sqrt(init_ax*init_ax+init_ay*init_ay+init_az*init_az);
    if (acc_sum > 1.001 || acc_sum < 0.999)
    {
        vx += acc_geo.q1*9.8*halfT*2;
        vy += acc_geo.q2*9.8*halfT*2;
        px += vx*halfT*2;
        py += vy*halfT*2;
    }
    else
    {
        vx = vy = 0;
    }
//                printf("Vx=%8.2f  Vy=%8.2f  Px=%9.4f  Py=%9.4f\r\n",
//            vx*100,vy*100,px*100,py*100);

//    printf("acc_geo.q0=%8.2f, acc_geo.q1=%8.2f, acc_geo.q2=%8.2f, acc_geo.q3=%8.2f, sum=%f\r\n",
//           acc_geo.q0, acc_geo.q1*9.8, acc_geo.q2*9.8, acc_geo.q3*9.8,sqrt(init_ax*init_ax+init_ay*init_ay+init_az*init_az));
}




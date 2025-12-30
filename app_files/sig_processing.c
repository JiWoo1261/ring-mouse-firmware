#include "sig_processing.h"
#include "common.h"
#include "Drv_mpu.h"
#include "math.h"

#define APPLY_COMPFILTER  1

const float ALPHA = (0.1725/(0.1725+0.015));
const float ONE_M_ALPHA = (1-ALPHA);

double deltaT = 0;
float acc_resolution = 0.f;
float gyro_resolution = 0.f;
float mag_resolution = 0.f;
float rpy[3] = {0.f, 0.f, 0.f};
float lin_accel[3] = {0.f,};
float accel[3] = {0.,};
float gyro[3] = {0.,};
float magn[3] = {0.f,};
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float mag_bias[3] = {0.f,};
float mag_scale[3] = {1.f, 1.f, 1.f};
float mag_bias_factory[3] = {0.f,};
float a31 = 0.0f;
float a32 = 0.0f;
float a33 = 0.0f;
float an, ae, ad, gn, ge, gd, mn, me, md;
float xtemp = 0;
float ztemp = 0;
// Temperature
int16_t temperature_count = 0;  // temperature raw count output
float temperature = 0.f;        // Stores the real internal chip temperature in degrees Celsius
float magnetic_declination = -7.51;  // Japan, 24th June

double angles[3] = {0}, prevAngle2 = 0, prevAngle1 = 0, prevAngle0 = 0;
double angleAccels[3] = {0};


void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q)
{
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // short name local variable for readability
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double hx, hy;
	double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float beta = 0.866 * GyroMeasError;  // compute beta
	float zeta = 0.866 * GyroMeasDrift;
	
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Normalise accelerometer measurement
	double a_norm = ax * ax + ay * ay + az * az;
	if (a_norm == 0.) return;  // handle NaN
	recipNorm = 1.0 / sqrt(a_norm);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Normalise magnetometer measurement
	double m_norm = mx * mx + my * my + mz * mz;
	if (m_norm == 0.) return;  // handle NaN
	recipNorm = 1.0 / sqrt(m_norm);
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
	recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltaT;
	q1 += qDot2 * deltaT;
	q2 += qDot3 * deltaT;
	q3 += qDot4 * deltaT;

	// Normalise quaternion
	recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}


void compFilter(float AcX, float AcY, float AcZ, float GyX, float GyY, float GyZ, double dt, double angles[3], double angleAccels[3])
{
#if APPLY_COMPFILTER
    double angleAcX, angleTmpX, angleTmpZ;
    double angleAcY, angleTmpY;
    
    angleAcX = -atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2)));
    angleAcX *= RAD_TO_DEG;
    angleAcY = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2)));
    angleAcY *= RAD_TO_DEG;
    // Accelerometer can not derive z-axis angle 

    angleAccels[0] = angleAcX;
    angleAccels[1] = angleAcY;
    
    // Hold temporary angle to process complementary filter 
    angleTmpX = angles[0] + GyX * GYRO_RES * dt;
    angleTmpY = angles[1] + GyY * GYRO_RES * dt;
    angleTmpZ = angles[2] + GyZ * GYRO_RES * dt;
  
    // (Process complementary filter value) temporary angle 0.96 wight
    // the values from accelerometer 0.04 weight 
    angles[0] = ALPHA * angleTmpX + ONE_M_ALPHA * angleAcX;
    angles[1] = ALPHA * angleTmpY + ONE_M_ALPHA * angleAcY;
    angles[2] = angleTmpZ;    // z-axis only uses zyroscope 

#else
    angles[0] = angles[0] + GyX * GYRO_RES * dt;
    angles[1] = angles[1] + GyY * GYRO_RES * dt;
    angles[2] = angles[2] + GyZ * GYRO_RES * dt;
#endif
  
}


void integGyro(float GyX, float GyY, float GyZ, double dt, double angles[3])
{
    double angleTmpX, angleTmpZ;

    /* Hold temporary angle to integrate gyroscope data */
    angleTmpX = angles[0] + GyX * GYRO_RES * (double)dt;
    angleTmpZ = angles[2] + GyZ * GYRO_RES * (double)dt;

    angles[0] = angleTmpX;
    angles[2] = angleTmpZ;
}




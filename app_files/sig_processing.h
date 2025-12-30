#ifndef __SIG_PROCESSING_H__
#define __SIG_PROCESSING_H__

#include <stdint.h>
#include <stdbool.h>
#include "feature_config.h"

#define DEG_TO_PIXEL (20);

void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void compFilter(float AcX, float AcY, float AcZ, float GyX, float GyY, float GyZ, double dt, double angles[3], double angleAccels[3]);
void integGyro(float GyX, float GyY, float GyZ, double dt, double angles[3]);

extern double deltaT;
extern float acc_resolution;
extern float gyro_resolution;
extern float mag_resolution;
extern float rpy[3];
extern float lin_accel[3];
extern float accel[3];
extern float gyro[3];
extern float magn[3];
extern float q[4];
extern float mag_bias[3];
extern float mag_scale[3];
extern float mag_bias_factory[3];
extern float a31;
extern float a32;
extern float a33;
extern float an, ae, ad, gn, ge, gd, mn, me, md;
extern float xtemp;
extern float ztemp;
// Temperature
extern int16_t temperature_count;  // temperature raw count output
extern float temperature;        // Stores the real internal chip temperature in degrees Celsius
extern float magnetic_declination;  // Japan, 24th June

extern double angles[3], prevAngle2, prevAngle1, prevAngle0;
extern double angleAccels[3];

#endif /* __SIG_PROCESSING_H__ */

#ifndef APP_UICR_H__
#define APP_UICR_H__

#include <stdbool.h>

void saveCalibration(float accBias[3], float gyroBias[3], float magBias[3], float magScale[3], bool left_handed);
void loadCalibration(float accBias[3], float gyroBias[3], float magBias[3], float magScale[3], bool *left_handed);
bool isCalibrated(void);

#endif /* APP_UICR_H__ */
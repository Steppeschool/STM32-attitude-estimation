#ifndef INC_ORIENTATION_EST_H_
#define INC_ORIENTATION_EST_H_

#include "mpu9250.h"

#define SAMPLING_RATE  1000
#define GYRO_TO_ANGLE  0.000266
#define ALPHA  0.99
typedef struct{
	float yaw;
	float pitch;
	float roll;
} EulerAngles;

void estimate_euler_angles(mpu9250_data imu_data, EulerAngles *euler_angles);
void update_euler_angles_gyro(mpu9250_data imu_data, EulerAngles *euler_angles);
EulerAngles complementary_filter(mpu9250_data imu_data);
#endif

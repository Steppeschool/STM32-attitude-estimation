#ifndef INC_ORIENTATION_EST_H_
#define INC_ORIENTATION_EST_H_

#include "mpu9250.h"

#define SAMPLING_RATE  1000
#define GYRO_TO_ANGLE  0.000266
#define ALPHA  0.95
typedef struct{
	float yaw;
	float pitch;
	float roll;
}EulerAngles;

typedef struct{
	float s;
	float x;
	float y;
	float z;
}Quaternion;



void estimate_euler_angles(mpu9250_data imu_data, EulerAngles *euler_angles);
void update_euler_angles_gyro(mpu9250_data imu_data, EulerAngles *euler_angles);
EulerAngles complementary_filter(mpu9250_data imu_data);

void euler2quater(Quaternion *quaternion, EulerAngles euler_angles);
void complementary_filter_quater(Quaternion *quaternion,mpu9250_data imu_data);
void update_quater_gyro(Quaternion *quaternion, mpu9250_data imu_data);
#endif

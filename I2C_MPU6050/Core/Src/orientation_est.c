#include "orientation_est.h"
#include "mpu9250.h"
#include <math.h>

void estimate_euler_angles(mpu9250_data imu_data, EulerAngles *euler_angles)
{

	float sinroll = sin(euler_angles  -> roll);
		float cosroll = cos(euler_angles  -> roll );
		float sinpitch = sin(euler_angles -> pitch);
		float cospitch = cos(euler_angles -> pitch );

	float ay_az_sqrt = sqrt((float)imu_data.y_accel * imu_data.y_accel +
			imu_data.z_accel * imu_data.z_accel);
	float ax_ay_az_sqrt = sqrt(ay_az_sqrt * ay_az_sqrt + (float)imu_data.x_accel * imu_data.x_accel);
	euler_angles ->pitch = atan2(imu_data.x_accel, ay_az_sqrt);
	euler_angles ->roll  = 	atan2(-imu_data.y_accel, -imu_data.z_accel);
//	float sinroll = ((float)-imu_data.y_accel)/ ay_az_sqrt;
//	float cosroll = ((float)-imu_data.z_accel)/ ay_az_sqrt;
//	float sinpitch = (float)imu_data.x_accel / ax_ay_az_sqrt;
//	float cospitch = (float)ay_az_sqrt / ax_ay_az_sqrt;

	euler_angles ->yaw = atan2((float)imu_data.z_mag * sinroll - (float)imu_data.y_mag * cosroll,
				(float)imu_data.x_mag * cospitch + sinpitch *
				((float)imu_data.z_mag * cosroll + (float)imu_data.y_mag * sinroll));
}

void update_euler_angles_gyro(mpu9250_data imu_data, EulerAngles *euler_angles)
{
	float sinroll = sin(euler_angles  -> roll);
	float cosroll = cos(euler_angles  -> roll );
	float sinpitch = sin(euler_angles -> pitch);
	float cospitch = cos(euler_angles -> pitch );

	euler_angles -> roll += GYRO_TO_ANGLE / SAMPLING_RATE * ((float)imu_data.x_gyro +
			sinpitch / cospitch * ((float)imu_data.y_gyro * sinroll + (float)imu_data.z_gyro * cosroll));

	euler_angles -> pitch += GYRO_TO_ANGLE / SAMPLING_RATE * ((float)imu_data.y_gyro * cosroll
			- (float)imu_data.z_gyro * sinroll);

	euler_angles -> yaw += GYRO_TO_ANGLE / SAMPLING_RATE * ((float)imu_data.y_gyro * sinroll
			+ (float)imu_data.z_gyro * cosroll) / cospitch;
}


EulerAngles complementary_filter(mpu9250_data imu_data)
{
	static EulerAngles euler_angles_static;
	EulerAngles euler_angles;
	static uint8_t first_time = 0;
	if(first_time == 0)
	{
		estimate_euler_angles(imu_data, &euler_angles_static);
		first_time = 1;
		printf("first time \n");
	}
	else
	{
		euler_angles = euler_angles_static;
		update_euler_angles_gyro(imu_data, &euler_angles_static);
		estimate_euler_angles(imu_data, &euler_angles);
		euler_angles_static.pitch = ALPHA * euler_angles_static.pitch + (1 - ALPHA) * euler_angles.pitch;
		euler_angles_static.roll = ALPHA * euler_angles_static.roll + (1 - ALPHA) * euler_angles.roll;
		euler_angles_static.yaw = ALPHA * euler_angles_static.yaw + (1 - ALPHA) * euler_angles.yaw;
	}

	return euler_angles_static;
}

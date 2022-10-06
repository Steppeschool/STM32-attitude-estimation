#include "orientation_est.h"
#include "mpu9250.h"
#include <math.h>

static void norm_quaternion(Quaternion *quat)
{
	float quat_norm = sqrt(quat->s * quat->s + quat->x * quat->x +
			quat->y * quat->y + quat->z * quat->z);
	quat ->s/=quat_norm;
	quat ->x/=quat_norm;
	quat ->y/=quat_norm;
	quat ->z/=quat_norm;
}

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

void euler2quater(Quaternion *quaternion, EulerAngles euler_angles)
{
	float cospitch = cos(euler_angles.pitch / 2);
	float sinpitch = sin(euler_angles.pitch / 2);
	float cosroll = cos(euler_angles.roll / 2);
	float sinroll = sin(euler_angles.roll / 2);
	float cosyaw = cos(euler_angles.yaw / 2);
	float sinyaw = sin(euler_angles.yaw / 2);

	quaternion->s =  cospitch * cosroll * cosyaw + sinpitch * sinroll * sinyaw;
	quaternion->x =  cospitch * sinroll * cosyaw - sinpitch * cosroll * sinyaw;
	quaternion->y =  sinpitch * cosroll * cosyaw + cospitch * sinroll * sinyaw;
	quaternion->z =  cospitch * cosroll * sinyaw - sinpitch * sinroll * cosyaw;
}
void update_quater_gyro(Quaternion *quaternion, mpu9250_data imu_data)
{
	Quaternion quat_old, gyro_data;
	quat_old = *quaternion;

	gyro_data.s = 0;
	gyro_data.x = GYRO_TO_ANGLE  * (float)imu_data.x_gyro/ (SAMPLING_RATE * 2);
	gyro_data.y = GYRO_TO_ANGLE  * (float)imu_data.y_gyro/ (SAMPLING_RATE * 2);
	gyro_data.z = GYRO_TO_ANGLE  * (float)imu_data.z_gyro/ (SAMPLING_RATE * 2);

	quaternion->s -=  quat_old.x * gyro_data.x + quat_old.y * gyro_data.y +
			quat_old.z * gyro_data.z;
	quaternion->x +=  quat_old.s * gyro_data.x + quat_old.y * gyro_data.z -
			quat_old.z * gyro_data.y;
	quaternion->y +=  quat_old.s * gyro_data.y - quat_old.x * gyro_data.z +
			quat_old.z * gyro_data.x;
	quaternion->z +=  quat_old.s * gyro_data.z + quat_old.x * gyro_data.y -
			quat_old.y * gyro_data.x;
	//norm_quaternion(quaternion);
}

void complementary_filter_quater(Quaternion *quaternion, mpu9250_data imu_data)
{
	static EulerAngles euler_angles;
//	static Quaternion quater_result;
	static Quaternion quater_temp;
	static uint8_t first_time = 0;
	if(first_time == 0)
	{
		estimate_euler_angles(imu_data, &euler_angles);
		euler2quater(quaternion, euler_angles);
		first_time = 1;

	}
	else
	{
		estimate_euler_angles(imu_data, &euler_angles);
		euler2quater(&quater_temp, euler_angles);
		update_quater_gyro(quaternion, imu_data);
		quaternion->s = ALPHA * quaternion->s + (1 - ALPHA) * quater_temp.s;
		quaternion->x = ALPHA * quaternion->x + (1 - ALPHA) * quater_temp.x;
		quaternion->y = ALPHA * quaternion->y + (1 - ALPHA) * quater_temp.y;
		quaternion->z = ALPHA * quaternion->z + (1 - ALPHA) * quater_temp.z;

	}
	//*quaternion =  quater_result;
}


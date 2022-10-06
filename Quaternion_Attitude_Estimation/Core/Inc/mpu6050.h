#include <stdint.h>

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define DEVICE_ADDRESS 0x68

#define FS_GYRO_250    0
#define FS_GYRO_500    8
#define FS_GYRO_1000   9
#define FS_GYRO_2000   10

#define FS_ACC_2G    0
#define FS_ACC_4G    8
#define FS_ACC_8G    9
#define FS_ACC_16G   10

#define REG_CONFIG_GYRO   27
#define REG_CONFIG_ACC    28
#define REG_USR_CTRL      107
#define REG_DATA          59

typedef struct {
	int16_t x_acc;
	int16_t y_acc;
	int16_t z_acc;
	int16_t temp;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
}mpu6050_data;

void mpu6050_init();
void mpu6050_read(mpu6050_data *data);



#endif /* INC_MPU6050_H_ */

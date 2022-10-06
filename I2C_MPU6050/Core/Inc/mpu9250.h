#include <stdio.h>

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#define GYRO_CONFIG_REG       27
#define ACCEL_CONFIG_REG      28
#define PWR_MGTM1_REG         107
#define ONSET_DATA_REG        59
#define I2C_MSTR_CTRL_REG     36
#define USR_CTRL_REG          106
#define I2C_MSTR_DEL_REG      103
#define I2C_SLV4_DO_REG       52
#define CONFIG_REG            26

#define GYRO_FS_250   0
#define GYRO_FS_500   0x08
#define GYRO_FS_1000  0x10
#define GYRO_FS_2000  0x18

#define ACCEL_FS_2G   0
#define ACCEL_FS_4G   0x08
#define ACCEL_FS_8G   0x10
#define ACCEL_FS_16G  0x18

#define I2C_MSTR_CTRL_I2CCLK_400    0x0d
#define USR_CTRL_REG_I2CEN    0x20

#define PWR_MGTM1_PLL_EXSLP     0x01

//	registers related to controlling I2C MASTER
#define I2C_SLV0_ADDR_REG     37
#define I2C_SLV0_REG_REG      38
#define I2C_SLV0_CTRL_REG     39
#define I2C_SLV0_DATA_REG     99
#define EXT_SENS_DATA_00_REG  73

#define I2C_SLV0_CTRL_EN  0x80

// magnetometer registers
#define AK8963_ADDRESS        0x0C
#define AK8963_CTRL_REG       0x0a
#define AK8963_SEN_ONSET_REG  0x10
#define AK8963_DATA_ONSET_REG 0x03
#define AK8963_CTRL2_REG      0x0b

//	magnetometer operation modes
#define AK8963_PWR_DWN   0x00
#define AK8963_FUSE_ROM   0x0f
#define AK8963_CONT_MEAS2   0x16

typedef struct{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;
}mpu9250_data;


#define XMAGN_BIAS   110
#define YMAGN_BIAS   190
#define ZMAGN_BIAS   75

#define X_GYRO_BIAS 82
#define Y_GYRO_BIAS 41
#define Z_GYRO_BIAS 10

void mpu9250_init();
void mpu9250_write_reg(uint8_t reg, uint8_t data);
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
void mpu9250_read_sensor(mpu9250_data *data_imu);
#endif /* INC_MPU9250_H_ */

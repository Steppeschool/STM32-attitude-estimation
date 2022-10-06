#include <mpu9250.h>
#include <main.h>

extern SPI_HandleTypeDef hspi1;
static uint8_t mag_adjust[3];
static uint16_t filter_counter;
static mpu9250_data filter_data[FILTER_LENGTH];
static int32_t filter_xacc_sum;
static int32_t filter_yacc_sum;
static int32_t filter_zacc_sum;
static int32_t filter_xmag_sum;
static int32_t filter_ymag_sum;
static int32_t filter_zmag_sum;

static void filter_measurements(mpu9250_data *data_imu)
{
	filter_xacc_sum += data_imu->x_accel - filter_data[filter_counter].x_accel;
	filter_yacc_sum += data_imu->y_accel - filter_data[filter_counter].y_accel;
	filter_zacc_sum += data_imu->z_accel - filter_data[filter_counter].z_accel;
	filter_xmag_sum += data_imu->x_mag - filter_data[filter_counter].x_mag;
	filter_ymag_sum += data_imu->y_mag - filter_data[filter_counter].y_mag;
	filter_zmag_sum += data_imu->z_mag - filter_data[filter_counter].z_mag;

	filter_data[filter_counter] = *data_imu;
	filter_counter++;
	if	(filter_counter == FILTER_LENGTH)
	{
		filter_counter = 0;
	}
	data_imu ->x_accel = filter_xacc_sum / FILTER_LENGTH;
	data_imu ->y_accel = filter_yacc_sum / FILTER_LENGTH;
	data_imu ->z_accel = filter_zacc_sum / FILTER_LENGTH;
	data_imu ->x_mag = filter_xmag_sum / FILTER_LENGTH;
	data_imu ->y_mag = filter_ymag_sum / FILTER_LENGTH;
	data_imu ->z_mag = filter_zmag_sum / FILTER_LENGTH;

}
static void activate_spi_mpu9250()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static void deactivate_spi_mpu9250()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void ak8963_write_reg(uint8_t reg, uint8_t data)
{
	mpu9250_write_reg(I2C_SLV0_ADDR_REG, AK8963_ADDRESS);
	mpu9250_write_reg(I2C_SLV0_REG_REG, reg);
	mpu9250_write_reg(I2C_SLV0_DATA_REG, data);
	mpu9250_write_reg(I2C_SLV0_CTRL_REG, (I2C_SLV0_CTRL_EN|0x01));
}

void ak8963_read_reg(uint8_t start_reg, uint8_t *data, uint8_t len)
{
	mpu9250_write_reg(I2C_SLV0_ADDR_REG, 0x80|AK8963_ADDRESS);
	mpu9250_write_reg(I2C_SLV0_REG_REG, start_reg);
	mpu9250_write_reg(I2C_SLV0_CTRL_REG, (I2C_SLV0_CTRL_EN|len));
	HAL_Delay(10);
	mpu9250_read_reg(EXT_SENS_DATA_00_REG, data, len);

}

void mpu9250_init()
{
	uint8_t temp_data[7];

	// Resetting IMU and magnetometer
	mpu9250_write_reg(USR_CTRL_REG, 0);
	HAL_Delay(300);
	mpu9250_write_reg(0x27, 2);
	HAL_Delay(300);
	mpu9250_write_reg(USR_CTRL_REG, USR_CTRL_REG_I2CEN);
	mpu9250_write_reg(I2C_MSTR_CTRL_REG, I2C_MSTR_CTRL_I2CCLK_400);
	ak8963_write_reg(AK8963_CTRL_REG, AK8963_PWR_DWN);
	HAL_Delay(300);
	ak8963_write_reg(AK8963_CTRL2_REG, 0x01);
	HAL_Delay(300);
	mpu9250_write_reg(PWR_MGTM1_REG, 0x80);
	HAL_Delay(100);

	// gyroscope configuration
	mpu9250_write_reg(GYRO_CONFIG_REG, GYRO_FS_500);

	// accelerometer configuration
	mpu9250_write_reg(ACCEL_CONFIG_REG, ACCEL_FS_4G);

	//	Enable I2C master setting 400 kHz
	mpu9250_write_reg(USR_CTRL_REG, USR_CTRL_REG_I2CEN);
	mpu9250_write_reg(I2C_MSTR_CTRL_REG, I2C_MSTR_CTRL_I2CCLK_400);

	// Setting the sampling rate of the IMU sensor to 1 kHz
	mpu9250_write_reg(CONFIG_REG, 1);


	ak8963_write_reg(AK8963_CTRL_REG, AK8963_PWR_DWN);
	HAL_Delay(100);
	ak8963_write_reg(AK8963_CTRL_REG, AK8963_FUSE_ROM);
	HAL_Delay(100);
	ak8963_read_reg(AK8963_SEN_ONSET_REG, mag_adjust, 3);
	ak8963_write_reg(AK8963_CTRL_REG, AK8963_PWR_DWN);
	HAL_Delay(100);
	ak8963_write_reg(AK8963_CTRL_REG, AK8963_CONT_MEAS2);
	HAL_Delay(100);
	// Setting the sampling rate of the magnetometer to 50 HZ
	mpu9250_write_reg(I2C_SLV4_DO_REG, 19);
	mpu9250_write_reg(I2C_MSTR_DEL_REG, 1);
	mpu9250_write_reg(PWR_MGTM1_REG, PWR_MGTM1_PLL_EXSLP);
		HAL_Delay(100);
	ak8963_read_reg(AK8963_DATA_ONSET_REG, temp_data, 7);
	HAL_Delay(100);

}

void mpu9250_read_sensor(mpu9250_data *data_imu)
{
	static uint8_t data1[20],data2[20], buffer_counter = 0;
	uint8_t temp_data = 0x80|ONSET_DATA_REG;
	int16_t temp_mag[3];

	activate_spi_mpu9250();
	HAL_SPI_Transmit(&hspi1, &temp_data, 1, 100);
	if(buffer_counter == 0)
	{
		HAL_SPI_Receive_DMA(&hspi1, data1, 20);


		data_imu ->x_accel = ((int16_t)data2[0]<<8) + data2[1];
		data_imu ->y_accel = ((int16_t)data2[2]<<8) + data2[3];
		data_imu ->z_accel = ((int16_t)data2[4]<<8) + data2[5];

		data_imu ->x_gyro = ((int16_t)data2[8]<<8) + data2[9] -   X_GYRO_BIAS;
		data_imu ->y_gyro = ((int16_t)data2[10]<<8) + data2[11] - Y_GYRO_BIAS;
		data_imu ->z_gyro = ((int16_t)data2[12]<<8) + data2[13] - Z_GYRO_BIAS;

		temp_mag[0] = ((int16_t)data2[15]<<8) + data2[14];
		temp_mag[1] = ((int16_t)data2[17]<<8) + data2[16];
		temp_mag[2] = ((int16_t)data2[19]<<8) + data2[18];
		buffer_counter = 1;
	}
	else
	{
		HAL_SPI_Receive_DMA(&hspi1, data2, 20);

		data_imu ->x_accel = ((int16_t)data1[0]<<8) + data1[1];
		data_imu ->y_accel = ((int16_t)data1[2]<<8) + data1[3];
		data_imu ->z_accel = ((int16_t)data1[4]<<8) + data1[5];

		data_imu ->x_gyro = ((int16_t)data1[8]<<8) + data1[9] - X_GYRO_BIAS;
		data_imu ->y_gyro = ((int16_t)data1[10]<<8) + data1[11] - Y_GYRO_BIAS;
		data_imu ->z_gyro = ((int16_t)data1[12]<<8) + data1[13] - Z_GYRO_BIAS;


		temp_mag[0] = ((int16_t)data1[15]<<8) + data1[14];
		temp_mag[1] = ((int16_t)data1[17]<<8) + data1[16];
		temp_mag[2] = ((int16_t)data1[19]<<8) + data1[18];
		buffer_counter = 0;
	}

	data_imu ->x_mag = (int16_t)((float) temp_mag[1]  * (((float)mag_adjust[1] - 128) / 256.0f + 1.0f)) - XMAGN_BIAS;
	data_imu ->y_mag = (int16_t)((float) temp_mag[0]  * (((float)mag_adjust[0] - 128) / 256.0f + 1.0f)) - YMAGN_BIAS;
	data_imu ->z_mag = -(int16_t)((float)temp_mag[2] * (((float)mag_adjust[2] - 128) / 256.0f + 1.0f)) - ZMAGN_BIAS;

	//	FILTERING
	filter_measurements(data_imu);
//	printf("accelerometer : %d, %d, and %d \n", data_imu ->x_accel, data_imu ->y_accel,data_imu ->z_accel);
//	printf("gyroscope : %d, %d, and %d \n", data_imu ->x_gyro, data_imu ->y_gyro,data_imu ->z_gyro);
//	printf("magnetometer : %d, %d, and %d \n", data_imu ->x_mag, data_imu ->y_mag,data_imu ->z_mag);
}

void mpu9250_write_reg(uint8_t reg, uint8_t data)
{
	activate_spi_mpu9250();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	deactivate_spi_mpu9250();
}

void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t temp_data = 0x80|reg;
	activate_spi_mpu9250();
	HAL_SPI_Transmit(&hspi1, &temp_data , 1, 100);
	HAL_SPI_Receive(&hspi1, data, len, 100);
	deactivate_spi_mpu9250();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	deactivate_spi_mpu9250();
}

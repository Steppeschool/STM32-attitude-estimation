#include <mpu6050.h>
#include <main.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
int8_t i2c_read_flag = 0;
void mpu6050_init()
{
	 HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS <<1) + 0, 1, 100);
	  if (ret == HAL_OK)
	  {
		  printf("The device is ready \n");
	  }
	  else
	  {
		  printf("The device is not ready. CHeck cables \n ");
	  }

	  uint8_t temp_data = FS_GYRO_500;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK)
	    {
	  	  printf("Configuring gyroscope \n");
	    }
	    else
	    {
	  	  printf("Failed to configure gyroscope \n ");
	    }

	  temp_data = FS_ACC_4G;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK)
		{
		  printf("Configuring accelerometer \n");
		}
		else
		{
		  printf("Failed to configure the accelerometer  \n ");
		}

	  temp_data = 0;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0, REG_USR_CTRL, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK)
		{
		  printf("Exiting from sleep mode \n");
		}
		else
		{
		  printf("Failed to exit from sleep mode \n ");
		}
}

void mpu6050_read(mpu6050_data *data_imu)
{
	static uint8_t data1[14], data2[14], buffer_count = 0;
	if (buffer_count == 0)
	{
		HAL_I2C_Mem_Read_DMA(&hi2c1, (DEVICE_ADDRESS <<1) + 1, REG_DATA, 1, data1, 14);

		data_imu -> x_acc = ((int16_t)data2[0] << 8) + data2[1];
		data_imu -> y_acc = ((int16_t)data2[2] << 8) + data2[3];
		data_imu -> z_acc = ((int16_t)data2[4] << 8) + data2[5];

		data_imu -> temp = ((int16_t)data2[6] << 8) + data2[7];
		data_imu -> x_gyro = ((int16_t)data2[8] << 8) + data2[9];
		data_imu -> y_gyro = ((int16_t)data2[10] << 8) + data2[11];
		data_imu -> z_gyro = ((int16_t)data2[12] << 8) + data2[13];
		buffer_count = 1;
	}
	else
	{
		HAL_I2C_Mem_Read_DMA(&hi2c1, (DEVICE_ADDRESS <<1) + 1, REG_DATA, 1, data2, 14);
		data_imu -> x_acc = ((int16_t)data1[0] << 8) + data1[1];
		data_imu -> y_acc = ((int16_t)data1[2] << 8) + data1[3];
		data_imu -> z_acc = ((int16_t)data1[4] << 8) + data1[5];

		data_imu -> temp = ((int16_t)data1[6] << 8) + data1[7];
		data_imu -> x_gyro = ((int16_t)data1[8] << 8) + data1[9];
		data_imu -> y_gyro = ((int16_t)data1[10] << 8) + data1[11];
		data_imu -> z_gyro = ((int16_t)data1[12] << 8) + data1[13];
		buffer_count = 0;
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_read_flag = 1;
}

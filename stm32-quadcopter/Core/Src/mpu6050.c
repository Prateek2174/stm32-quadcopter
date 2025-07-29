/*
 * mpu6050.c
 *
 *  Created on: Jul 19, 2025
 *      Author: prate
 */

#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "mpu6050.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


#define MPU6050_ADDR 0x68

//Register 26 – Configuration CONFIG
#define EXT_SYNC_SET(mode) (mode << 3)
#define EXT_SYNC_SET_MASK  (7U << 3)
#define DLPF_CFG(mode)     (mode << 0)
#define DLPF_CFG_MASK      (0x07)

//Register 27 – Gyroscope Configuration GYRO_CONFIG
#define FS_SEL(mode) (mode << 3)
#define FS_SEL_MASK  0x18
#define XG_ST        (1U << 7)
#define YG_ST        (1U << 6)
#define ZG_ST        (1U << 5)

//Register 28 – Accelerometer Configuration ACCEL_CONFIG
#define XA_ST         (1U << 7)
#define YA_ST         (1U << 6)
#define ZA_ST         (1U << 5)
#define AFS_SEL_MASK  0x18
#define AFS_SEL(mode) (mode << 3)

//Register 35 – FIFO Enable FIFO_EN
#define TEMP_FIFO_EN  (1U << 7)
#define XG_FIFO_EN    (1U << 6)
#define YG_FIFO_EN    (1U << 5)
#define ZG_FIFO_EN    (1U << 4)
#define ACCEL_FIFO_EN (1U << 3)
#define SLV2_FIFO_EN  (1U << 2)
#define SLV1_FIFO_EN  (1U << 1)
#define SLV0_FIFO_EN  (1U << 0)

//Register 36 – I2C Master Control I2C_MST_CTRL
#define MULT_MST_EN       (1U << 7)
#define WAIT_FOR_ES       (1U << 6)
#define SLV_3_FIFO_EN     (1U << 5)
#define I2C_MST_P_NSR     (1U << 4)
#define I2C_MST_CLK(mode) (mode << 0)
#define I2C_MST_CLK_MASK  (15U << 0)

//Register 56 – Interrupt Enable INT_ENABLE
#define FIFO_OFLOW_EN  (1U << 4)
#define I2C_MST_INT_EN (1U << 3)
#define DATA_RDY_EN    (1U << 0)

//Register 58 – Interrupt Status INT_STATUS
#define FIFO_OFLOW_INT (1U << 4)
#define I2C_MST_INT    (1U << 3)
#define DATA_RDY_INT   (1U << 0)

//Register 104 – Signal Path Reset SIGNAL_PATH_RESET
#define GYRO_RESET  (1U << 2)
#define ACCEL_RESET (1U << 1)
#define TEMP_RESET  (1U << 0)

//Register 106 – User Control USER_CTRL
#define I2C_IF_DIS (1U << 4)
#define FIFO_RESET (1U << 2)
#define FIFO_EN    (1U << 6)

//Register 107 – Power Management 1 PWR_MGMT_1
#define SLEEP        (1U << 6)
#define CYCLE        (1U << 5)
#define DEVICE_RESET (1U << 7)
#define TEMP_DIS     (1U << 3)
#define CLKSEL(mode) (mode << 0)
#define CLKSEL_MASK  0x07

//Register 108 – Power Management 2 PWR_MGMT_2
#define LP_WAKE_CTRL_1_25 0x00
#define LP_WAKE_CTRL_5    0x01
#define LP_WAKE_CTRL_20   0x02
#define LP_WAKE_CTRL_40   0x03
#define STBY_ZG (1U << 0)
#define STBY_YG (1U << 1)
#define STBY_XG (1U << 2)
#define STBY_ZA (1U << 3)
#define STBY_YA (1U << 4)
#define STBY_XA (1U << 5)

//Register 117 – Who Am I WHO_AM_I
#define WHO_AM_I (0x3F << 1)

void mpu6050_init(void){

	//1.	Reset the device
	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, DEVICE_RESET);
	//2.	Wake from sleep
	i2c_clear_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, SLEEP);
	//3.	Set clock source
	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
	//4.	Enable I2C interface
	i2c_clear_bits(MPU6050_ADDR, MPU6050_USER_CTRL, I2C_IF_DIS);
	//5.	Set sample rate
	//6.	Configure DLPF
	mpu6050_set_sample_rate(3, 4); //DLPF = Mode 3, Sample Rate = 200 Hz
	//7.	Set gyro full scale range
	i2c_update_bitfield(MPU6050_ADDR, MPU6050_GYRO_CONFIG, FS_SEL_MASK, FS_SEL(3));
	//8.	Set accel full scale range
	i2c_update_bitfield(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, AFS_SEL_MASK, AFS_SEL(2));
	//9.	Disable FIFO -> Reset -> Enable, Clear FIFO_EN
	i2c_clear_bits(MPU6050_ADDR, MPU6050_USER_CTRL, FIFO_EN);
	i2c_set_bits(MPU6050_ADDR, MPU6050_USER_CTRL, FIFO_RESET);
	i2c_set_bits(MPU6050_ADDR, MPU6050_USER_CTRL, FIFO_EN);
	i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, 0xFF);
	//10.	Disable interrupts (optional)

	//11.	Verify WHO_AM_I register (optional)


}

//User Control Functions
void mpu6050_fifo_enable(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_USER_CTRL, FIFO_EN);
}

void mpu6050_fifo_reset(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_USER_CTRL, FIFO_RESET);
}

void mpu6050_set_dlpf(uint8_t dlpf_mode){

	i2c_set_bits(MPU6050_ADDR, MPU6050_CONFIG, dlpf_mode);
}

void mpu6050_set_sample_rate(uint8_t dlpf_mode, uint8_t divider){

	i2c_set_bits(MPU6050_ADDR, MPU6050_CONFIG, dlpf_mode);
	i2c_set_bits(MPU6050_ADDR, MPU6050_SMPLRT_DIV, divider);
}

//Self Test Functions
float mpu6050_get_sample_rate(void){

	float gyro_out_rate = 0;
	uint8_t data;
	i2c_read_reg(MPU6050_ADDR, MPU6050_CONFIG, &data);

	if((data & 0x07) == 0 || (data & 0x07) == 7U){
		gyro_out_rate = 8000.0f;
	}else{
		gyro_out_rate = 1000.0f;
	}

	i2c_read_reg(MPU6050_ADDR, MPU6050_SMPLRT_DIV, &data);

	float sample_rate = gyro_out_rate / (1 + data);
	return sample_rate;
}

//Gyroscope Configuration Functions


//Power Management Functions

void mpu6050_clock_source_Xgyro(void){

	i2c_update_bitfield(MPU6050_ADDR, MPU6050_PWR_MGMT_1, CLKSEL_MASK, CLKSEL(1));
}

void mpu6050_clock_source_Ygyro(void){

	i2c_update_bitfield(MPU6050_ADDR, MPU6050_PWR_MGMT_1, CLKSEL_MASK, CLKSEL(2));
}

void mpu6050_clock_source_Zgyro(void){

	i2c_update_bitfield(MPU6050_ADDR, MPU6050_PWR_MGMT_1, CLKSEL_MASK, CLKSEL(3));
}

void mpu6050_sleep(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, SLEEP);
}

void mpu6050_device_reset(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, DEVICE_RESET);
}

void mpu6050_temp_sensor_disable(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, TEMP_DIS);
}

bool mpu6050_accel_only_low_power_mode(float wakeup_freq){

	bool check = mpu6050_set_wakeup_freq(wakeup_freq);
	if(!check) return false;

	//Set CYCLE
	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, CYCLE);

	//Disable sleep
	i2c_clear_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_1, SLEEP);

	//Set TEMP_DIS bit to 1
	mpu6050_temp_sensor_disable();

	//Set STBY_XG, STBY_YG, STBY_ZG bits to 1
	mpu6050_gyro_standby();

	return true;
}

bool mpu6050_set_wakeup_freq(float wakeup_freq){

	// LP_WAKE_CTRL   Wake-up Frequency
	//      0           1.25 Hz
	//      1              5 Hz
	//      2             20 Hz
	//      3             40 Hz

	if(wakeup_freq == 1.25){

		i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, LP_WAKE_CTRL_1_25);

	}else if(wakeup_freq == 5){

		i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, LP_WAKE_CTRL_5);

	}else if(wakeup_freq == 20){

		i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, LP_WAKE_CTRL_20);

	}else if(wakeup_freq == 40){

		i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, LP_WAKE_CTRL_40);

	}else{

		return false;
		//**********Return an error statement
	}

	return true;
}

void mpu6050_gyro_standby(void){

	uint8_t enable = STBY_XG | STBY_YG | STBY_ZG;
	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, enable);
}

void mpu6050_Xgyro_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_XG);
}

void mpu6050_Ygyro_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_YG);
}

void mpu6050_Zgyro_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_ZG);
}

void mpu6050_accel_standby(void){

	mpu6050_Xaccel_standby();
	mpu6050_Yaccel_standby();
	mpu6050_Zaccel_standby();

}

void mpu6050_Xaccel_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_XA);
}

void mpu6050_Yaccel_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_YA);
}

void mpu6050_Zaccel_standby(void){

	i2c_set_bits(MPU6050_ADDR, MPU6050_PWR_MGMT_2, STBY_ZA);
}

void mpu6050_temp_read(MPU6050_data *data){

	//Enable FIFO for temp
	i2c_set_bits(MPU6050_ADDR, MPU6050_FIFO_EN, TEMP_FIFO_EN);

	//Check to make sure 2 bytes are available
	if(!(mpu6050_fifo_count_check(2))){
		i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, TEMP_FIFO_EN);
		return;
	}

	uint8_t low, high;
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);

	data->temp = (((int16_t)((high << 8) | low))/340 + 36.53);

	//Disable FIFO for temp
	i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, TEMP_FIFO_EN);
}

void mpu6050_gyro_read(MPU6050_data *data){

	i2c_set_bits(MPU6050_ADDR, MPU6050_FIFO_EN, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);

	if(!(mpu6050_fifo_count_check(6))){
		i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
		return;
	}

	uint8_t low, high;
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->gyro_x = (int16_t)((high << 8) | low);

	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->gyro_y = (int16_t)((high << 8) | low);

	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->gyro_z = (int16_t)((high << 8) | low);

	i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
}

void mpu6050_accel_read(MPU6050_data *data){

	i2c_set_bits(MPU6050_ADDR, MPU6050_FIFO_EN, ACCEL_FIFO_EN);

	if(!(mpu6050_fifo_count_check(6))){
		i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, ACCEL_FIFO_EN);
		return;
	}

	uint8_t low, high;
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->accel_x = (int16_t)((high << 8) | low);

	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->accel_y = (int16_t)((high << 8) | low);

	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &low);
	data->accel_z = (int16_t)((high << 8) | low);

	i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, ACCEL_FIFO_EN);
}

void mpu6050_read_all(MPU6050_data *data){

	uint8_t enable = ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN | TEMP_FIFO_EN;

	i2c_set_bits(MPU6050_ADDR, MPU6050_FIFO_EN, enable);

	if(!(mpu6050_fifo_count_check(14))){
		i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, enable);
		return;
	}

	uint8_t buffer[14];

	for(int i = 0; i < 14; i++){

		i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_R_W, &buffer[i]);
	}

	data -> accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
	data -> accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
	data -> accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
	data -> gyro_x = (int16_t)((buffer[6] << 8) | buffer[7]);
	data -> gyro_y = (int16_t)((buffer[8] << 8) | buffer[9]);
	data -> gyro_z = (int16_t)((buffer[10] << 8) | buffer[11]);
	data -> temp = (((int16_t)((buffer[12] << 8) | buffer[13]))/340 + 36.53);

	i2c_clear_bits(MPU6050_ADDR, MPU6050_FIFO_EN, enable);
}

bool mpu6050_fifo_count_check(uint16_t num){

	uint8_t count_low, count_high;
	uint16_t count;
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_COUNT_H, &count_high);
	i2c_read_reg(MPU6050_ADDR, MPU6050_FIFO_COUNT_L, &count_low);
	count = (uint16_t)((count_high << 8) | count_low);

	if(count < num){

		return false;
	}

	return true;
}

void mpu6050_compute_orientation(MPU6050_data *data){

	float ratio;

	//Compute Pitch
	ratio = data->accel_x / 9.81f;

	if(ratio > 1.0f){

		ratio = 1.0f;

	}else if(ratio < -1.0f){

		ratio = -1.0f;

	}

	data->pitch = asinf(ratio);

	//Compute Roll
	data->roll = atan2f(data->accel_y, data->accel_z);



}


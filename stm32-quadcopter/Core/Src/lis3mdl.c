/*
 * lis3mdl.c
 *
 *  Created on: Sep 20, 2025
 *      Author: prate
 */

#include "lis3mdl.h"
#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define LIS3MDL_ADDR 0x1E

//CTRL_REGX
#define TEMP_EN  (1U << 7)
#define OM(mode) (mode << 5)
#define OM_MASK  (3U << 5)
#define DO(mode) (mode << 2)
#define DO_MASK  (7U << 2)
#define FAST_ODR (1U << 1)
#define ST       (1U << 0)

#define FS(mode) (mode << 5)
#define FS_MASK  (3U << 5)
#define REBOOT   (1U << 3)
#define SOFT_RST (1U << 2)

#define LP       (1U << 5)
#define SIM      (1U << 2)
#define MD(mode) (mode << 0)
#define MD_MASK  (3U << 0)

#define OMZ(mode) (mode << 2)
#define OMZ_MASK  (3U << 2)
#define BLE       (1U << 1)

#define FAST_READ (1U << 7)
#define BDU       (1U << 6)

#define ZYXOR (1U << 7)
#define ZOR   (1U << 6)
#define YOR   (1U << 5)
#define XOR   (1U << 4)
#define ZYXDA (1U << 3) //IMPORTANT
#define ZDA   (1U << 2)
#define YDA   (1U << 1)
#define XDA   (1U << 0)

#define XIEN (1U << 7)
#define YIEN (1U << 6)
#define ZIEN (1U << 5)
#define IEA  (1U << 2)
#define LIR  (1U << 1)
#define IEN  (1U << 0)

#define PTH_X (1U << 7)
#define PTH_Y (1U << 6)
#define PTH_Z (1U << 5)
#define NTH_X (1U << 4)
#define NTH_Y (1U << 3)
#define NTH_Z (1U << 2)
#define MROI  (1U << 1)
#define INT   (1U << 0)

void lis3mdl_init(void){

	//Check WHO_AM_I to make sure device is working correctly
	uint8_t who_am_i;
	i2c_read_reg(LIS3MDL_ADDR, WHO_AM_I, &who_am_i);
	if(who_am_i != 0x3D){

	}

	//Configure XYZ performance mode
	lismdl_high_performance_mode();

	//Set sampling rate to 80 Hz
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG1, DO_MASK, DO(7));

	//Enable temperature sensor if necessary

	//Configure full-scale selection to +- 4 gauss
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG2, FS_MASK, FS(0));

	//Enable continuous conversion mode
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG3, MD_MASK, MD(0));

	//Set BDU to get updates in blocks

	//Configure interrupts
}

void lis3mdl_low_performance_mode(void){
	//set OM and OMZ
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG1, OM_MASK, OM(0));
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG4, OMZ_MASK, OMZ(0));

}

void lis3mdl_medium_performance_mode(void){

	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG1, OM_MASK, OM(1));
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG4, OMZ_MASK, OMZ(1));
}

void lis3mdl_high_performance_mode(void){

	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG1, OM_MASK, OM(2));
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG4, OMZ_MASK, OMZ(2));
}

void lis3mdl_ultra_high_performance_mode(void){

	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG1, OM_MASK, OM(3));
	i2c_update_bitfield(LIS3MDL_ADDR, CTRL_REG4, OMZ_MASK, OMZ(3));
}

void lis3mdl_read(LIS3MDL_data *data){

	uint8_t status;

	do{

		i2c_read(LIS3MDL_ADDR, STATUS_REG, &status, 1);

	}while((status & ZYXDA) != ZYXDA);


}


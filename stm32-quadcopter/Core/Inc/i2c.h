/*
 * i2c.h
 *
 *  Created on: Jul 20, 2025
 *      Author: prate
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f4xx.h"

//Function Prototypes
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_transmit(uint8_t address, uint8_t *data, uint16_t length);
void i2c_read(uint8_t address, uint8_t *data, uint16_t length);
void i2c_write_reg(uint8_t device_address, uint8_t register_address, uint8_t data);
void i2c_read_reg(uint8_t device_address, uint8_t register_address, uint8_t *data);
void i2c_set_bits(uint8_t device_address, uint8_t register_address, uint8_t data);
void i2c_clear_bits(uint8_t device_address, uint8_t register_address, uint8_t data);
void i2c_update_bitfield(uint8_t device_address, uint8_t register_address,uint8_t mask, uint8_t data);

void i2c_general_init(I2C_TypeDef *i2c);
void i2c_general_start(I2C_TypeDef *i2c);
void i2c_general_stop(I2C_TypeDef *i2c);
void i2c_general_transmit(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint16_t length);
void i2c_general_read(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint16_t length);
void i2c_general_read_reg(I2C_TypeDef *i2c, uint8_t device_address, uint8_t register_address, uint8_t *data);


#endif /* INC_I2C_H_ */

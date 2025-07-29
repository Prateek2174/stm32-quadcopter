/*
 * i2c.c
 *
 *  Created on: Jul 20, 2025
 *      Author: prate
 */

#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

void i2c_init(void){

	//I2C Initialization
	RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;

	I2C1 -> CR1 |= I2C_CR1_SWRST;
	I2C1 -> CR1 &= ~I2C_CR1_SWRST;
	//Peripheral clock to 16 MHz
	I2C1 -> CR2 |= (16U & 0x3F);
	//Disable PE
	I2C1 -> CR1 &= ~I2C_CR1_PE;
	//Set CCR value
	I2C1 -> CCR = 0x50;
	I2C1 -> CCR &= ~I2C_CCR_FS;
	I2C1 -> TRISE = 17U;
	//Enable PE
	I2C1 -> CR1 |= I2C_CR1_PE;
}

void i2c_start(void){

	I2C1 -> CR1 |= I2C_CR1_START;
	while(!(I2C1 -> SR1 & I2C_SR1_SB)); //Wait for SB to be set
}

void i2c_stop(void){

	I2C1 -> CR1 |= I2C_CR1_STOP;
}

void i2c_transmit(uint8_t address, uint8_t *data, uint16_t length){

	i2c_start();

	I2C1 -> DR = (address << 1) | 0;

	//Check ADDR to make sure address was sent
	while(!(I2C1 -> SR1 & I2C_SR1_ADDR));
	//Controller waits for a read of SR1 and SR2
	(void)I2C1 -> SR1;
	(void)I2C1 -> SR2;

	for(uint16_t i = 0; i < length; i++){

		while(!(I2C1 -> SR1 & I2C_SR1_TXE)); //Wait for the data register to be empty
		I2C1 -> DR = data[i];
	}

	//Wait for BTF or TC before stopping
	while(!(I2C1 -> SR1 & I2C_SR1_BTF));
	while(!(I2C1 -> SR1 & I2C_SR1_TXE));

	i2c_stop();
}

void i2c_read(uint8_t address, uint8_t *data, uint16_t length){

	i2c_start();

	//Special Case: In case a single byte has to be received, the Acknowledge disable is made
	//before the ADDR flag is cleared and the STOP condition generation is made after
	if(length == 1){

		I2C1 -> DR = (address << 1) | 1;
		I2C1 -> CR1 &= ~I2C_CR1_ACK;
		while(!(I2C1 -> SR1 & I2C_SR1_ADDR));
		(void)I2C1 -> SR1;
		(void)I2C1 -> SR2;
		i2c_stop();
		//while(!(I2C1 -> SR1 & I2C_SR1_RXNE)); <-- Not Sure
		data[0] = I2C1 -> DR;

		return;
	}

	I2C1 -> CR1 |= I2C_CR1_ACK;
	I2C1 -> DR = (address << 1) | 1;
	while(!(I2C1 -> SR1 & I2C_SR1_ADDR));
	(void)I2C1 -> SR1;
	(void)I2C1 -> SR2;

	for(uint16_t i = 0; i < length; i++){

		while(!(I2C1 -> SR1 & I2C_SR1_RXNE));
		data[i] = I2C1 -> DR;

		if((length-2) == i){
			//Sending a NACK after reading the second last data byte
			I2C1 -> CR1 &= ~I2C_CR1_ACK;
			i2c_stop();

		}
	}
}

void i2c_write_reg(uint8_t device_address, uint8_t register_address, uint8_t data){

	uint8_t reg_data[2] = {register_address, data};
	i2c_transmit(device_address, reg_data, 2);
}

void i2c_read_reg(uint8_t device_address, uint8_t register_address, uint8_t *data){

	i2c_start();

	I2C1 -> DR = (device_address << 1) | 0;
	while(!(I2C1 -> SR1 & I2C_SR1_ADDR));
	(void)I2C1 -> SR1;
	(void)I2C1 -> SR2;

	while(!(I2C1 -> SR1 & I2C_SR1_TXE));
	I2C1 -> DR = register_address;

	while(!(I2C1 -> SR1 & I2C_SR1_BTF));
	while(!(I2C1 -> SR1 & I2C_SR1_TXE));

	i2c_start();

	I2C1 -> DR = (device_address << 1) | 1;
	I2C1 -> CR1 &= ~I2C_CR1_ACK;

	while(!(I2C1 -> SR1 & I2C_SR1_ADDR));
	(void)I2C1 -> SR1;
	(void)I2C1 -> SR2;

	i2c_stop();

	while(!(I2C1 -> SR1 & I2C_SR1_RXNE));
	data[0] = I2C1 -> DR;

}

void i2c_set_bits(uint8_t device_address, uint8_t register_address, uint8_t data){

	uint8_t reg = 0;
	i2c_read_reg(device_address, register_address, &reg);
	reg |= data;
	i2c_write_reg(device_address, register_address, reg);
}

void i2c_clear_bits(uint8_t device_address, uint8_t register_address, uint8_t data){

	uint8_t reg = 0;
	i2c_read_reg(device_address, register_address, &reg);
	reg &= ~data;
	i2c_write_reg(device_address, register_address, reg);
}

void i2c_update_bitfield(uint8_t device_address, uint8_t register_address,uint8_t mask, uint8_t data){

	uint8_t reg = 0;
	i2c_read_reg(device_address, register_address, &reg);
	reg &= ~mask;
	reg |= (mask & data);
	i2c_write_reg(device_address, register_address, reg);
}

void i2c_general_init(I2C_TypeDef *i2c){

	if(i2c == I2C1){

		RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;

	}else if(i2c == I2C2){

		RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;

	}else if(i2c == I2C3){

		RCC -> APB1ENR |= RCC_APB1ENR_I2C3EN;
	}

	i2c -> CR1 |= I2C_CR1_SWRST;
	i2c -> CR1 &= ~I2C_CR1_SWRST;
	//Peripheral clock to 16 MHz
	i2c -> CR2 |= (16U & 0x3F);
	//Disable PE
	i2c -> CR1 &= ~I2C_CR1_PE;
	//Set CCR value
	i2c -> CCR = 0x50;
	i2c -> CCR &= ~I2C_CCR_FS;
	i2c -> TRISE = 17U;
	//Enable PE
	i2c -> CR1 |= I2C_CR1_PE;
}

void i2c_general_start(I2C_TypeDef *i2c){

	i2c -> CR1 |= I2C_CR1_START;
	while(!(i2c -> SR1 & I2C_SR1_SB));
}

void i2c_general_stop(I2C_TypeDef *i2c){

	i2c -> CR1 |= I2C_CR1_STOP;
}

void i2c_general_transmit(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint16_t length){

	i2c_general_start(i2c);

	i2c -> DR = (address << 1) | 0;

	//Check ADDR to make sure address was sent
	while(!(i2c -> SR1 & I2C_SR1_ADDR));
	//Controller waits for a read of SR1 and SR2
	(void)i2c -> SR1;
	(void)i2c -> SR2;

	for(uint16_t i = 0; i < length; i++){

		while(!(i2c -> SR1 & I2C_SR1_TXE)); //Wait for the data register to be empty
		i2c -> DR = data[i];
	}

	//Wait for BTF or TC before stopping
	while(!(i2c -> SR1 & I2C_SR1_BTF));
	while(!(i2c -> SR1 & I2C_SR1_TXE));

	i2c_general_stop(i2c);
}

void i2c_general_read(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint16_t length){

	i2c_general_start(i2c);

	//Special Case: In case a single byte has to be received, the Acknowledge disable is made
	//before the ADDR flag is cleared and the STOP condition generation is made after
	if(length == 1){

		i2c -> DR = (address << 1) | 1;
		i2c -> CR1 &= ~I2C_CR1_ACK;
		while(!(i2c -> SR1 & I2C_SR1_ADDR));
		(void)i2c -> SR1;
		(void)i2c -> SR2;
		i2c_general_stop(i2c);
		//while(!(I2C1 -> SR1 & I2C_SR1_RXNE)); <-- Not Sure
		data[0] = i2c -> DR;

		return;
	}

	i2c -> CR1 |= I2C_CR1_ACK;
	i2c -> DR = (address << 1) | 1;
	while(!(i2c -> SR1 & I2C_SR1_ADDR));
	(void)i2c -> SR1;
	(void)i2c -> SR2;

	for(uint16_t i = 0; i < length; i++){

		while(!(i2c -> SR1 & I2C_SR1_RXNE));
		data[i] = i2c -> DR;

		if((length-2) == i){
			//Sending a NACK after reading the second last data byte
			i2c -> CR1 &= ~I2C_CR1_ACK;
			i2c_general_stop(i2c);

			}
		}
}

void i2c_general_read_reg(I2C_TypeDef *i2c, uint8_t device_address, uint8_t register_address, uint8_t *data){

	i2c_general_start(i2c);

	i2c -> DR = (device_address << 1) | 0;
	while(!(i2c -> SR1 & I2C_SR1_ADDR));
	(void)i2c -> SR1;
	(void)i2c -> SR2;

	while(!(i2c -> SR1 & I2C_SR1_TXE));
	i2c -> DR = register_address;

	while(!(i2c -> SR1 & I2C_SR1_BTF));
	while(!(i2c -> SR1 & I2C_SR1_TXE));

	i2c_general_start(i2c);

	i2c -> DR = (device_address << 1) | 1;
	i2c -> CR1 &= ~I2C_CR1_ACK;

	while(!(i2c -> SR1 & I2C_SR1_ADDR));
	(void)i2c -> SR1;
	(void)i2c -> SR2;

	i2c_general_stop(i2c);

	while(!(i2c -> SR1 & I2C_SR1_RXNE));
	data[0] = i2c -> DR;
}

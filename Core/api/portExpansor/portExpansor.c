/*
 * portExpansor.c
 *
 *  Created on: Nov 23, 2021
 *      Author: onias
 */

#include "portExpansor.h"

/*
 * p00 - p03 : sw1 - sw4
 * p04 - p07 : bt1 - bt3
 * p10 - p17 : display 7 seg
 * */

/*
 * */
portExpansor_status_e portExpansor_ctor(portExpansor_t * const me, I2C_HandleTypeDef *hi2c, uint8_t i2cAddressMask/*,
										uint16_t irqPin_pin, GPIO_TypeDef *irqPin_port,
										uint16_t resetPin_pin, GPIO_TypeDef *resetPin_port*/)
{
	me->hi2c = hi2c;
	me->i2cAddress = (0b11101 << 3) | i2cAddressMask;
	/*me->irqPin.gpio_pin = irqPin_pin;
	me->irqPin.gpio_port = irqPin_port;
	me->resetPin.gpio_pin = resetPin_pin;
	me->resetPin.gpio_port = resetPin_port;*/
	return me->status;
}

portExpansor_status_e portExpansor_cfgPort(portExpansor_t * const me, uint8_t port, portExpansor_pinMode_e mode, uint8_t pinMask)
{
	portExpansor_status_e portExpansorStatus;
	HAL_StatusTypeDef i2c_status;
	//uint8_t MemAddress = CONFIGURATION_REG | port;
	/*TODO: pegar config atual dos pinos e mudar somente os pinos que estiverem na mascara que foi recebida na entrada*/
	pinMask = pinMask && ~mode;
	i2c_status = HAL_I2C_Mem_Write(me->hi2c, me->i2cAddress, (CONFIGURATION_REG | port), 1, &pinMask, 1, 1000);
	if(i2c_status != HAL_OK)
	{
		API_ERROR_REPORT(ERR_I2C, ERR_LVL_ERROR, "error in I2C writing");
		portExpansorStatus = PORT_EXPANSOR_ERROR;
		me->status = portExpansorStatus;
		return portExpansorStatus;
	}
	return me->status;
}

portExpansor_status_e portExpansor_writePins(portExpansor_t * const me, uint8_t port, uint8_t pinMask, uint8_t value)
{
	HAL_StatusTypeDef i2c_status;
	uint8_t currentPortValues;
	uint8_t output;
	//uint8_t MemAddress = OUTPUT_REG | port;
	/*TODO: pegar config atual dos pinos e mudar somente os pinos que estiverem na mascara que foi recebida na entrada*/

	i2c_status = HAL_I2C_Mem_Read(me->hi2c, me->i2cAddress, (OUTPUT_REG | port), 1, &currentPortValues, 1, 1000);
	if(i2c_status != HAL_OK)
	{
		API_ERROR_REPORT(ERR_I2C, ERR_LVL_ERROR, "error in I2C writing");
		me->status = PORT_EXPANSOR_ERROR;
		return me->status;
	}

	output = (~pinMask & currentPortValues);
	if(value == 1) output = output | (pinMask & 0b11111111);
	else if(value == 0) output = output & (~pinMask);
	i2c_status = HAL_I2C_Mem_Write(me->hi2c, me->i2cAddress, (OUTPUT_REG | port), 1, &output, 1, 1000);
	if(i2c_status != HAL_OK)
	{
		API_ERROR_REPORT(ERR_I2C, ERR_LVL_ERROR, "error in I2C writing");
		me->status = PORT_EXPANSOR_ERROR;
		return me->status;
	}
	return me->status;
}

portExpansor_status_e portExpansor_readPins(portExpansor_t * const me, uint8_t port, uint8_t *bitValue)
{
	HAL_StatusTypeDef i2c_status;
	//uint8_t MemAddress = INPUT_REG | port;
	/*TODO: pegar config atual dos pinos e mudar somente os pinos que estiverem na mascara que foi recebida na entrada*/
	i2c_status = HAL_I2C_Mem_Read(me->hi2c, me->i2cAddress, (INPUT_REG | port), 1, bitValue, 1, 1000);
	if(i2c_status != HAL_OK)
	{
		API_ERROR_REPORT(ERR_I2C, ERR_LVL_ERROR, "error in I2C writing");
		me->status = PORT_EXPANSOR_ERROR;
		return me->status;
	}
	return me->status;
}

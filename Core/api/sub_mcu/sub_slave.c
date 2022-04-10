/*
 * sub_slave.c
 *
 *  Created on: 3 Apr 2022
 *      Author: onias
 */

#include "sub_slave.h"
#include "sub_mcu.h"

uint8_t slave_ctor(mcuSpiSubsriber_t* subs, uint8_t id, SPI_HandleTypeDef *hspi, io_pin_t 	chipSelectPin, io_pin_t 	irqPin)
{
	subs->id = id;
	subs->hspi = hspi;
	subs->chipSelectPin = chipSelectPin;
	subs->irqPin = irqPin;

	HAL_GPIO_WritePin(subs->chipSelectPin.gpio_port, subs->chipSelectPin.gpio_pin, GPIO_PIN_SET);
	return 0;
}


uint8_t slave_set_value(mcuSpiSubsriber_t* subs, float v)
{
	comms_packet_t out_packet, in_packet;

	uint32_t start = HAL_GetTick();
	bool got_ack = false;

	uint8_t float_array[4];
	memcpy(float_array,&v,4);

	comms_build_packet(&out_packet,COMMS_TYPE_SET,float_array);

	do {
		HAL_GPIO_WritePin(subs->chipSelectPin.gpio_port, subs->chipSelectPin.gpio_pin, GPIO_PIN_RESET); // select the slave

		HAL_SPI_Transmit(subs->hspi, (uint8_t*)&out_packet, 8, 1000);
		HAL_Delay(10);
		HAL_SPI_Receive(subs->hspi, (uint8_t*)&in_packet, 8, 1000);

		HAL_GPIO_WritePin(subs->chipSelectPin.gpio_port, subs->chipSelectPin.gpio_pin, GPIO_PIN_SET); // deselect the slave
		HAL_Delay(10);

		//if(in_packet.type == COMMS_TYPE_ACK)
			got_ack = true;
	} while (
		(HAL_GetTick() < (start + SLAVE_COMMS_TIMEOUT)) ||
		(got_ack == false)
	);

	if(got_ack)
		subs->set_value = v;

	return !got_ack;
}

uint8_t slave_get_value(mcuSpiSubsriber_t* subs)
{
	float f;

	comms_packet_t out_packet, in_packet;

	uint8_t blank_data[4] = {0,0,0,0};

	uint32_t start = HAL_GetTick();
	bool got_ack = false;

	comms_build_packet(&out_packet,COMMS_TYPE_GET,blank_data);

	do
	{
		HAL_GPIO_WritePin(subs->chipSelectPin.gpio_port, subs->chipSelectPin.gpio_pin, GPIO_PIN_RESET); // select the slave

		HAL_SPI_Transmit(subs->hspi, (uint8_t*)&out_packet, 8, 1000);
		HAL_Delay(10);
		HAL_SPI_Receive(subs->hspi, (uint8_t*)&in_packet, 8, 1000);
		HAL_GPIO_WritePin(subs->chipSelectPin.gpio_port, subs->chipSelectPin.gpio_pin, GPIO_PIN_SET); // deselect the slave

		if(comms_packet_valid(&in_packet)){
			got_ack = true;
			break;
		}
		HAL_Delay(50);
	}
	while ((HAL_GetTick()-start) < SLAVE_COMMS_TIMEOUT);

	if(comms_packet_valid(&in_packet))
	{
		memcpy(&f,&in_packet.data1,4);
		subs->get_value = f;
		return !got_ack;
	}

	HAL_Delay(1000);

	return !got_ack;
}

/*
 * portExpansor.h
 *
 *  Created on: Nov 23, 2021
 *      Author: onias
 */

#ifndef API_PORTEXPANSOR_PORTEXPANSOR_H_
#define API_PORTEXPANSOR_PORTEXPANSOR_H_

#include "../api.h"
#include "../states.h"
#include "main.h"
#include "../Error_Report.h"

#define NUMBER_OF_PORTS 2U
#define INPUT_REG 0U
#define OUTPUT_REG 2U
#define POLARITY_INV_REG 4U
#define CONFIGURATION_REG 6U

typedef enum {
	pinInput = 0x0U,
	pinOutput
} portExpansor_pinMode_e;

typedef enum {
	port0 = 0x0U,
	port1
} portExpansor_pinPort_e;

typedef struct {
	uint8_t pinValue;
	uint8_t pinPolarity;
	portExpansor_pinMode_e pinMode;
} portExpansor_port_t;

typedef struct {
	struct portExpansor_vtable const *vptr; /*virtual pointer*/
	portExpansor_port_t ioPins[NUMBER_OF_PORTS];
	portExpansor_status_e status;
	io_pin_t irqPin;
	io_pin_t resetPin;
	I2C_HandleTypeDef *hi2c;
	uint8_t i2cAddress;
} portExpansor_t;

struct portExpansor_vtable {
	portExpansor_status_e (*portExpansor_WriteDigitVCall)(portExpansor_t * const me);
};

portExpansor_status_e portExpansor_ctor(portExpansor_t * const me, I2C_HandleTypeDef *hi2c, uint8_t i2cAddressMask/*,
										uint16_t irqPin_pin, GPIO_TypeDef *irqPin_port,
										uint16_t resetPin_pin, GPIO_TypeDef *resetPin_port*/);

portExpansor_status_e portExpansor_cfgPort(portExpansor_t * const me, portExpansor_pinPort_e port, portExpansor_pinMode_e mode, uint8_t pinMask);

portExpansor_status_e portExpansor_writePins(portExpansor_t * const me, portExpansor_pinPort_e port, uint8_t pinMask, uint8_t value);

portExpansor_status_e portExpansor_readPins(portExpansor_t * const me, portExpansor_pinPort_e port, uint8_t *bitValue);


static inline portExpansor_status_e portExpansor_WritePin(portExpansor_t * const me)
{
	return (*me->vptr->portExpansor_WriteDigitVCall)(me);
}

#endif /* API_PORTEXPANSOR_PORTEXPANSOR_H_ */

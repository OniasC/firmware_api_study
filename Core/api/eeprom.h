/*
 * eeprom.h
 *
 *  Created on: Jun 12, 2021
 *      Author: onias
 */

#ifndef API_EEPROM_H_
#define API_EEPROM_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"

typedef enum {
	EEPROM_AT24C02C = 0
	/* Enter later other EEPROMs we could end up using */
} eeprom_chip;

typedef enum {
	EEPROM_NO_ERROR = 0,
	EEPROM_ERROR = 1,
	EEPROM_ERROR_OVERFLW_PAGS = 2,
} eeprom_status;

typedef enum {
	NORMAL_WRITE_OP = 0U,
	FULL_ARRAY_PROTECTED = 1U
} write_protect_pin;

typedef struct{
	struct eeprom_vtable const *vptr; /*virtual pointer*/

	I2C_HandleTypeDef *hi2c;
	eeprom_chip eeprom_chip;
	uint8_t i2c_address;
	write_protect_pin write_protect;
	uint8_t number_pages;
	uint8_t page_size_bytes;
	uint16_t memAddSize;

} eeprom_t;

struct eeprom_vtable {
	eeprom_status (*eeprom_ReadVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
	eeprom_status (*eeprom_WriteVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
	eeprom_status (*eeprom_PageEraseVCall)(eeprom_t const * const me, uint16_t page);
	eeprom_status (*eeprom_WriteNumberVCall)(eeprom_t * const me, uint16_t page, uint16_t offset, float fdata);
	eeprom_status (*eeprom_ReadNumberVCall)(eeprom_t * const me, uint16_t page, uint16_t offset, float *fdata);
};

eeprom_status eeprom_ctor(eeprom_t * const me, eeprom_chip eeprom, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask);

/* virtual call (late binding) */
static inline eeprom_status eeprom_Read(eeprom_t * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	return (*me->vptr->eeprom_ReadVCall)(me, page, offset, data, size);
}

static inline eeprom_status eeprom_Write(eeprom_t * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	return (*me->vptr->eeprom_WriteVCall)(me, page, offset, data, size);
}

static inline eeprom_status eeprom_PageErase(eeprom_t * const me, uint16_t page)
{
	return (*me->vptr->eeprom_PageEraseVCall)(me, page);
}

static inline eeprom_status eeprom_WriteNumber(eeprom_t * const me, uint16_t page, uint16_t offset, float fdata)
{
	return (*me->vptr->eeprom_WriteNumberVCall)(me, page, offset, fdata);
}

static inline eeprom_status eeprom_ReadNumber(eeprom_t * const me, uint16_t page, uint16_t offset, float *fdata)
{
	return (*me->vptr->eeprom_ReadNumberVCall)(me, page, offset, fdata);
}

#endif /* API_EEPROM_H_ */

/*
 * eeprom.h
 *
 *  Created on: Jun 12, 2021
 *      Author: onias
 */

#ifndef API_EEPROM_H_
#define API_EEPROM_H_

#include "../api.h"
#include "stdint.h"
#include "../Error_Report.h"

typedef enum {
	EEPROM_AT24C02C,
	EEPROM_M24C64
	/* Enter later other EEPROMs we could end up using */
} eeprom_chip_e;



typedef enum {
	NORMAL_WRITE_OP = 0U,
	FULL_ARRAY_PROTECTED = 1U
} write_protect_e;

typedef struct {
	write_protect_e mode;
	io_pin_t io_pin;
} write_protect_pin_t;

typedef struct{
	struct eeprom_vtable const *vptr; /*virtual pointer*/

	I2C_HandleTypeDef *hi2c;
	eeprom_chip_e eeprom_chip;
	uint8_t i2c_address;
	write_protect_pin_t write_protect;
	uint16_t number_pages;
	uint8_t page_size_bytes;
	uint16_t memAddressSize;
	eeprom_status_e status;
} eeprom_t;

struct eeprom_vtable {
	eeprom_status_e (*eeprom_ReadVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
	eeprom_status_e (*eeprom_WriteVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
	eeprom_status_e (*eeprom_PageEraseVCall)(eeprom_t const * const me, uint16_t page);
	eeprom_status_e (*eeprom_WriteNumberVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, float fdata);
	eeprom_status_e (*eeprom_ReadNumberVCall)(eeprom_t const * const me, uint16_t page, uint16_t offset, float *fdata);
};

eeprom_status_e eeprom_ctor(eeprom_t * const me, eeprom_chip_e eeprom_chip, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask);

eeprom_status_e eeprom_EraseAllPages(eeprom_t * const eeprom);

uint16_t bytestowrite (const eeprom_t me, uint16_t size, uint16_t offset);

/* virtual call (late binding) */
static inline eeprom_status_e eeprom_Read(eeprom_t * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	return (*me->vptr->eeprom_ReadVCall)(me, page, offset, data, size);
}

static inline eeprom_status_e eeprom_Write(eeprom_t * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	return (*me->vptr->eeprom_WriteVCall)(me, page, offset, data, size);
}

static inline eeprom_status_e eeprom_PageErase(eeprom_t * const me, uint16_t page)
{
	return (*me->vptr->eeprom_PageEraseVCall)(me, page);
}

static inline eeprom_status_e eeprom_WriteNumber(eeprom_t * const me, uint16_t page, uint16_t offset, float fdata)
{
	return (*me->vptr->eeprom_WriteNumberVCall)(me, page, offset, fdata);
}

static inline eeprom_status_e eeprom_ReadNumber(eeprom_t * const me, uint16_t page, uint16_t offset, float *fdata)
{
	return (*me->vptr->eeprom_ReadNumberVCall)(me, page, offset, fdata);
}

#endif /* API_EEPROM_H_ */

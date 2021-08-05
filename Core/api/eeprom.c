/*
 * eeprom.c
 *
 *  Created on: Jun 12, 2021
 *      Author: onias
 */
#include "eeprom.h"

static eeprom_status eeprom_ReadVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

static eeprom_status eeprom_WriteVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

static eeprom_status eeprom_PageEraseVTable(eeprom_t const * const me, uint16_t page);

static eeprom_status eeprom_WriteNumberVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, float fdata);

static eeprom_status eeprom_ReadNumberVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, float *fdata);

eeprom_status eeprom_ctor(eeprom_t * const me, eeprom_chip eeprom, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask)
{
	static const struct eeprom_vtable vtable = {
		&eeprom_ReadVTable,
		&eeprom_WriteVTable,
		&eeprom_PageEraseVTable,
		&eeprom_WriteNumberVTable,
		&eeprom_ReadNumberVTable
	};
	me->vptr = &vtable;
	me->hi2c = hi2c;
	me->i2c_address = (0b1010 << 4) | i2c_address_mask; /*0xA0*/

	eeprom_status eeprom_init_status;

	if (me->eeprom_chip == EEPROM_AT24C02C)
	{
		me->number_pages = 32;
		me->page_size_bytes = 8;
		me->memAddSize = 1;
		eeprom_init_status = EEPROM_NO_ERROR;
	}
	else eeprom_init_status = EEPROM_NOT_INIT;

	return eeprom_init_status;
}

eeprom_status eeprom_EraseAllPages(eeprom_t * const me)
{
	for (int i=0; i<me->number_pages; i++)
	{
		eeprom_PageErase(me, i);
	}
	return EEPROM_NO_ERROR;
}

static eeprom_status eeprom_ReadVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	(void)me; /* unused parameter */
	return EEPROM_NO_ERROR;
}

static eeprom_status eeprom_WriteVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	(void)me; /* unused parameter */
	return EEPROM_NO_ERROR;
}

static eeprom_status eeprom_PageEraseVTable(eeprom_t const * const me, uint16_t page)
{
	(void)me; /* unused parameter */
	return EEPROM_NO_ERROR;
}

static eeprom_status eeprom_WriteNumberVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, float fdata)
{
	(void)me; /* unused parameter */
	return EEPROM_NO_ERROR;
}

static eeprom_status eeprom_ReadNumberVTable(eeprom_t const * const me, uint16_t page, uint16_t offset, float *fdata)
{
	(void)me; /* unused parameter */
	return EEPROM_NO_ERROR;
}


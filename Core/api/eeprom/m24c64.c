/*
 * m24c64.c
 *
 *  Created on: Nov 20, 2021
 *      Author: onias
 */

#include "math.h"
#include "string.h"

#include "m24c64.h"

eeprom_status_e eeprom_m24c64_ctor(eeprom_m24c64_t * const m24c64, eeprom_chip_e eeprom_chip,
									I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask,
									uint16_t wr_protect_Pin, GPIO_TypeDef * wr_protect_GPIO_Port)
{
	static const struct eeprom_vtable vtable = {
		(eeprom_status_e (*)(	eeprom_t const * const m24c64,
							uint16_t page,
							uint16_t offset,
							uint8_t *data,
							uint16_t size))&eeprom_m24c64_readVTable,
		(eeprom_status_e (*)(eeprom_t const * const m24c64,
							uint16_t page,
							uint16_t offset,
							uint8_t *data,
							uint16_t size))&eeprom_m24c64_writeVTable,
		(eeprom_status_e (*)(eeprom_t const * const m24c64,
							uint16_t page))&eeprom_m24c64_pageEraseVTable
	};
	eeprom_status_e eeprom_ctor_status = eeprom_ctor(&(m24c64->eeprom),
													eeprom_chip,
													hi2c,
													i2c_address_mask);
	m24c64->eeprom.vptr = &vtable;
	if (m24c64->eeprom.eeprom_chip == EEPROM_M24C64)
	{
		m24c64->eeprom.number_pages = 256;
		m24c64->eeprom.page_size_bytes = 32;
		m24c64->eeprom.memAddressSize = 2;
	}
	m24c64->eeprom.write_protect.io_pin.gpio_pin = wr_protect_Pin;
	m24c64->eeprom.write_protect.io_pin.gpio_port = wr_protect_GPIO_Port;
	return eeprom_ctor_status;
}


eeprom_status_e eeprom_m24c64_readVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef i2c_status;
	eeprom_status_e eeprom_s;

	int paddrposition = log(m24c64->eeprom.page_size_bytes)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/m24c64->eeprom.page_size_bytes);

	uint16_t numofpages = (endPage-startPage) + 1;
	if (endPage > m24c64->eeprom.number_pages-1)
	{
		eeprom_s = EEPROM_ERROR_OVERFLW_PAGS;
		API_ERROR_REPORT(ERR_EEPROM, ERR_LVL_ERROR, "page overflow");
		goto endfunction;
	}
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(m24c64->eeprom, size, offset);
		i2c_status = HAL_I2C_Mem_Read(m24c64->eeprom.hi2c, m24c64->eeprom.i2c_address, MemAddress, m24c64->eeprom.memAddressSize, &data[pos], bytesremaining, 1000);
		if(i2c_status != HAL_OK)
		{
			API_ERROR_REPORT(ERR_EEPROM, ERR_LVL_ERROR, "error in I2C writing");
			eeprom_s = EEPROM_ERROR;
			goto endfunction;
		}
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
	eeprom_s = EEPROM_NO_ERROR;

endfunction:
	return eeprom_s;
}

eeprom_status_e eeprom_m24c64_writeVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef i2c_status;
	eeprom_status_e eeprom_status;

	if (offset > m24c64->eeprom.page_size_bytes-1)
	{
		API_ERROR_REPORT(ERR_EEPROM, ERR_LVL_ERROR, "offset out of bounds");
		return EEPROM_ERROR_OVERFLW_PAGS;
	}
	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(m24c64->eeprom.page_size_bytes)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/m24c64->eeprom.page_size_bytes);
	if (endPage > m24c64->eeprom.number_pages-1)
	{
		API_ERROR_REPORT(ERR_EEPROM, ERR_LVL_ERROR, "too big of a message");
		return EEPROM_ERROR_OVERFLW_PAGS;
	}
	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	//remove write protection for writing operation
	HAL_GPIO_WritePin(m24c64->eeprom.write_protect.io_pin.gpio_port, m24c64->eeprom.write_protect.io_pin.gpio_pin, GPIO_PIN_RESET);
	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(m24c64->eeprom, size, offset);  // calculate the remaining bytes to be written

		i2c_status = HAL_I2C_Mem_Write(m24c64->eeprom.hi2c, m24c64->eeprom.i2c_address, MemAddress, m24c64->eeprom.memAddressSize, &data[pos], bytesremaining, 1000);

		if(i2c_status != HAL_OK)
		{
			API_ERROR_REPORT(ERR_EEPROM, ERR_LVL_ERROR, "error in I2C writing");
			eeprom_status = EEPROM_ERROR;
			break;
		}
		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
	//re-add write protection for writing operation
	HAL_GPIO_WritePin(m24c64->eeprom.write_protect.io_pin.gpio_port, m24c64->eeprom.write_protect.io_pin.gpio_pin, GPIO_PIN_SET);

	eeprom_status = EEPROM_NO_ERROR;
	return eeprom_status;
}

eeprom_status_e eeprom_m24c64_pageEraseVTable(eeprom_m24c64_t const * const m24c64, uint16_t page)
{
	return EEPROM_ERROR;
}

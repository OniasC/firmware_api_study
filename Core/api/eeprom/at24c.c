/*
 * at24c02c.c
 *
 *  Created on: Jun 12, 2021
 *      Author: onias
 */

#include "math.h"
#include "string.h"

#include "at24c.h"

eeprom_status eeprom_at24c_ctor(eeprom_at24c * const me, eeprom_chip eeprom, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask)
{
	static const struct eeprom_vtable vtable = {
		(eeprom_status (*)(	eeprom_t const * const me,
							uint16_t page,
							uint16_t offset,
							uint8_t *data,
							uint16_t size))&eeprom_at24c_readVTable,
		(eeprom_status (*)(eeprom_t const * const me,
							uint16_t page,
							uint16_t offset,
							uint8_t *data,
							uint16_t size))&eeprom_at24c_writeVTable,
		(eeprom_status (*)(eeprom_t const * const me,
							uint16_t page))&eeprom_at24c_pageEraseVTable,
		(eeprom_status (*)(eeprom_t const * const me,
							uint16_t page,
							uint16_t offset,
							float fdata))&eeprom_at24c_writeNumVTable,
		(eeprom_status (*)(eeprom_t const * const me,
							uint16_t page,
							uint16_t offset,
							float *fdata))&eeprom_at24c_readNumVTable
	};
	eeprom_status eeprom_ctor_status = eeprom_ctor(&(me->super),
													eeprom,
													hi2c,
													i2c_address_mask);
	me->super.vptr = &vtable;
	return eeprom_ctor_status;
}


/*****************************************************************************************************************************************/
uint8_t bytes_temp[4];

// function to determine the remaining bytes
uint16_t bytestowrite (eeprom_at24c const * const me, uint16_t size, uint16_t offset)
{
	if ((size+offset) < me->super.page_size_bytes) return size;
	else return me->super.page_size_bytes-offset;
}

void float2Bytes(uint8_t * ftoa_bytes_temp, float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

eeprom_status eeprom_at24c_readVTable (eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	eeprom_status eeprom_s;

	int paddrposition = log(me->super.page_size_bytes)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/me->super.page_size_bytes);

	uint16_t numofpages = (endPage-startPage) + 1;
	if (endPage > me->super.number_pages-1)
	{
		eeprom_s = EEPROM_ERROR_OVERFLW_PAGS;
		goto endfunction;
	}
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(me, size, offset);
		HAL_I2C_Mem_Read(me->super.hi2c, me->super.i2c_address, MemAddress, me->super.memAddSize, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
	eeprom_s = EEPROM_NO_ERROR;

endfunction:
	API_Error_Report((error_e *)ERR_EEPROM, (bsp_status *)eeprom_s);
	return eeprom_s;
}

eeprom_status eeprom_at24c_writeVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	if (offset > me->super.number_pages-1)
		return EEPROM_ERROR_OVERFLW_PAGS;
	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(me->super.page_size_bytes)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/me->super.page_size_bytes);
	if (endPage > me->super.number_pages-1)
		return EEPROM_ERROR_OVERFLW_PAGS;
	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(me, size, offset);  // calculate the remaining bytes to be written
		HAL_I2C_Mem_Write(me->super.hi2c, me->super.i2c_address, MemAddress, me->super.memAddSize, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
	return EEPROM_NO_ERROR;
}

eeprom_status eeprom_at24c_pageEraseVTable(eeprom_at24c const * const me, uint16_t page)
{
	// calculate the memory address based on the page number
	int paddrposition = log(me->super.page_size_bytes)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[me->super.page_size_bytes];
	memset(data,0xff, me->super.page_size_bytes);

	// write the data to the EEPROM
	//HAL_I2C_Mem_Write(me->super.hi2c, me->super.i2c_address, MemAddress, me->super.memAddSize, data, me->super.page_size_bytes, 1000);
	HAL_I2C_Mem_Write(me->super.hi2c, me->super.i2c_address, MemAddress, 1, data, me->super.page_size_bytes, 1000);

	HAL_Delay (5);  // write cycle delay

	return EEPROM_NO_ERROR;
}

eeprom_status eeprom_at24c_writeNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float fdata)
{
	float2Bytes(bytes_temp, fdata);

	return eeprom_at24c_writeVTable(me, page, offset, bytes_temp, 4);
}

eeprom_status eeprom_at24c_readNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float *fdata)
{
	uint8_t buffer[4];

	eeprom_status status = eeprom_at24c_readVTable(me, page, offset, buffer, 4);

	*fdata = (Bytes2float(buffer));

	return status;
}

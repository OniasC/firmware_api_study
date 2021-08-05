/*
 * at24c02c.h
 *
 *  Created on: Jun 12, 2021
 *      Author: onias
 */

#ifndef API_EEPROM_AT24C02C_H_
#define API_EEPROM_AT24C02C_H_

#include "../eeprom.h"
/*
 * The AT24C01C/AT24C02C provides 1,024/2,048 bits of Serial Electrically Erasable and Programmable
Read-Only Memory (EEPROM) organized as 128/256 words of 8 bits each.
 * */


typedef struct {
	eeprom_t super;
} eeprom_at24c;

//TODO: write doxygen style documentation

eeprom_status eeprom_at24c_ctor(eeprom_at24c * const me, eeprom_chip eeprom, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask);

/* READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
eeprom_status eeprom_at24c_readVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

/* write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
eeprom_status eeprom_at24c_writeVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

/* Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 */
eeprom_status eeprom_at24c_pageEraseVTable(eeprom_at24c const * const me, uint16_t page);

/*Write the Float/Integer values to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the float/integer value that you want to write
 */
eeprom_status eeprom_at24c_writeNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float fdata);

/* Reads the single Float/Integer values from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @returns the float/integer value
 */
eeprom_status eeprom_at24c_readNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float *fdata);


#endif /* API_EEPROM_AT24C02C_H_ */

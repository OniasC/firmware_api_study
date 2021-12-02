/*
 * m24c64.h
 *
 *  Created on: Nov 20, 2021
 *      Author: onias
 */

#ifndef API_EEPROM_M24C64_H_
#define API_EEPROM_M24C64_H_

#include "eeprom.h"

typedef struct {
	eeprom_t eeprom;
} eeprom_m24c64_t;

//TODO: UPDATE doxygen style documentation

eeprom_status_e eeprom_m24c64_ctor(eeprom_m24c64_t * const m24c64, eeprom_chip_e eeprom_chip,
									I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask,
									uint16_t wr_protect_Pin, GPIO_TypeDef * wr_protect_GPIO_Port);

/* READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
eeprom_status_e eeprom_m24c64_readVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

/* write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
eeprom_status_e eeprom_m24c64_writeVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

/* Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 */
eeprom_status_e eeprom_m24c64_pageEraseVTable(eeprom_m24c64_t const * const m24c64, uint16_t page);

/*Write the Float/Integer values to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the float/integer value that you want to write
 */
eeprom_status_e eeprom_m24c64_writeNumVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, float fdata);

/* Reads the single Float/Integer values from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @returns the float/integer value
 */
eeprom_status_e eeprom_m24c64_readNumVTable(eeprom_m24c64_t const * const m24c64, uint16_t page, uint16_t offset, float *fdata);


#endif /* API_EEPROM_M24C64_H_ */

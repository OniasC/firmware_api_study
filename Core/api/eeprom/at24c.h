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

eeprom_status eeprom_at24c_readVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

eeprom_status eeprom_at24c_writeVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

eeprom_status eeprom_at24c_pageEraseVTable(eeprom_at24c const * const me, uint16_t page);

eeprom_status eeprom_at24c_writeNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float fdata);

eeprom_status eeprom_at24c_readNumVTable(eeprom_at24c const * const me, uint16_t page, uint16_t offset, float *fdata);


#endif /* API_EEPROM_AT24C02C_H_ */

# firmware_api_study

Firmware and api study project in order to study embedded C and its features.

Code is developed with C using OOP features, such as inheritance and polymorphism. 

Hardware is described in the bsp folder

Perihperals are defined in the api folder and for each different type of sensor there are folder with implemented ICs (i.e. eeprom has declared an eeprom.c and .h files with eeprom/at24c.c and eeprom/at24c.h)

For example, eeprom is an at24c object but can be called by eeprom functions using a casting.

`eeprom_PageErase((eeprom_t *)&eeprom, 0U);`

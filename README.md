# firmware_api_study

Firmware and api study project in order to study embedded C and its features.

Code is developed with C using OOP features, such as inheritance and polymorphism. 

Hardware is described in the bsp folder

Perihperals are defined in the api folder and for each different type of sensor there are folder with implemented ICs (i.e. eeprom has declared an eeprom.c and .h files with eeprom/at24c.c and eeprom/at24c.h)

For example, eeprom is an at24c object but can be called by eeprom functions using a casting.

`eeprom_PageErase((eeprom_t *)&eeprom, 0U);`

api folder contains the drivers and libraries to communicate with the different peripherals, while bsp (board support package) initliazes the objects used in the board/project.

app folder would the application code, which would use the code exposed by the api. 

better ways to organize the code exist, this repo has a ongoing goal to serve me as a basis to study and test different (hopefully better ways) to organize my code. 

Things still not working:

- NRF24Network doesnt work in C'. There is an issue that a hardfault is triggered at some point which I didnt have time to investigate.
- tinyNetwork is supposed to be a lightweight version of that, built from ground up instead of based on the ready nrf24network arduino library. 
- the port expansor chosen for some reason doesn't work on pins p00 and p10. Still need to investigate that

Further TODOS:

- shrink code space - code is to big for microcontrollers like stm32f103 with 64kb of flash. Some optimizations are necessary.
- add more imu's (I have the mpu9250 and icm to implement and test)
- maybe create another HAL_API layer to make the api independent of the stm32_hal code?


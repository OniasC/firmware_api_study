/*
 * sub_mcu.h
 *
 *  Created on: 3 Apr 2022
 *      Author: onias
 */

#ifndef API_SUB_MCU_SUB_MCU_H_
#define API_SUB_MCU_SUB_MCU_H_


#include <stdint.h>
#include <stdbool.h>
#include "string.h"

#define SYNC_1 0xF
#define SYNC_2 0xE

typedef struct _comms_packet_t {
	uint8_t sync1;
	uint8_t sync2;
	uint8_t type;
	uint8_t checksum;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t data4;
} comms_packet_t;

typedef enum _comms_type_e {
	COMMS_TYPE_SET = 0,
	COMMS_TYPE_GET,
	COMMS_TYPE_ACK,
	COMMS_TYPE_COUNT
} comms_type_e;

bool comms_packet_valid(comms_packet_t* packet);
void comms_input_packet(uint8_t* packet);
bool comms_get_packet(comms_packet_t* packet);
void comms_build_packet(comms_packet_t* packet, comms_type_e type, uint8_t* data);
bool comms_packet_ready();



#endif /* API_SUB_MCU_SUB_MCU_H_ */

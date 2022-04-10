/*
 * sub_mcu.c
 *
 *  Created on: 3 Apr 2022
 *      Author: onias
 */

#include "sub_mcu.h"

typedef union _packet_builder_t {
	uint8_t buff[8];
	comms_packet_t packet;
} packet_builder_t;

static packet_builder_t _packet_builder;
static uint8_t _pos;
static volatile comms_packet_t _packet;
static volatile bool _packet_ready;

static uint8_t checksum(uint8_t* data)
{
	uint8_t sum = 0;

	sum += data[0];
	sum += data[1];
	sum += data[2];
	sum += data[3];

	return sum;
}

static bool comms_input_byte(uint8_t byte)
{
	switch(_pos){
		case 0:
			if (byte == SYNC_1)
				goto valid;
			break;

		case 1:
			if (byte == SYNC_2)
				goto valid;
			break;

		case 2:
			if(byte < COMMS_TYPE_COUNT)
				goto valid;
			break;

		case 3:
			goto valid;
			break;

		case 4:
		case 5:
		case 6:
			goto valid;
			break;

		case 7:
			_packet_builder.buff[_pos] = byte;
			_pos = 0;
			if(!comms_packet_valid(&_packet_builder.packet))
				goto invalid;
			else
			{
				if(_packet_ready) // we already have a packet
					goto invalid;
				else
				{
					memcpy(&_packet, &_packet_builder.packet, 8);
					_packet_ready = true;
					goto invalid;
				}
			}
			break;
	}

	invalid:
		_pos = 0;
		return true;

	valid:
		_packet_builder.buff[_pos] = byte;
		_pos++;
		return true;
}

bool comms_packet_valid(comms_packet_t* packet)
{
	uint8_t sum = checksum(&packet->data1);

	if(sum == packet->checksum)
		return true;
	else
		return false;
}

/* Data can be 4 bytes maximum */
void comms_build_packet(comms_packet_t* packet, comms_type_e type, uint8_t* data)
{
	packet->sync1 = SYNC_1;
	packet->sync2 = SYNC_2;

	packet->type = type;

	packet->data1 = data[0];
	packet->data2 = data[1];
	packet->data3 = data[2];
	packet->data4 = data[3];

	packet->checksum = checksum(data);
}

void comms_input_packet(uint8_t* packet)
{
	for (uint32_t i=0; i<8; i++)
	{
		comms_input_byte(*packet);
		packet++;
	}
}

bool comms_get_packet(comms_packet_t* packet)
{
	if(_packet_ready)
	{
		memcpy(packet, &_packet, 8);
		_packet_ready = false;
		return true;
	}
	return false;
}

bool comms_packet_ready()
{
	return _packet_ready;
}

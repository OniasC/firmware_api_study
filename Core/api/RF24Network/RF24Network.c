/*
 * network.c
 *
 *  Created on: Oct 9, 2021
 *      Author: onias
 */


#include "../radio/MY_NRF24.h"
#include "RF24Network.h"

void RF24Network_ctor(RF24Network_t * const rf24network, nrf24_t * const radio)
{
	rf24network->radio = radio;
	/*configuring radio*/
	rf24network->radio->payload_size = 32;
#if defined(DISABLE_FRAGMENTATION)
	rf24network->radio->dynamic_payloads_enabled = true;
#else
	rf24network->radio->dynamic_payloads_enabled = false;
#endif
	rf24network->radio->addr_width = 5;

	rf24network->radio->pipe0_reading_address[0] = 0;
	/*end configuring radio*/
	rf24network->max_frame_payload_size = MAX_FRAME_SIZE - sizeof(RF24NetworkHeader);
	rf24network->next_frame = rf24network->frame_queue;
	#if !defined(DISABLE_FRAGMENTATION)
	rf24network->frag_queue.message_buffer = &(rf24network->frag_queue_message_buffer[0]);
	rf24network->frag_ptr = &(rf24network->frag_queue);
	#endif
	rf24network->networkFlags = 0;
	rf24network->returnSysMsgs = 0;
	rf24network->multicastRelay = 0;
}

void RF24Network_begin(RF24Network_t * const rf24network, uint8_t _channel, uint16_t _node_address)
{
    if (!RF24Network_is_valid_address(rf24network, _node_address))
        return;

    rf24network->node_address = _node_address;

    //if (!radio.isValid())
    //    return;

    // Set up the radio the way we want it to look
    if (_channel != USE_CURRENT_CHANNEL)
        NRF24_setChannel(rf24network->radio, _channel);

    //radio.enableDynamicAck();
    NRF24_setAutoAck(rf24network->radio, 1);
    NRF24_setAutoAckPipe(rf24network->radio, 0, 0);

    #if defined(ENABLE_DYNAMIC_PAYLOADS)
    NRF24_enableDynamicPayloads(rf24network->radio);
    #endif

    // Use different retry periods to reduce data collisions
    uint8_t retryVar = (((rf24network->node_address % 6) + 1) * 2) + 3;
    NRF24_setRetries(rf24network->radio, retryVar, 5);// max about 85ms per attempt
    rf24network->txTimeout = 25;

    rf24network->routeTimeout = rf24network->txTimeout * 3;  // Adjust for max delay per node within a single chain

    // Setup our address helper cache
    RF24Network_setup_address(rf24network);

    // Open up all listening pipes
    uint8_t i = 6;
    while (i--)
    	NRF24_openReadingPipe(rf24network->radio, i, RF24Network_pipe_address(rf24network, _node_address, i));

    NRF24_startListening(rf24network->radio);
}

uint8_t RF24Network_update(RF24Network_t * const rf24network)
{

    uint8_t returnVal = 0;
    uint32_t timeout = HAL_GetTick();

    while (NRF24_available(rf24network->radio)) {
        if (HAL_GetTick() - timeout > 1000) {
            #if defined FAILURE_HANDLING
        	rf24network->radio.failureDetected = 1;
            #endif
            break;
        }
        #if defined(ENABLE_DYNAMIC_PAYLOADS) && !defined(XMEGA_D3)
        rf24network->frame_size = NRF24_getDynamicPayloadSize(rf24network->radio);
        #else
        rf24network->frame_size = MAX_FRAME_SIZE;
        #endif

        // Fetch the payload, and see if this was the last one.
        NRF24_read(rf24network->radio, rf24network->frame_buffer, rf24network->frame_size);

        // Read the beginning of the frame as the header
        RF24NetworkHeader *header = (RF24NetworkHeader *)(&(rf24network->frame_buffer));

        // Throw it away if it's not a valid address or too small
        if (rf24network->frame_size < sizeof(RF24NetworkHeader) ||
        		!RF24Network_is_valid_address(rf24network, header->to_node) ||
				!RF24Network_is_valid_address(rf24network, header->from_node)) {
        	API_DEBUG_MESSAGE("threw away message -> header wrong\n\r");
            continue;
        }
        #if defined(RF24_LINUX)
        IF_SERIAL_DEBUG(printf_P("%u: MAC Received %s\n\r", millis(), header->toString()));
        if (frame_size) {
            IF_SERIAL_DEBUG_FRAGMENTATION_L2(printf("%u: FRG Rcv frame size %i\n", millis(), frame_size););
            IF_SERIAL_DEBUG_FRAGMENTATION_L2(printf("%u: FRG Rcv frame ", millis()); const char *charPtr = reinterpret_cast<const char *>(frame_buffer); for (uint16_t i = 0; i < frame_size; i++) { printf("%02X ", charPtr[i]); }; printf("\n\r"));
        }
        #else
	    sprintf(rf24network->debugString, "received header: from: %d; to: %d; type: %d; id: %d; reserved: %d\r\n\n", header->from_node, header->to_node, header->type, header->id, header->reserved);
	    API_DEBUG_MESSAGE(rf24network->debugString);
//        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received %s\n\r"), millis(), header->toString()));
//        IF_SERIAL_DEBUG(const uint16_t *i = reinterpret_cast<const uint16_t *>(frame_buffer + sizeof(RF24NetworkHeader)); printf_P(PSTR("%lu: NET message %04x\n\r"), millis(), *i));
        #endif

        returnVal = header->type;
        // Is this for us?
        if (header->to_node == rf24network->node_address) {
            if (header->type == NETWORK_PING) {
                continue;
            }
            if (header->type == NETWORK_ADDR_RESPONSE) {
                uint16_t requester = NETWORK_DEFAULT_ADDRESS;
                if (requester != rf24network->node_address) {
                    header->to_node = requester;
                    RF24Network_write_2_params(rf24network, header->to_node, USER_TX_TO_PHYSICAL_ADDRESS);
                    continue;
                }
            }
            if (header->type == NETWORK_REQ_ADDRESS && rf24network->node_address) {
                header->from_node = rf24network->node_address;
                header->to_node = 0;
                RF24Network_write_2_params(rf24network, header->to_node, TX_NORMAL);
                continue;
            }
            if ((rf24network->returnSysMsgs && header->type > 127) || header->type == NETWORK_ACK) {
//                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu MAC: System payload rcvd %d\n"), millis(), returnVal););
                if (header->type != NETWORK_FIRST_FRAGMENT && header->type != NETWORK_MORE_FRAGMENTS && header->type != EXTERNAL_DATA_TYPE && header->type != NETWORK_LAST_FRAGMENT) {
                    return returnVal;
                }
            }

            if (RF24Network_enqueue(rf24network, header) == 2) { //External data received
                #if defined(SERIAL_DEBUG_MINIMAL)
            	API_DEBUG_MESSAGE("ret ext\n\r");
                #endif
                return EXTERNAL_DATA_TYPE;
            }
        } else {
            #if defined(RF24NetworkMulticast)

            if (header->to_node == 0100) {
                if (header->type == NETWORK_POLL) {
                    returnVal = 0;
                    if (!(networkFlags & FLAG_NO_POLL) && node_address != NETWORK_DEFAULT_ADDRESS) {
                        header->to_node = header->from_node;
                        header->from_node = node_address;
                        delay(parent_pipe);
                        RF24Network_write_2_params(rf24network, header->to_node, USER_TX_TO_PHYSICAL_ADDRESS);
                    }
                    continue;
                }

                uint8_t val = enqueue(header);

                if (multicastRelay) {
                    IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%u MAC: FWD multicast frame from 0%o to level %u\n"), millis(), header->from_node, multicast_level + 1););
                    if ((node_address >> 3) != 0) {
                        // for all but the first level of nodes, those not directly connected to the master, we add the total delay per level
                        delayMicroseconds(600 * 4);
                    }
                    delayMicroseconds((node_address % 4) * 600);
                    RF24Network_write_2_params(levelToAddress(multicast_level) << 3, 4);
                }
                if (val == 2) { //External data received
                    return EXTERNAL_DATA_TYPE;
                }
            } else {
                if (node_address != NETWORK_DEFAULT_ADDRESS) {
                	RF24Network_write_2_params(rf24network, header->to_node, 1); //Send it on, indicate it is a routed payload
                    returnVal = 0;
                }
            }
            #else // not defined(RF24NetworkMulticast)
            if (rf24network->node_address != NETWORK_DEFAULT_ADDRESS) {
            	RF24Network_write_2_params(rf24network, header->to_node, 1); //Send it on, indicate it is a routed payload
                returnVal = 0;
            }
            #endif // defined(RF24NetworkMulticast)
        }

    } // radio.available()
    return returnVal;
}

uint8_t RF24Network_enqueue(RF24Network_t * const rf24network, RF24NetworkHeader *header)
{
    bool result = false;
    uint16_t message_size = rf24network->frame_size - sizeof(RF24NetworkHeader);

//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue @%x "), millis(), next_frame - frame_queue));

    #if !defined(DISABLE_FRAGMENTATION)

    bool isFragment = header->type == NETWORK_FIRST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS || header->type == NETWORK_LAST_FRAGMENT;

    if (isFragment) {

        if (header->type == NETWORK_FIRST_FRAGMENT) {

            memcpy(&(rf24network->frag_queue), &(rf24network->frame_buffer), sizeof(RF24NetworkHeader));
            memcpy(rf24network->frag_queue.message_buffer, rf24network->frame_buffer + sizeof(RF24NetworkHeader), message_size);

//            IF_SERIAL_DEBUG_FRAGMENTATION(Serial.print(F("queue first, total frags ")); Serial.println(header->reserved););
            //Store the total size of the stored frame in message_size
            rf24network->frag_queue.message_size = message_size;
            --(rf24network->frag_queue.header.reserved);
//            IF_SERIAL_DEBUG_FRAGMENTATION_L2(for (int i = 0; i < frag_queue.message_size; i++) { Serial.println(frag_queue.message_buffer[i], HEX); });
            return true;
        } else // NETWORK_MORE_FRAGMENTS
        if (header->type == NETWORK_LAST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS) {

            if (rf24network->frag_queue.message_size + message_size > MAX_PAYLOAD_SIZE) {
//                #if defined(SERIAL_DEBUG_FRAGMENTATION) || defined(SERIAL_DEBUG_MINIMAL)
//                Serial.print(F("Drop frag "));
//                Serial.print(header->reserved);
//                Serial.println(F(" Size exceeds max"));
//                #endif
            	rf24network->frag_queue.header.reserved = 0;
                return false;
            }
            if (rf24network->frag_queue.header.reserved == 0 ||
            		(header->type != NETWORK_LAST_FRAGMENT && header->reserved != rf24network->frag_queue.header.reserved) ||
					rf24network->frag_queue.header.id != header->id) {
//                #if defined(SERIAL_DEBUG_FRAGMENTATION) || defined(SERIAL_DEBUG_MINIMAL)
//                Serial.print(F("Drop frag "));
//                Serial.print(header->reserved);
//                Serial.println(F(" Out of order "));
//                #endif
                return false;
            }

            memcpy(rf24network->frag_queue.message_buffer + rf24network->frag_queue.message_size, rf24network->frame_buffer + sizeof(RF24NetworkHeader), message_size);
            rf24network->frag_queue.message_size += message_size;

            if (header->type != NETWORK_LAST_FRAGMENT) {
                --(rf24network->frag_queue.header.reserved);
                return true;
            }
            rf24network->frag_queue.header.reserved = 0;
            rf24network->frag_queue.header.type = header->reserved;

//            IF_SERIAL_DEBUG_FRAGMENTATION(printf_P(PSTR("fq 3: %d\n"), frag_queue.message_size););
//            IF_SERIAL_DEBUG_FRAGMENTATION_L2(for (int i = 0; i < frag_queue.message_size; i++) { Serial.println(frag_queue.message_buffer[i], HEX); });

            //Frame assembly complete, copy to main buffer if OK
            if (rf24network->frag_queue.header.type == EXTERNAL_DATA_TYPE) {
                return 2;
            }
//            #if defined(DISABLE_USER_PAYLOADS)
//           return 0;
//            #endif

            if ((uint16_t)(MAX_PAYLOAD_SIZE) - (rf24network->next_frame - rf24network->frame_queue) >= rf24network->frag_queue.message_size) {
                memcpy(rf24network->next_frame, &(rf24network->frag_queue), 10);
                memcpy(rf24network->next_frame + 10, rf24network->frag_queue.message_buffer, rf24network->frag_queue.message_size);
                rf24network->next_frame += (10 + rf24network->frag_queue.message_size);
//                #if !defined(ARDUINO_ARCH_AVR)
//COMMENT IF BELLOW???
                uint8_t padding;
                padding = (rf24network->frag_queue.message_size + 10) % 4;
                if (padding) {
                	rf24network->next_frame += 4 - padding;
                }
//                #endif
//                IF_SERIAL_DEBUG_FRAGMENTATION(printf_P(PSTR("enq size %d\n"), frag_queue.message_size););
                return true;
            }
//            IF_SERIAL_DEBUG_FRAGMENTATION(printf_P(PSTR("Drop frag payload, queue full\n")););
            return false;
        } //If more or last fragments
    } else //else is not a fragment
    #endif // End fragmentation enabled

    // Copy the current frame into the frame queue
    #if !defined(DISABLE_FRAGMENTATION)
    if (header->type == EXTERNAL_DATA_TYPE) {
        memcpy(&(rf24network->frag_queue), &(rf24network->frame_buffer), 8);
        rf24network->frag_queue.message_buffer = rf24network->frame_buffer + sizeof(RF24NetworkHeader);
        rf24network->frag_queue.message_size = message_size;
        return 2;
    }
    #endif
//    #if defined(DISABLE_USER_PAYLOADS)
//    return 0;
//	}
//    #else // !defined(DISABLE_USER_PAYLOADS)
    if (message_size + (rf24network->next_frame - rf24network->frame_queue) <= MAIN_BUFFER_SIZE) {
        memcpy(rf24network->next_frame, &(rf24network->frame_buffer), 8);
        memcpy(rf24network->next_frame + 8, &message_size, 2);
        memcpy(rf24network->next_frame + 10, rf24network->frame_buffer + 8, message_size);

        //IF_SERIAL_DEBUG_FRAGMENTATION( for(int i=0; i<message_size;i++){ Serial.print(next_frame[i],HEX); Serial.print(" : "); } Serial.println(""); );

        rf24network->next_frame += (message_size + 10);
        #if !defined(ARDUINO_ARCH_AVR)
        uint8_t padding;
		padding = (rf24network->frag_queue.message_size + 10) % 4;
		if (padding) {
        	rf24network->next_frame += 4 - padding;
        }
        #endif
//        IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print("Enq "); Serial.println(next_frame-frame_queue); );//printf_P(PSTR("enq %d\n"),next_frame-frame_queue); );

        result = true;
    } else {
        result = false;
//        IF_SERIAL_DEBUG(printf_P(PSTR("NET **Drop Payload** Buffer Full")));
    }
    return result;
}

bool RF24Network_available(RF24Network_t * const rf24network)
{
	return (rf24network->next_frame > rf24network->frame_queue);
}

uint16_t RF24Network_read(RF24Network_t * const rf24network, RF24NetworkHeader *header, void *message, uint16_t maxlen)
{
    uint16_t bufsize = 0;

    ///memcpy(&header, rf24network->frame_queue, 8);
    memcpy(header, rf24network->frame_queue, 8);
    memcpy(&bufsize, rf24network->frame_queue + 8, 2);

    if (maxlen > 0) {
        maxlen = rf24_min(maxlen, bufsize);
        memcpy(message, rf24network->frame_queue + 10, maxlen);
//        IF_SERIAL_DEBUG(printf("%lu: NET message size %d\n", millis(), bufsize););

//        IF_SERIAL_DEBUG(uint16_t len = maxlen; printf_P(PSTR("%lu: NET r message "), millis()); const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message); while (len--) { printf("%02x ", charPtr[len]); } printf_P(PSTR("\n\r")));
    }
    rf24network->next_frame -= bufsize + 10;
    uint8_t padding = 0;
    #if !defined(ARDUINO_ARCH_AVR)
    if ((padding = (bufsize + 10) % 4)) {
        padding = 4 - padding;
        rf24network->next_frame -= padding;
    }
    #endif // !defined(ARDUINO_ARCH_AVR)
    memmove(rf24network->frame_queue, rf24network->frame_queue + bufsize + 10 + padding, sizeof(rf24network->frame_queue) - bufsize);
    //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Received %s\n\r"),millis(),header.toString()));

    return bufsize;
}

bool RF24Network_write_message(RF24Network_t * const rf24network, RF24NetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
	#if defined(DISABLE_FRAGMENTATION)
	 frame_size = rf24_min(len + sizeof(RF24NetworkHeader), MAX_FRAME_SIZE);
	 return _write(header, message, rf24_min(len, max_frame_payload_size), writeDirect);
	 #else // !defined(DISABLE_FRAGMENTATION)
	 if (len <= rf24network->max_frame_payload_size) {
		 //Normal Write (Un-Fragmented)
		 rf24network->frame_size = len + sizeof(RF24NetworkHeader);
		 if (RF24Network_write(rf24network, header, message, len, writeDirect)) {
			 return 1;
		 }
		 return 0;
	 }
	 //Check payload size
	 if (len > MAX_PAYLOAD_SIZE) {
//		 IF_SERIAL_DEBUG(printf("%u: NET write message failed. Given 'len' %d is bigger than the MAX Payload size %i\n\r", millis(), len, MAX_PAYLOAD_SIZE););
		 return false;
	 }

	 //Divide the message payload into chunks of max_frame_payload_size
	 uint8_t fragment_id = (len % rf24network->max_frame_payload_size != 0) + ((len) / rf24network->max_frame_payload_size); //the number of fragments to send = ceil(len/max_frame_payload_size)

	 uint8_t msgCount = 0;

//	 IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG Total message fragments %d\n\r", millis(), fragment_id););

	 if (header->to_node != 0100) {
		 rf24network->networkFlags |= FLAG_FAST_FRAG;
		 NRF24_stopListening(rf24network->radio);
	 }

	 uint8_t retriesPerFrag = 0;
	 uint8_t type = header->type;
	 bool ok = 0;

	 while (fragment_id > 0) {

		 //Copy and fill out the header
		 //RF24NetworkHeader fragmentHeader = header;
		 header->reserved = fragment_id;

		 if (fragment_id == 1) {
			 header->type = NETWORK_LAST_FRAGMENT; //Set the last fragment flag to indicate the last fragment
			 header->reserved = type;              //The reserved field is used to transmit the header type
		 } else {
			 if (msgCount == 0) {
				 header->type = NETWORK_FIRST_FRAGMENT;
			 } else {
				 header->type = NETWORK_MORE_FRAGMENTS; //Set the more fragments flag to indicate a fragmented frame
			 }
		 }

		 uint16_t offset = msgCount * rf24network->max_frame_payload_size;
		 uint16_t fragmentLen = rf24_min((uint16_t)(len - offset), rf24network->max_frame_payload_size);

		 //Try to send the payload chunk with the copied header
		 rf24network->frame_size = sizeof(RF24NetworkHeader) + fragmentLen;
		 ok = RF24Network_write(rf24network, header, ((char *)message) + offset, fragmentLen, writeDirect);

		 if (!ok) {
			 HAL_Delay(2);
			 ++retriesPerFrag;
		 } else {
			 retriesPerFrag = 0;
			 fragment_id--;
			 msgCount++;
		 }

		 //if(writeDirect != 070){ delay(2); } //Delay 2ms between sending multicast payloads

		 if (!ok && retriesPerFrag >= 3) {
//			 IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG TX with fragmentID '%d' failed after %d fragments. Abort.\n\r", millis(), fragment_id, msgCount););
			 break;
		 }

		 //Message was successful sent
		 #if defined SERIAL_DEBUG_FRAGMENTATION_L2
		 printf("%lu: FRG message transmission with fragmentID '%d' sucessfull.\n\r", millis(), fragment_id);
		 #endif
	 }
	 header->type = type;
	 if (rf24network->networkFlags & FLAG_FAST_FRAG) {
		 ok = NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
		 NRF24_startListening(rf24network->radio);
		 NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
	 }
	 rf24network->networkFlags &= ~FLAG_FAST_FRAG;

	 //Return true if all the chunks where sent successfully

//	 IF_SERIAL_DEBUG_FRAGMENTATION(printf("%u: FRG total message fragments sent %i. \n", millis(), msgCount););
	 if (!ok || fragment_id > 0) {
		 return false;
	 }
	 return true;

	#endif //Fragmentation enabled
}


bool RF24Network_write(RF24Network_t * const rf24network, RF24NetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
    // Fill out the header
    header->from_node = rf24network->node_address;

    // Build the full frame to send
    //memcpy(rf24network->frame_buffer, &header, sizeof(RF24NetworkHeader));
    memcpy(rf24network->frame_buffer, header, sizeof(RF24NetworkHeader));

    #if defined(RF24_LINUX)
//    IF_SERIAL_DEBUG(printf_P(PSTR("%u: NET Sending %s\n\r"), millis(), header.toString()));
    #else
//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"), millis(), header.toString()));
    #endif
    if (len) {
        #if defined(RF24_LINUX)
        memcpy(frame_buffer + sizeof(RF24NetworkHeader), message, rf24_min(frame_size - sizeof(RF24NetworkHeader), len));
//        IF_SERIAL_DEBUG(printf("%u: FRG frame size %i\n", millis(), frame_size););
//        IF_SERIAL_DEBUG(printf("%u: FRG frame ", millis()); const char *charPtr = reinterpret_cast<const char *>(frame_buffer); for (uint16_t i = 0; i < frame_size; i++) { printf("%02X ", charPtr[i]); }; printf("\n\r"));
        #else

        memcpy(rf24network->frame_buffer + sizeof(RF24NetworkHeader), message, len);

//        IF_SERIAL_DEBUG(uint16_t tmpLen = len; printf_P(PSTR("%lu: NET message "), millis()); const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message); while (tmpLen--) { printf("%02x ", charPtr[tmpLen]); } printf_P(PSTR("\n\r")));
        #endif
    }

    // If the user is trying to send it to himself
    /*if ( header.to_node == node_address ){
	#if defined (RF24_LINUX)
	  RF24NetworkFrame frame = RF24NetworkFrame(header,message,rf24_min(MAX_FRAME_SIZE-sizeof(RF24NetworkHeader),len));
	#else
      RF24NetworkFrame frame(header,len);
    #endif
	// Just queue it in the received queue
    return enqueue(frame);
  }*/
    // Otherwise send it out over the air

    if (writeDirect != 070) {
        uint8_t sendType = USER_TX_TO_LOGICAL_ADDRESS; // Payload is multicast to the first node, and routed normally to the next

        if (header->to_node == 0100)
            sendType = USER_TX_MULTICAST;
        if (header->to_node == writeDirect)
            sendType = USER_TX_TO_PHYSICAL_ADDRESS; // Payload is multicast to the first node, which is the recipient

        return RF24Network_write_2_params(rf24network, writeDirect, sendType);
    }
    return RF24Network_write_2_params(rf24network, header->to_node, TX_NORMAL);
}


bool RF24Network_write_2_params(RF24Network_t * const rf24network, uint16_t to_node, uint8_t directTo) // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
{
    bool ok = false;
    bool isAckType = false;
    if (rf24network->frame_buffer[6] > 64 && rf24network->frame_buffer[6] < 192)
        isAckType = true;

    /*if( ( (frame_buffer[7] % 2) && frame_buffer[6] == NETWORK_MORE_FRAGMENTS) ){
	isAckType = 0;
    }*/

    // Throw it away if it's not a valid address
    if (!RF24Network_is_valid_address(rf24network, to_node))
        return false;

    //Load info into our conversion structure, and get the converted address info
    logicalToPhysicalStruct conversion = {to_node, directTo, 0};
    RF24Network_logicalToPhysicalAddress(rf24network, &conversion);

    #if defined(RF24_LINUX)
    IF_SERIAL_DEBUG(printf_P(PSTR("%u: MAC Sending to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe));
    #else
//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe));
    #endif
    /**Write it*/
    if (directTo == TX_ROUTED && conversion.send_node == to_node && isAckType) {
        HAL_Delay(2);
    }
    ok = RF24Network_write_to_pipe(rf24network, conversion.send_node, conversion.send_pipe, conversion.multicast);

    if (!ok) {
        #if defined(RF24_LINUX)
        IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%u: MAC Send fail to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
    }
        #else
//        IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
    }
        #endif // !defined(RF24_LINUX)

    if (directTo == TX_ROUTED && ok && conversion.send_node == to_node && isAckType) {

        RF24NetworkHeader *header = (RF24NetworkHeader *)&(rf24network->frame_buffer);
        header->type = NETWORK_ACK;          // Set the payload type to NETWORK_ACK
        header->to_node = header->from_node; // Change the 'to' address to the 'from' address

        conversion.send_node = header->from_node;
        conversion.send_pipe = TX_ROUTED;
        conversion.multicast = 0;
        RF24Network_logicalToPhysicalAddress(rf24network, &conversion);

        //Write the data using the resulting physical address
        rf24network->frame_size = sizeof(RF24NetworkHeader);
        RF24Network_write_to_pipe(rf24network, conversion.send_node, conversion.send_pipe, conversion.multicast);

        //dynLen=0;
        #if defined(RF24_LINUX)
        IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%u MAC: Route OK to 0%o ACK sent to 0%o\n"), millis(), to_node, header->from_node););
        #else
//        IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu MAC: Route OK to 0%o ACK sent to 0%o\n"), millis(), to_node, header->from_node););
        #endif
    }

    if (ok && conversion.send_node != to_node && (directTo == 0 || directTo == 3) && isAckType) {
        // Now, continue listening
        if (rf24network->networkFlags & FLAG_FAST_FRAG) {
/**
 * NAO TENHO FUNCAO TX STAND BY NA MINHA BIBLIOTECA DO MY_NRF24.h
 * */       NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
 	 	 	rf24network->networkFlags &= ~FLAG_FAST_FRAG;
 	 	 	NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
        }
        NRF24_startListening(rf24network->radio);
        uint32_t reply_time = HAL_GetTick();

        while (RF24Network_update(rf24network) != NETWORK_ACK) {
            #if defined(RF24_LINUX)
            delayMicroseconds(900);
            #endif
            if (HAL_GetTick() - reply_time > rf24network->routeTimeout) {
                #if defined(RF24_LINUX)
                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%u: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
                #else
//                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
                #endif
                ok = false;
                break;
            }
        }
    }
    if (!(rf24network->networkFlags & FLAG_FAST_FRAG)) {
        // Now, continue listening
    	NRF24_startListening(rf24network->radio);
    }

    #if defined ENABLE_NETWORK_STATS
    if (ok == true) {
        ++nOK;
    } else {
        ++nFails;
    }
    #endif
    return ok;
}

// Provided the to_node and directTo option, it will return the resulting node and pipe
void RF24Network_logicalToPhysicalAddress(RF24Network_t * const rf24network, logicalToPhysicalStruct *conversionInfo)
{

    //Create pointers so this makes sense.. kind of
    //We take in the to_node(logical) now, at the end of the function, output the send_node(physical) address, etc.
    //back to the original memory address that held the logical information.
    uint16_t *to_node = &conversionInfo->send_node;
    uint8_t *directTo = &conversionInfo->send_pipe;
    bool *multicast = &conversionInfo->multicast;

    // Where do we send this?  By default, to our parent
    uint16_t pre_conversion_send_node = rf24network->parent_node;

    // On which pipe
    uint8_t pre_conversion_send_pipe = rf24network->parent_pipe;

    if (*directTo > TX_ROUTED) {
        pre_conversion_send_node = *to_node;
        *multicast = 1;
        //if(*directTo == USER_TX_MULTICAST || *directTo == USER_TX_TO_PHYSICAL_ADDRESS){
        pre_conversion_send_pipe = 0;
        //}
    }
    // If the node is a direct child,
    else if (RF24Network_is_direct_child(rf24network, *to_node)) {
        // Send directly
        pre_conversion_send_node = *to_node;
        // To its listening pipe
        pre_conversion_send_pipe = 5;
    }
    // If the node is a child of a child
    // talk on our child's listening pipe,
    // and let the direct child relay it.
    else if (RF24Network_is_descendant(rf24network, *to_node)) {
        pre_conversion_send_node = RF24Network_direct_child_route_to(rf24network, *to_node);
        pre_conversion_send_pipe = 5;
    }

    *to_node = pre_conversion_send_node;
    *directTo = pre_conversion_send_pipe;
}

bool RF24Network_write_to_pipe(RF24Network_t * const rf24network, uint16_t node, uint8_t pipe, bool multicast)
{
    bool ok = false;

    // Open the correct pipe for writing.
    // First, stop listening so we can talk

    if (!(rf24network->networkFlags & FLAG_FAST_FRAG)) {
    	NRF24_stopListening(rf24network->radio);
    }

    if (multicast)
    	NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
    else
    	NRF24_setAutoAckPipe(rf24network->radio, 0, 1);

    NRF24_openWritingPipe(rf24network->radio, RF24Network_pipe_address(rf24network, node, pipe));

    ok = NRF24_write(rf24network->radio, rf24network->frame_buffer, rf24network->frame_size); //removed condition for multicasting

    if (!(rf24network->networkFlags & FLAG_FAST_FRAG)) {
        ok = NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
        NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
    }

    /*
    #if defined (__arm__) || defined (RF24_LINUX)
    IF_SERIAL_DEBUG(printf_P(PSTR("%u: MAC Sent on %x %s\n\r"),millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
    #else
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sent on %lx %S\n\r"),millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
    #endif
    */
    return ok;
}

uint16_t RF24Network_direct_child_route_to(RF24Network_t * const rf24network, uint16_t node)
{
    // Presumes that this is in fact a child!!
    uint16_t child_mask = (rf24network->node_mask << 3) | 0x07;
    return node & child_mask;
}

bool RF24Network_is_valid_address(RF24Network_t * const rf24network, uint16_t node)
{
    bool result = true;
    if (node == 0100 || node == 010) {
        return result;
    }
    uint8_t count = 0;
    //#if defined(SERIAL_DEBUG_MINIMAL)
    //uint16_t origNode = node;
    //#endif
    while (node) {
        uint8_t digit = node & 0x07;
        if (digit < 1 || digit > 5) {
            result = false;
            //API_Error_Report(NO_ERROR, ERR_LVL_WARNING, "invalid node address", );
            //printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"), origNode);
            break;
        }
        node >>= 3;
        count++;
    }

    if (count > 4) {
        //IF_SERIAL_DEBUG_MINIMAL(printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"), origNode););
        return false;
    }
    return result;
}

void RF24Network_setup_address(RF24Network_t * const rf24network)
{
    // First, establish the node_mask
    uint16_t node_mask_check = 0xFFFF;
    #if defined(RF24NetworkMulticast)
    uint8_t count = 0;
    #endif

    while (rf24network->node_address & node_mask_check) {
        node_mask_check <<= 3;
        #if defined(RF24NetworkMulticast)
        count++;
    }
    rf24network->multicast_level = count;
        #else
    }
        #endif

	rf24network->node_mask = ~node_mask_check;

    // parent mask is the next level down
    uint16_t parent_mask = rf24network->node_mask >> 3;

    // parent node is the part IN the mask
    rf24network->parent_node = rf24network->node_address & parent_mask;

    // parent pipe is the part OUT of the mask
    uint16_t i = rf24network->node_address;
    uint16_t m = parent_mask;
    while (m) {
        i >>= 3;
        m >>= 3;
    }
    rf24network->parent_pipe = i;

//    IF_SERIAL_DEBUG_MINIMAL(printf_P(PSTR("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r"), node_address, node_mask, parent_node, parent_pipe););
    //  IF_SERIAL_DEBUG_MINIMAL(Serial.print(F("setup_address node=")));
    //  IF_SERIAL_DEBUG_MINIMAL(Serial.print(node_address,OCT));
    //  IF_SERIAL_DEBUG_MINIMAL(Serial.print(F(" parent=")));
    //  IF_SERIAL_DEBUG_MINIMAL(Serial.println(parent_node,OCT));
}

bool RF24Network_is_direct_child(RF24Network_t * const rf24network, uint16_t node)
{
    bool result = false;

    // A direct child of ours has the same low numbers as us, and only
    // one higher number.
    //
    // e.g. node 0234 is a direct child of 034, and node 01234 is a
    // descendant but not a direct child

    // First, is it even a descendant?
    if (RF24Network_is_descendant(rf24network, node)) {
        // Does it only have ONE more level than us?
        uint16_t child_node_mask = (~rf24network->node_mask) << 3;
        result = (node & child_node_mask) == 0;
    }
    return result;
}

bool RF24Network_is_descendant(RF24Network_t * const rf24network, uint16_t node)
{
    return (node & rf24network->node_mask) == rf24network->node_address;
}

uint64_t RF24Network_pipe_address(RF24Network_t * const rf24network, uint16_t node, uint8_t pipe)
{

    static uint8_t address_translation[] = {0xc3, 0x3c, 0x33, 0xce, 0x3e, 0xe3, 0xec};
    uint64_t result = 0xCCCCCCCCCCLL;
    uint8_t *out = (uint8_t *)(&result);

    // Translate the address to use our optimally chosen radio address bytes
    uint8_t count = 1;
    uint16_t dec = node;

    while (dec) {
        #if defined(RF24NetworkMulticast)
        if (pipe != 0 || !node)
        #endif
            out[count] = address_translation[(dec % 8)]; // Convert our decimal values to octal, translate them to address bytes, and set our address

        dec /= 8;
        count++;
    }

    #if defined(RF24NetworkMulticast)
    if (pipe != 0 || !node)
    #endif
        out[0] = address_translation[pipe];
    #if defined(RF24NetworkMulticast)
    else
        out[1] = address_translation[count - 1];
    #endif

//    API_Error_Report(ERR_RADIO, ERR_LVL_LOG, "WRITE MESSAGE HERE");
//    IF_SERIAL_DEBUG(uint32_t *top = reinterpret_cast<uint32_t *>(out + 1); printf_P(PSTR("%lu: NET Pipe %i on node 0%o has address %lx%x\n\r"), millis(), pipe, node, *top, *out));

    return result;
}

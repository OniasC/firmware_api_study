/*
 * tinyNetwork.c
 *
 *  Created on: Oct 31, 2021
 *      Author: onias
 */

#include "tinyNetwork.h"


void tinyNetwork_ctor(tinyNetwork_t * const tiny_network, nrf24_t * const radio)
{
	tiny_network->radio = radio;
	/*configuring radio*/
	tiny_network->radio->payload_size = 32;
	tiny_network->radio->dynamic_payloads_enabled = true;
	tiny_network->radio->addr_width = 5;

	tiny_network->radio->pipe0_reading_address[0] = 0;
	/*end configuring radio*/

	tiny_network->max_frame_payload_size = MAX_FRAME_SIZE - sizeof(tinyNetworkHeader);
	tiny_network->next_frame = tiny_network->frame_queue;
	#if !defined(DISABLE_FRAGMENTATION)
	tiny_network->frag_queue.message_buffer = &(tiny_network->frag_queue_message_buffer[0]);
	tiny_network->frag_ptr = &(tiny_network->frag_queue);
	#endif
	tiny_network->networkFlags = 0;
	tiny_network->returnSysMsgs = 0;
	tiny_network->multicastRelay = 0;
}


void tinyNetwork_begin(tinyNetwork_t * const rf24network, uint8_t _channel, uint16_t _node_address)
{

    rf24network->node_address = _node_address;

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
    tinyNetwork_setup_address(rf24network);

    // Open up all listening pipes
    uint8_t i = 6;
    while (i--)
    	NRF24_openReadingPipe(rf24network->radio, i, tinyNetwork_pipe_address(rf24network, _node_address, i));

    NRF24_startListening(rf24network->radio);
}

uint8_t tinyNetwork_update(tinyNetwork_t * const rf24network)
{

    uint8_t returnVal = 0;
    uint32_t timeout = HAL_GetTick();

    while (NRF24_available(rf24network->radio))
    {
        if (HAL_GetTick() - timeout > 1000) {
            break;
        }
        rf24network->frame_size = NRF24_getDynamicPayloadSize(rf24network->radio);

        // Fetch the payload, and see if this was the last one.
        NRF24_read(rf24network->radio, rf24network->frame_buffer, rf24network->frame_size);

        // Read the beginning of the frame as the header
        tinyNetworkHeader *header = (tinyNetworkHeader *)(&(rf24network->frame_buffer));

        // Throw it away if it's not a valid address or too small
        if (rf24network->frame_size < sizeof(tinyNetworkHeader) ||
        		!tinyNetwork_is_valid_address(rf24network, header->to_node) ||
				!tinyNetwork_is_valid_address(rf24network, header->from_node)) {
        	API_DEBUG_MESSAGE("threw away message -> header wrong\n\r");
            continue;
        }

	    sprintf(rf24network->debugString, "received header: from: %d; to: %d; type: %d; id: %d; reserved: %d\r\n\n", header->from_node, header->to_node, header->type, header->id, header->reserved);
	    API_DEBUG_MESSAGE(rf24network->debugString);
//        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received %s\n\r"), millis(), header->toString()));
//        IF_SERIAL_DEBUG(const uint16_t *i = reinterpret_cast<const uint16_t *>(frame_buffer + sizeof(tinyNetworkHeader)); printf_P(PSTR("%lu: NET message %04x\n\r"), millis(), *i));

        returnVal = header->type;
        // Is this for us?
        if (header->to_node == rf24network->node_address)
        {
            if (header->type == NETWORK_PING) {
                continue;
            }
            if (header->type == NETWORK_ADDR_RESPONSE) {
                uint16_t requester = NETWORK_DEFAULT_ADDRESS;
                if (requester != rf24network->node_address) {
                    header->to_node = requester;
                    tinyNetwork_write_2_params(rf24network, header->to_node, USER_TX_TO_PHYSICAL_ADDRESS);
                    continue;
                }
            }
            if (header->type == NETWORK_REQ_ADDRESS && rf24network->node_address) {
                header->from_node = rf24network->node_address;
                header->to_node = 0;
                tinyNetwork_write_2_params(rf24network, header->to_node, TX_NORMAL);
                continue;
            }
            if ((rf24network->returnSysMsgs && header->type > 127) || header->type == NETWORK_ACK) {
//                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu MAC: System payload rcvd %d\n"), millis(), returnVal););
                if (header->type != NETWORK_FIRST_FRAGMENT && header->type != NETWORK_MORE_FRAGMENTS && header->type != EXTERNAL_DATA_TYPE && header->type != NETWORK_LAST_FRAGMENT) {
                    return returnVal;
                }
            }

            if (tinyNetwork_enqueue(rf24network, header) == 2) { //External data received
                return EXTERNAL_DATA_TYPE;
            }
        }/*
        else
        {
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

            if (rf24network->node_address != NETWORK_DEFAULT_ADDRESS) {
            	RF24Network_write_2_params(rf24network, header->to_node, 1); //Send it on, indicate it is a routed payload
                returnVal = 0;
            }

        }*/

    } // radio.available()
    return returnVal;
}

uint8_t tinyNetwork_enqueue(tinyNetwork_t * const rf24network, tinyNetworkHeader *header)
{
	/* This functions places the newly received frame into the frame_queue */
	/* Its a very important function! */
    bool result = false;
    uint16_t message_size = rf24network->frame_size - sizeof(tinyNetworkHeader);

//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue @%x "), millis(), next_frame - frame_queue));

    #if !defined(DISABLE_FRAGMENTATION)

    bool isFragment = header->type == NETWORK_FIRST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS || header->type == NETWORK_LAST_FRAGMENT;

    if (isFragment) {

        if (header->type == NETWORK_FIRST_FRAGMENT) {

            memcpy(&(rf24network->frag_queue), &(rf24network->frame_buffer), sizeof(tinyNetworkHeader));
            memcpy(rf24network->frag_queue.message_buffer, rf24network->frame_buffer + sizeof(tinyNetworkHeader), message_size);

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

            memcpy(rf24network->frag_queue.message_buffer + rf24network->frag_queue.message_size, rf24network->frame_buffer + sizeof(tinyNetworkHeader), message_size);
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
        rf24network->frag_queue.message_buffer = rf24network->frame_buffer + sizeof(tinyNetworkHeader);
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

bool tinyNetwork_available(tinyNetwork_t * const rf24network)
{
	return (rf24network->next_frame > rf24network->frame_queue);
}

uint16_t tinyNetwork_read(tinyNetwork_t * const rf24network, tinyNetworkHeader *header, void *message, uint16_t maxlen)
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

bool tinyNetwork_write_message(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
	/* doing without fragmentation at the beginning */
	 if (len <= rf24network->max_frame_payload_size)
	 {
		 //Normal Write (Un-Fragmented)
		 rf24network->frame_size = len + sizeof(tinyNetworkHeader);
		 if (tinyNetwork_write(rf24network, header, message, len, writeDirect))
		 {
			 return 1;
		 }
		 else
		 {
			 return 0;
		 }
	 }
	 return 0;
}

bool tinyNetwork_write(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
    // Fill out the header
    header->from_node = rf24network->node_address;

    // Build the full frame to send
    memcpy(rf24network->frame_buffer, header, sizeof(tinyNetworkHeader));

//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"), millis(), header.toString()));
    if (len)
    {
        memcpy(rf24network->frame_buffer + sizeof(tinyNetworkHeader), message, len);
    }

    return tinyNetwork_write_2_params(rf24network, header->to_node, TX_NORMAL);
}

bool tinyNetwork_write_2_params(tinyNetwork_t * const rf24network, uint16_t to_node, uint8_t directTo)
{
	// Direct To: 	0 = First Payload, standard routing,
	//				1 = routed payload,
	//				2 = directRoute to host,
	//				3 = directRoute to Route
    bool ok = false;
    bool isAckType = false;
    if (rf24network->frame_buffer[6] > 64 && rf24network->frame_buffer[6] < 192)
        isAckType = true;

    //Load info into our conversion structure, and get the converted address info
    tinyLogicalToPhysicalStruct conversion = {to_node, directTo, 0};
    tinyNetwork_logicalToPhysicalAddress(rf24network, &conversion);

    sprintf(DEBUG_STRING, "%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r", HAL_GetTick(), to_node, conversion.send_node, conversion.send_pipe);
    API_DEBUG_MESSAGE(DEBUG_STRING);

//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe));
    /**Write it*/
    if (directTo == TX_ROUTED && conversion.send_node == to_node && isAckType) {
        HAL_Delay(2);
    }

    ok = tinyNetwork_write_to_pipe(rf24network, conversion.send_node, conversion.send_pipe, conversion.multicast);

    if (ok)
    {
    	sprintf(DEBUG_STRING, "\t%lu: MAC Sent to 0%o via 0%o on pipe %x\n\r", HAL_GetTick(), to_node, conversion.send_node, conversion.send_pipe);
    } else {
    	sprintf(DEBUG_STRING, "\t%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r", HAL_GetTick(), to_node, conversion.send_node, conversion.send_pipe);
    }
    API_DEBUG_MESSAGE(DEBUG_STRING);

    /*
     *
    if (ok && conversion.send_node != to_node && (directTo == 0 || directTo == 3) && isAckType)
    {
        // Now, continue listening
        if (rf24network->networkFlags & FLAG_FAST_FRAG)
        {
        	//NAO TENHO FUNCAO TX STAND BY NA MINHA BIBLIOTECA DO MY_NRF24.h
        	NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
 	 	 	rf24network->networkFlags &= ~FLAG_FAST_FRAG;
 	 	 	NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
        }
        NRF24_startListening(rf24network->radio);
        uint32_t reply_time = HAL_GetTick();

        while (tinyNetwork_update(rf24network) != NETWORK_ACK)
        {
            if (HAL_GetTick() - reply_time > rf24network->routeTimeout)
            {
//                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
                ok = false;
                break;
            }
        }
    }

    if (!(rf24network->networkFlags & FLAG_FAST_FRAG))
    {
        // Now, continue listening
    	NRF24_startListening(rf24network->radio);
    }
    *
    */

    return ok;
}


bool tinyNetwork_write_to_pipe(tinyNetwork_t * const rf24network, uint16_t node, uint8_t pipe, bool multicast)
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

    NRF24_openWritingPipe(rf24network->radio, tinyNetwork_pipe_address(rf24network, node, pipe));

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

// Provided the to_node and directTo option, it will return the resulting node and pipe
void tinyNetwork_logicalToPhysicalAddress(tinyNetwork_t * const rf24network, tinyLogicalToPhysicalStruct *conversionInfo)
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
    /*else if (RF24Network_is_direct_child(rf24network, *to_node)) {
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
    }*/

    *to_node = pre_conversion_send_node;
    *directTo = pre_conversion_send_pipe;
}

void tinyNetwork_setup_address(tinyNetwork_t * const rf24network)
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

uint64_t tinyNetwork_pipe_address(tinyNetwork_t * const rf24network, uint16_t node, uint8_t pipe)
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

bool tinyNetwork_is_valid_address(tinyNetwork_t * const rf24network, uint16_t node)
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

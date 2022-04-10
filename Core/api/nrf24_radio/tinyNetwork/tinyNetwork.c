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
	/*tiny_network->radio->payload_size = 32;
	tiny_network->radio->dynamic_payloads_enabled = true;
	tiny_network->radio->addr_width = 5;

	tiny_network->radio->pipe0_reading_address[0] = 0;*/
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
    //NRF24_setAutoAckPipe(rf24network->radio, 0, 0);

    //#if defined(ENABLE_DYNAMIC_PAYLOADS)
    //NRF24_enableDynamicPayloads(rf24network->radio);
    //#endif

    // Use different retry periods to reduce data collisions
    uint8_t retryVar = (((rf24network->node_address % 6) + 1) * 2) + 3;
    NRF24_setRetries(rf24network->radio, retryVar, 5);// max about 85ms per attempt
    rf24network->txTimeout = 25;

    rf24network->routeTimeout = rf24network->txTimeout * 3;  // Adjust for max delay per node within a single chain

    // Setup our address helper cache
    //tinyNetwork_setup_address(rf24network);

    // Open up all listening pipes
    /*uint8_t i = 6;
    while (i--)
    {
    	NRF24_openReadingPipe(rf24network->radio, i, basePipeAddress+i*3);
    }*/

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
        	API_DEBUG_MESSAGE_2("threw away message -> header wrong\n\r");
            continue;
        }

        API_DEBUG_MESSAGE_2("received header: from: %d; to: %d; type: %d; id: %d; reserved: %d\r\n\n", header->from_node, header->to_node, header->type, header->id, header->reserved);

        returnVal = header->type;
        // Is this for us?
        if (header->to_node == rf24network->node_address)
        {
            if ((rf24network->returnSysMsgs && header->type > 127) || header->type == NETWORK_ACK)
            {
            	API_DEBUG_MESSAGE_2("%lu MAC: System payload rcvd %d\n", HAL_GetTick(), returnVal);
                if (header->type != NETWORK_FIRST_FRAGMENT &&
                	header->type != NETWORK_MORE_FRAGMENTS &&
					header->type != EXTERNAL_DATA_TYPE 	   &&
					header->type != NETWORK_LAST_FRAGMENT)
                {
                    return returnVal;
                }
            }
            if (tinyNetwork_enqueue(rf24network, header) == 2) { //External data received
                return EXTERNAL_DATA_TYPE;
            }
        }
    }
    return returnVal;
}

uint8_t tinyNetwork_enqueue(tinyNetwork_t * const rf24network, tinyNetworkHeader *header)
{
	bool result = false;
    uint16_t message_size = rf24network->frame_size - sizeof(tinyNetworkHeader);
    uint8_t offset = 0U;

    // Copy the current frame into the frame queue
    if (message_size + (rf24network->next_frame - rf24network->frame_queue) <= MAIN_BUFFER_SIZE)
    {
        memcpy(rf24network->next_frame, &(rf24network->frame_buffer), sizeof(rf24network->frag_queue.header));
        offset += sizeof(rf24network->frag_queue.header);
        memcpy(rf24network->next_frame + offset, &message_size, sizeof(rf24network->frag_queue.message_size));
        offset += sizeof(rf24network->frag_queue.message_size);
        memcpy(rf24network->next_frame + offset, rf24network->frame_buffer + 8, message_size);

        for(int i=0; i<message_size;i++)
        {
        	API_DEBUG_MESSAGE_2("%u : ", rf24network->next_frame[i]);
        }
        API_DEBUG_MESSAGE("\n\r");

        rf24network->next_frame += (message_size + offset);
        #if !defined(ARDUINO_ARCH_AVR)
        uint8_t padding;
		padding = (rf24network->frag_queue.message_size + offset) % 4;
		if (padding) {
        	rf24network->next_frame += 4 - padding;
        }
        #endif
//        IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print("Enq "); Serial.println(next_frame-frame_queue); );//printf_P(PSTR("enq %d\n"),next_frame-frame_queue); );
		offset = 0U;
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

    API_DEBUG_MESSAGE_2("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r", HAL_GetTick(), to_node, conversion.send_node, conversion.send_pipe);

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

    /*if (!(rf24network->networkFlags & FLAG_FAST_FRAG)) {
        ok = NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
        NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
    }*/

    return ok;
}

bool tinyNetwork_write_test(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len)
{
    bool ok = false;
    char AckPayload[32];
	/* doing without fragmentation at the beginning */
	 if (len <= rf24network->max_frame_payload_size)
	 {
		 //Normal Write (Un-Fragmented)
		 rf24network->frame_size = len + sizeof(tinyNetworkHeader);

		header->from_node = rf24network->node_address;

		// Build the full frame to send
		memcpy(rf24network->frame_buffer, header, sizeof(tinyNetworkHeader));

	//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"), millis(), header.toString()));
		if (len)
		{
			memcpy(rf24network->frame_buffer + sizeof(tinyNetworkHeader), message, len);
		}


		 NRF24_stopListening(rf24network->radio);
		 NRF24_openWritingPipe(rf24network->radio, rf24network->radio->writing_address);

		if(NRF24_write(rf24network->radio, rf24network->frame_buffer, rf24network->frame_size))
		{
			API_DEBUG_MESSAGE("Transmitted Successfully\r\n");
			NRF24_read(rf24network->radio, AckPayload, 32);
			API_DEBUG_MESSAGE_2("AckPayload:  %s \r\n", AckPayload);
			ok = true;
		}

		uint64_t temp = 0;
		temp = temp | ((uint64_t)rf24network->radio->pipes1to5_MSB_reading_address[3] << 32);
		temp = temp | ((uint64_t)rf24network->radio->pipes1to5_MSB_reading_address[2] << 24);
		temp = temp | ((uint64_t)rf24network->radio->pipes1to5_MSB_reading_address[1] << 16);
		temp = temp | ((uint64_t)rf24network->radio->pipes1to5_MSB_reading_address[0] << 8) ;
		temp = temp | (uint64_t)rf24network->radio->pipes1to5_LSB_reading_address[0];

		NRF24_openReadingPipe(rf24network->radio, 1, temp);
		NRF24_startListening(rf24network->radio);


	 }
	 return 0;

    // Open the correct pipe for writing.
    // First, stop listening so we can talk


    ok = NRF24_write(rf24network->radio, rf24network->frame_buffer, rf24network->frame_size); //removed condition for multicasting

    /*if (!(rf24network->networkFlags & FLAG_FAST_FRAG)) {
        ok = NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
        NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
    }*/

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

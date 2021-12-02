/*
 * tinyNetwork.c
 *
 *  Created on: Oct 31, 2021
 *      Author: onias
 */

#include "tinyNetwork.h"
#include "../radio/MY_NRF24.h"

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


bool tinyNetwork_write_message(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
	#if defined(DISABLE_FRAGMENTATION)
	 frame_size = rf24_min(len + sizeof(RF24NetworkHeader), MAX_FRAME_SIZE);
	 return _write(header, message, rf24_min(len, max_frame_payload_size), writeDirect);
	 #else // !defined(DISABLE_FRAGMENTATION)
	 if (len <= rf24network->max_frame_payload_size) {
		 //Normal Write (Un-Fragmented)
		 rf24network->frame_size = len + sizeof(tinyNetworkHeader);
		 if (tinyNetwork_write(rf24network, header, message, len, writeDirect)) {
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
		 rf24network->frame_size = sizeof(tinyNetworkHeader) + fragmentLen;
		 ok = tinyNetwork_write(rf24network, header, ((char *)message) + offset, fragmentLen, writeDirect);

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

bool tinyNetwork_write(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect)
{
    // Fill out the header
    header->from_node = rf24network->node_address;

    // Build the full frame to send
    //memcpy(rf24network->frame_buffer, &header, sizeof(RF24NetworkHeader));
    memcpy(rf24network->frame_buffer, header, sizeof(tinyNetworkHeader));

    #if defined(RF24_LINUX)
//    IF_SERIAL_DEBUG(printf_P(PSTR("%u: NET Sending %s\n\r"), millis(), header.toString()));
    #else
//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"), millis(), header.toString()));
    #endif
    if (len) {
        memcpy(rf24network->frame_buffer + sizeof(tinyNetworkHeader), message, len);
//        IF_SERIAL_DEBUG(uint16_t tmpLen = len; printf_P(PSTR("%lu: NET message "), millis()); const uint8_t *charPtr = reinterpret_cast<const uint8_t *>(message); while (tmpLen--) { printf("%02x ", charPtr[tmpLen]); } printf_P(PSTR("\n\r")));
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

//    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe));
    /**Write it*/
    if (directTo == TX_ROUTED && conversion.send_node == to_node && isAckType) {
        HAL_Delay(2);
    }
    ok = tinyNetwork_write_to_pipe(rf24network, conversion.send_node, conversion.send_pipe, conversion.multicast);

    if (!ok) {
//        IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
    }

    if (directTo == TX_ROUTED && ok && conversion.send_node == to_node && isAckType) {

    	tinyNetworkHeader *header = (tinyNetworkHeader *)&(rf24network->frame_buffer);
        header->type = NETWORK_ACK;          // Set the payload type to NETWORK_ACK
        header->to_node = header->from_node; // Change the 'to' address to the 'from' address

        conversion.send_node = header->from_node;
        conversion.send_pipe = TX_ROUTED;
        conversion.multicast = 0;
        tinyNetwork_logicalToPhysicalAddress(rf24network, &conversion);

        //Write the data using the resulting physical address
        rf24network->frame_size = sizeof(tinyNetworkHeader);
        tinyNetwork_write_to_pipe(rf24network, conversion.send_node, conversion.send_pipe, conversion.multicast);

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

       //NAO TENHO FUNCAO TX STAND BY NA MINHA BIBLIOTECA DO MY_NRF24.h
       NRF24_txStandBy(rf24network->radio, rf24network->txTimeout, 0);
 	 	 	rf24network->networkFlags &= ~FLAG_FAST_FRAG;
 	 	 	NRF24_setAutoAckPipe(rf24network->radio, 0, 0);
        }
        NRF24_startListening(rf24network->radio);
        uint32_t reply_time = HAL_GetTick();

        /*while (tinyNetwork_update(rf24network) != NETWORK_ACK) {
            if (HAL_GetTick() - reply_time > rf24network->routeTimeout) {
//                IF_SERIAL_DEBUG_ROUTING(printf_P(PSTR("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"), millis(), to_node, conversion.send_node, conversion.send_pipe););
                ok = false;
                break;
            }
        }*/
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

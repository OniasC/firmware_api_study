/*
 * tinyNetwork.h
 *
 *  Created on: Oct 31, 2021
 *      Author: onias
 */

#ifndef API_TINYNETWORK_TINYNETWORK_H_
#define API_TINYNETWORK_TINYNETWORK_H_

#include "tinyNetwork_config.h"
#include "main.h"
#include "../api.h"
#include "../states.h"
#include "../Error_Report.h"

#include <stddef.h>
#include <stdint.h>

/* Header types range */
#define MIN_USER_DEFINED_HEADER_TYPE 0
#define MAX_USER_DEFINED_HEADER_TYPE 127

// ACK Response Types
/**
 * **Reserved network message types**
 *
 * The network will determine whether to automatically acknowledge payloads based on their general type <br>
 *
 * **User types** (1-127) 1-64 will NOT be acknowledged <br>
 * **System types** (128-255) 192 through 255 will NOT be acknowledged<br>
 *
 * @defgroup DEFINED_TYPES Reserved System Message Types
 *
 * System types can also contain message data.
 *
 * @{
 */

/**
 * A NETWORK_ADDR_RESPONSE type is utilized to manually route custom messages containing a single RF24Network address
 *
 * Used by RF24Mesh
 *
 * If a node receives a message of this type that is directly addressed to it, it will read the included message, and forward the payload
 * on to the proper recipient. <br>
 * This allows nodes to forward multicast messages to the master node, receive a response, and forward it back to the requester.
 */
#define NETWORK_ADDR_RESPONSE 128

/**
 * Messages of type NETWORK_PING will be dropped automatically by the recipient. A NETWORK_ACK or automatic radio-ack will indicate to the sender whether the
 * payload was successful. The time it takes to successfully send a NETWORK_PING is the round-trip-time.
 */
#define NETWORK_PING 130

/**
 * External data types are used to define messages that will be passed to an external data system. This allows RF24Network to route and pass any type of data, such
 * as TCP/IP frames, while still being able to utilize standard RF24Network messages etc.
 *
 * **Linux**
 * Linux devices (defined RF24_LINUX) will buffer all data types in the user cache.
 *
 * **Arduino/AVR/Etc:** Data transmitted with the type set to EXTERNAL_DATA_TYPE will not be loaded into the user cache. <br>
 * External systems can extract external data using the following process, while internal data types are cached in the user buffer, and accessed using network.read() :
 * @code
 * uint8_t return_type = network.update();
 * if(return_type == EXTERNAL_DATA_TYPE){
 *     uint16_t size = network.frag_ptr->message_size;
 *     memcpy(&myDataBuffer, network.frag_ptr->message_buffer, network.frag_ptr->message_size);
 * }
 * @endcode
 */
#define EXTERNAL_DATA_TYPE 131

/**
 * Messages of this type designate the first of two or more message fragments, and will be re-assembled automatically.
 */
#define NETWORK_FIRST_FRAGMENT 148

/**
 * Messages of this type indicate a fragmented payload with two or more message fragments.
 */
#define NETWORK_MORE_FRAGMENTS 149

/**
 * Messages of this type indicate the last fragment in a sequence of message fragments.
 * Messages of this type do not receive a NETWORK_ACK
 */
#define NETWORK_LAST_FRAGMENT 150
//#define NETWORK_LAST_FRAGMENT 201

// NO ACK Response Types
//#define NETWORK_ACK_REQUEST 192

/**
 * Messages of this type are used internally, to signal the sender that a transmission has been completed.
 * RF24Network does not directly have a built-in transport layer protocol, so message delivery is not 100% guaranteed.<br>
 * Messages can be lost via corrupted dynamic payloads, or a NETWORK_ACK can fail, while the message was actually successful.
 *
 * NETWORK_ACK messages can be utilized as a traffic/flow control mechanism, since transmitting nodes will be forced to wait until
 * the payload is transmitted across the network and acknowledged, before sending additional data.
 *
 * In the event that the transmitting device will be waiting for a direct response, manually sent by the recipient, a NETWORK_ACK is not required. <br>
 * User messages utilizing a 'type' with a decimal value of 64 or less will not be acknowledged across the network via NETWORK_ACK messages.
 */
#define NETWORK_ACK 193

/**
 * Used by RF24Mesh
 *
 * Messages of this type are used with multi-casting , to find active/available nodes.
 * Any node receiving a NETWORK_POLL sent to a multicast address will respond directly to the sender with a blank message, indicating the
 * address of the available node via the header.
 */
#define NETWORK_POLL 194

/**
 * Used by RF24Mesh
 *
 * Messages of this type are used to request information from the master node, generally via a unicast (direct) write.
 * Any (non-master) node receiving a message of this type will manually forward it to the master node using a normal network write.
 */
#define NETWORK_REQ_ADDRESS 195
//#define NETWORK_ADDR_LOOKUP 196
//#define NETWORK_ADDR_RELEASE 197
/** @} */

#define NETWORK_MORE_FRAGMENTS_NACK 200

/** Internal defines for handling written payloads */
#define TX_NORMAL 0
#define TX_ROUTED 1
#define USER_TX_TO_PHYSICAL_ADDRESS 2 // no network ACK
#define USER_TX_TO_LOGICAL_ADDRESS 3  // network ACK
#define USER_TX_MULTICAST 4

#define MAX_FRAME_SIZE 32    // Size of individual radio frames
#define FRAME_HEADER_SIZE 10 // Size of RF24Network frames - data

#define USE_CURRENT_CHANNEL 255 // Use current radio channel when setting up the network

/** Internal defines for handling internal payloads - prevents reading additional data from the radio
 * when buffers are full
 */
#define FLAG_HOLD_INCOMING 1
/** FLAG_BYPASS_HOLDS is mainly for use with RF24Mesh as follows:
 * a: Ensure no data in radio buffers, else exit
 * b: Address is changed to multicast address for renewal
 * c: Holds Cleared (bypass flag is set)
 * d: Address renewal takes place and is set
 * e: Holds Enabled (bypass flag off)
 */
#define FLAG_BYPASS_HOLDS 2
#define FLAG_FAST_FRAG 4
#define FLAG_NO_POLL 8

/**
 * Header which is sent with each message
 *
 * The frame put over the air consists of this header and a message
 *
 * Headers are addressed to the appropriate node, and the network forwards them on to their final destination.
 */
typedef struct
{
    uint8_t from_node; /** Logical address where the message was generated */
    uint8_t to_node;   /** Logical address where the message is going */
    uint16_t id;        /** Sequential message ID, incremented every time a new frame is constructed */
    /**
     * Message Types:
     * User message types 1 through 64 will NOT be acknowledged by the network, while message types 65 through 127 will receive a network ACK.
     * System message types 192 through 255 will NOT be acknowledged by the network. Message types 128 through 192 will receive a network ACK. <br>
     * <br><br>
     */
    unsigned char type; /** <b>Type of the packet.</b> 0-127 are user-defined types, 128-255 are reserved for system */

    /**
     * During fragmentation, it carries the fragment_id, and on the last fragment
     * it carries the header_type.<br>
     */
    unsigned char reserved; /** *Reserved for system use* */
    /**
	 * Send constructor
	 *
	 * @note Now supports automatic fragmentation for very long messages, which can be sent as usual if fragmentation is enabled.
	 *
	 * Fragmentation is enabled by default for all devices except ATTiny <br>
	 * Configure fragmentation and max payload size in RF24Network_config.h
	 *
	 * Use this constructor to create a header and then send a message
	 *
	 * @code
	 * uint16_t recipient_address = 011;
	 *
	 * RF24NetworkHeader header(recipient_address, 't');
	 *
	 * network.write(header, &message, sizeof(message));
	 * @endcode
	 *
	 * @param _to The Octal format, logical node address where the message is going
	 * @param _type The type of message which follows.  Only 0-127 are allowed for
	 * user messages. Types 1-64 will not receive a network acknowledgement.
	 */
} tinyNetworkHeader;

/**
 * Frame structure for internal message handling, and for use by external applications
 *
 * The actual frame put over the air consists of a header (8-bytes) and a message payload (Up to 24-bytes)<br>
 * When data is received, it is stored using the RF24NetworkFrame structure, which includes:
 * 1. The header
 * 2. The size of the included message
 * 3. The 'message' or data being received
 *
 */
 typedef struct
 {
     tinyNetworkHeader header; /** Header which is sent with each message */
     uint16_t message_size;    /** The size in bytes of the payload length */
     uint8_t *message_buffer; //< Pointer to the buffer storing the actual message
 } tinyNetworkFrame;

typedef struct
{
	uint16_t send_node;
	uint8_t send_pipe;
	bool multicast;
} tinyLogicalToPhysicalStruct;

typedef struct
{
	nrf24_t *radio;
	bool multicastRelay;
	/**
	 * @note: This value is automatically assigned based on the node address
	 * to reduce errors and increase throughput of the network.
	 *
	 * Sets the timeout period for individual payloads in milliseconds at staggered intervals.
	 * Payloads will be retried automatically until success or timeout
	 * Set to 0 to use the normal auto retry period defined by radio.setRetries()
	 *
	 */
	uint32_t txTimeout; /** Network timeout value */

	/**
	 * This only affects payloads that are routed by one or more nodes.
	 * This specifies how long to wait for an ack from across the network.
	 * Radios sending directly to their parent or children nodes do not
	 * utilize this value.
	 */
	uint16_t routeTimeout; /** Timeout for routed payloads */
	/** The raw system frame buffer of received data. */
	uint8_t frame_buffer[MAX_FRAME_SIZE];
	tinyNetworkFrame *frag_ptr;
	bool returnSysMsgs;
	uint8_t networkFlags;

	uint8_t multicast_level;

	uint16_t node_address; /** Logical node address of this unit, 1 .. UINT_MAX */
	uint8_t frame_size;
	unsigned int max_frame_payload_size;// = MAX_FRAME_SIZE - sizeof(tinyNetworkHeader);
	uint8_t frame_queue[MAIN_BUFFER_SIZE]; /** Space for a small set of frames that need to be delivered to the app layer */
	uint8_t *next_frame; /** Pointer into the @p frame_queue where we should place the next received frame */

	#if !defined(DISABLE_FRAGMENTATION)
	tinyNetworkFrame frag_queue;
	uint8_t frag_queue_message_buffer[MAX_PAYLOAD_SIZE]; //frame size + 1
	#endif


	uint16_t parent_node; /** Our parent's node address */
	uint8_t parent_pipe;  /** The pipe our parent uses to listen to us */
	uint16_t node_mask;   /** The bits which contain significant node address information */

} tinyNetwork_t;

void tinyNetwork_ctor(tinyNetwork_t * const tiny_network, nrf24_t * const radio);
void tinyNetwork_begin(tinyNetwork_t * const rf24network, uint8_t _channel, uint16_t _node_address);
bool tinyNetwork_write_message(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect);

bool tinyNetwork_write(tinyNetwork_t * const rf24network, tinyNetworkHeader * header, const void *message, uint16_t len, uint16_t writeDirect);

bool tinyNetwork_write_2_params(tinyNetwork_t * const rf24network, uint16_t to_node, uint8_t directTo); // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
bool tinyNetwork_write_to_pipe(tinyNetwork_t * const rf24network, uint16_t node, uint8_t pipe, bool multicast);
void tinyNetwork_logicalToPhysicalAddress(tinyNetwork_t * const rf24network, tinyLogicalToPhysicalStruct *conversionInfo);
void tinyNetwork_setup_address(tinyNetwork_t * const rf24network);
uint64_t tinyNetwork_pipe_address(tinyNetwork_t * const rf24network, uint16_t node, uint8_t pipe);
#endif /* API_TINYNETWORK_TINYNETWORK_H_ */

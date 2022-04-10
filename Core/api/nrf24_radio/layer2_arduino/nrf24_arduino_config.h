/*
 * nrf24_arduino_config.h
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#ifndef API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_CONFIG_H_
#define API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_CONFIG_H_

/**
 * User access to internally used delay time (in microseconds) during RF24::powerUp()
 * @warning This default value compensates for all supported hardware. Only adjust this if you
 * know your radio's hardware is, in fact, genuine and reliable.
 */
#if !defined(RF24_POWERUP_DELAY)
#define RF24_POWERUP_DELAY	5000
#endif

/**********************/
#define rf24_max(a, b) (a>b?a:b)
#define rf24_min(a, b) (a<b?a:b)
#define pgm_read_byte(p) (*(p))
 #define _BV(x) (1<<(x))

#define RF24_SPI_SPEED 10000000

    #ifndef NETWORK_DEFAULT_ADDRESS
        #define NETWORK_DEFAULT_ADDRESS 04444
    #endif // NETWORK_DEFAULT_ADDRESS

   // Different set of defaults for ATTiny - fragmentation is disabled and user payloads are set to 3 max
        /********** USER CONFIG - ATTiny **************/
        //#define ENABLE_SLEEP_MODE  //AVR only
        #define RF24NetworkMulticast
        #define MAX_PAYLOAD_SIZE 72
        #define MAIN_BUFFER_SIZE (MAX_PAYLOAD_SIZE + FRAME_HEADER_SIZE)
        #define DISABLE_FRAGMENTATION
        // Enable MAX PAYLOAD SIZE if enabling fragmentation
        //#define MAX_PAYLOAD_SIZE  MAIN_BUFFER_SIZE-10
        #define ENABLE_DYNAMIC_PAYLOADS
        //#define DISABLE_USER_PAYLOADS


#endif /* API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_CONFIG_H_ */

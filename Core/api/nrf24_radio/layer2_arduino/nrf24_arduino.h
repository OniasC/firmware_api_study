/*
 * nrf24_arduino.h
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#ifndef API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_H_
#define API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_H_


#include "nrf24_arduino_config.h"
#include "main.h"
#include "../../api.h"
#include "../../states.h"
#include "../../Error_Report.h"
#include "../api/api_hal/api_hal_gpio.h"
#include "../api/api_hal/api_hal.h"
#include "../api/api_hal/api_hal_spi.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

static const uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2,
                                                    ERX_P3, ERX_P4, ERX_P5};
static const uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2,
                                             RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

typedef enum {
	RF24_MODE_RX,
	RF24_MODE_TX
} nrf24_mode_e;
/**
 * @defgroup PALevel Power Amplifier level
 * Power Amplifier level. The units dBm (decibel-milliwatts or dB<sub>mW</sub>)
 * represents a logarithmic signal loss.
 * @see RF24::setPALevel()
 * @see RF24::getPALevel()
 * @{
 */
typedef enum {
    /**
     * (0) represents:
     * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
     * :-------:|:-----------------------------:|:----------------------------:
     *  -18 dBm | -6 dBm | -12 dBm
     */
    RF24_PA_MIN = 0,
    /**
     * (1) represents:
     * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
     * :-------:|:-----------------------------:|:----------------------------:
     *  -12 dBm | 0 dBm | -4 dBm
     */
    RF24_PA_LOW,
    /**
     * (2) represents:
     * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
     * :-------:|:-----------------------------:|:----------------------------:
     *  -6 dBm | 3 dBm | 1 dBm
     */
    RF24_PA_HIGH,
    /**
     * (3) represents:
     * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
     * :-------:|:-----------------------------:|:----------------------------:
     *  0 dBm | 7 dBm | 4 dBm
     */
    RF24_PA_MAX,
    /**
     * (4) This should not be used and remains for backward compatibility.
     */
    RF24_PA_ERROR
} rf24_pa_dbm_e;

/**
 * @}
 * @defgroup Datarate datarate
 * How fast data moves through the air. Units are in bits per second (bps).
 * @see RF24::setDataRate()
 * @see RF24::getDataRate()
 * @{
 */
typedef enum {
    /** (0) represents 1 Mbps */
    RF24_1MBPS = 0,
    /** (1) represents 2 Mbps */
    RF24_2MBPS,
    /** (2) represents 250 kbps */
    RF24_250KBPS
} rf24_datarate_e;

/**
 * @}
 * @defgroup CRCLength CRC length
 * The length of a CRC checksum that is used (if any).<br>Cyclical Redundancy
 * Checking (CRC) is commonly used to ensure data integrity.
 * @see RF24::setCRCLength()
 * @see RF24::getCRCLength()
 * @see RF24::disableCRC()
 * @{
 */
typedef enum {
    /** (0) represents no CRC checksum is used */
    RF24_CRC_DISABLED = 0,
    /** (1) represents CRC 8 bit checksum is used */
    RF24_CRC_8,
    /** (2) represents CRC 16 bit checksum is used */
    RF24_CRC_16
} rf24_crclength_e;

/**
 * @}
 * @brief Driver class for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

typedef struct {
	spi_t * chip_spi;
	io_pin_t ce_pin;
	io_pin_t csn_pin;
	io_pin_t chip_int;

	rf24_pa_dbm_e power_level;
	rf24_datarate_e data_rate;
	rf24_crclength_e crc_length;

    uint32_t spi_speed; /**< SPI Bus Speed */
	bool enableAutoAck;
	nrf24_mode_e mode;

    uint8_t status; /** The status byte returned from every SPI transaction */
    uint8_t payload_size; /**< Fixed size of payloads */
    bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
    bool ack_payloads_enabled; /**< Whether ack payloads are enabled. */
    uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
    uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */
    uint8_t config_reg; /**< For storing the value of the NRF_CONFIG register */
    bool _is_p_variant; /** For storing the result of testing the toggleFeatures() affect */
    bool failureDetected;

    uint32_t txDelay;
    uint32_t csDelay;
} RF24_t;

void beginTransaction();

void endTransaction();


void RF24(io_pin_t _cepin, io_pin_t _cspin, uint32_t _spi_speed);

bool begin(spi_t _spi, io_pin_t _cepin, io_pin_t _cspin);

bool isChipConnected();

void startListening(void);

void stopListening(void);


bool available(void);


void read(void* buf, uint8_t len);

bool write(const void* buf, uint8_t len);

bool writeMulticast(const void* buf, uint8_t len, const bool multicast);

void openWritingPipePointer(const uint8_t* address);

void openWritingPipe(uint64_t address);

void openReadingPipePointer(uint8_t number, const uint8_t* address);

void openReadingPipe(uint8_t number, uint64_t address);


void printDetails(void);

void printPrettyDetails(void);

bool available2(uint8_t* pipe_num);

bool rxFifoFull();

void powerDown(void);

void powerUp(void);


bool writeFast(const void* buf, uint8_t len);


bool writeFastMulticast(const void* buf, uint8_t len, const bool multicast);


bool writeBlocking(const void* buf, uint8_t len, uint32_t timeout);


bool txStandBy();


bool txStandBy_2(uint32_t timeout, bool startTx);


bool writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);


void  whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready);


void startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx);


bool startWrite(const void* buf, uint8_t len, const bool multicast);


void reUseTX();


uint8_t flush_tx(void);


uint8_t flush_rx(void);


bool testCarrier(void);


bool testRPD(void);


bool isValid();


void closeReadingPipe(uint8_t pipe);


void setAddressWidth(uint8_t a_width);

void setRetries(uint8_t delay, uint8_t count);

void setChannel(uint8_t channel);

uint8_t getChannel(void);


void setPayloadSize(uint8_t size);

uint8_t getPayloadSize(void);

uint8_t getDynamicPayloadSize(void);

void enableAckPayload(void);

void disableAckPayload(void);


void enableDynamicPayloads(void);

void disableDynamicPayloads(void);

void enableDynamicAck();

bool isPVariant(void);

void setAutoAck(bool enable);

void setAutoAck_2(uint8_t pipe, bool enable);


void setPALevel(uint8_t level, bool lnaEnable);

uint8_t getPALevel(void);

uint8_t getARC(void);

bool setDataRate(rf24_datarate_e speed);

rf24_datarate_e getDataRate(void);

void setCRCLength(rf24_crclength_e length);


rf24_crclength_e getCRCLength(void);

void disableCRC(void);


void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);

void startConstCarrier(rf24_pa_dbm_e level, uint8_t channel);

void stopConstCarrier(void);




bool isAckPayloadAvailable(void);

void _init_obj();

bool _init_radio();


bool _init_pins();


void csn(bool mode);


void ce(bool level);


void read_registerN(uint8_t reg, uint8_t* buf, uint8_t len);


uint8_t read_register(uint8_t reg);


void write_register_1(uint8_t reg, const uint8_t* buf, uint8_t len);

void write_register_2(uint8_t reg, uint8_t value);

void write_register(uint8_t reg, uint8_t value, bool is_cmd_only);


void write_payload(const void* buf, uint8_t len, const uint8_t writeType);


void read_payload(void* buf, uint8_t len);


uint8_t get_status(void);



void print_status(uint8_t status);


void print_observe_tx(uint8_t value);


void print_byte_register(const char* name, uint8_t reg, uint8_t qty);


void print_address_register(const char* name, uint8_t reg, uint8_t qty);


void toggle_features(void);



#endif /* API_NRF24_RADIO_LAYER2_ARDUINO_NRF24_ARDUINO_H_ */

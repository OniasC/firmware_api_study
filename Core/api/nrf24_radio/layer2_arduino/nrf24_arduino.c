/*
 * nrf24_arduino.c
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "nrf24_arduino_config.h"
#include "nrf24_arduino.h"

RF24_t radioThis = {0};
/****************************************************************************/

void  csn(bool mode)
{
    digitalWrite(radioThis.csn_pin, mode);
    delayMicroseconds(radioThis.csDelay);
}

/****************************************************************************/

void  ce(bool level)
{
	digitalWrite(radioThis.ce_pin, level);
}

/****************************************************************************/

inline void  beginTransaction()
{

    csn(API_LOW);
}

/****************************************************************************/

inline void  endTransaction()
{
    csn(API_HIGH);
}

/****************************************************************************/

void  read_registerN(uint8_t reg, uint8_t* buf, uint8_t len)
{


    beginTransaction();

    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi, R_REGISTER | reg);
    while (len--) { *buf++ = spiTransmitReceiveOnce(radioThis.chip_spi,0xFF); }


    endTransaction();

}

/****************************************************************************/

uint8_t  read_register(uint8_t reg)
{
    uint8_t result;

    beginTransaction();
    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi, (R_REGISTER | reg));

    result = spiTransmitReceiveOnce(radioThis.chip_spi,0xff);
    endTransaction();

    return result;
}

/****************************************************************************/

void  write_register_1(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    beginTransaction();

    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,W_REGISTER | reg);
    while (len--) { spiTransmitReceiveOnce(radioThis.chip_spi,*buf++); }

    endTransaction();

}

/****************************************************************************/
void  write_register_2(uint8_t reg, uint8_t value)
{
	////IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"), reg, value));
	beginTransaction();
	radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,W_REGISTER | reg);
	spiTransmitReceiveOnce(radioThis.chip_spi,value);
	endTransaction();
}


void  write_register(uint8_t reg, uint8_t value, bool is_cmd_only)
{
    if (is_cmd_only) {
        if (reg != RF24_NOP) { // don't print the get_status() operation
//            //IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x)\r\n"), reg));
        }
        beginTransaction();
        radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,W_REGISTER | reg);
        endTransaction();
    }
    else
    {
        ////IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"), reg, value));
        beginTransaction();
        radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,W_REGISTER | reg);
        spiTransmitReceiveOnce(radioThis.chip_spi,value);
        endTransaction();
    }
}

/****************************************************************************/

void  write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
    const uint8_t* current = (uint8_t*)(buf);

    uint8_t blank_len = !data_len ? 1 : 0;
    if (!radioThis.dynamic_payloads_enabled) {
        data_len = rf24_min(data_len, radioThis.payload_size);
        blank_len = radioThis.payload_size - data_len;
    }
    else {
        data_len = rf24_min(data_len, 32);
    }

    //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
    ////IF_SERIAL_DEBUG(printf("[Writing %u bytes %u blanks]\n", data_len, blank_len); );

    beginTransaction();

    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,writeType);
    while (data_len--) { spiTransmitReceiveOnce(radioThis.chip_spi,*current++); }
    while (blank_len--) { spiTransmitReceiveOnce(radioThis.chip_spi,0); }

    endTransaction();

}

/****************************************************************************/

void  read_payload(void* buf, uint8_t data_len)
{
    uint8_t* current = (uint8_t*)(buf);

    uint8_t blank_len = 0;
    if (!radioThis.dynamic_payloads_enabled) {
        data_len = rf24_min(data_len, radioThis.payload_size);
        blank_len = radioThis.payload_size - data_len;
    }
    else {
        data_len = rf24_min(data_len, 32);
    }

    //printf("[Reading %u bytes %u blanks]",data_len,blank_len);

    ////IF_SERIAL_DEBUG(printf("[Reading %u bytes %u blanks]\n", data_len, blank_len); );



    beginTransaction();

    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,R_RX_PAYLOAD);
    while (data_len--) { *current++ = spiTransmitReceiveOnce(radioThis.chip_spi,0xFF); }
    while (blank_len--) { spiTransmitReceiveOnce(radioThis.chip_spi,0xff); }


    endTransaction();

}

/****************************************************************************/

uint8_t  flush_rx(void)
{
    write_register(FLUSH_RX, RF24_NOP, true);
    return radioThis.status;
}

/****************************************************************************/

uint8_t  flush_tx(void)
{
    write_register(FLUSH_TX, RF24_NOP, true);
    return radioThis.status;
}

/****************************************************************************/

uint8_t  get_status(void)
{
    write_register(RF24_NOP, RF24_NOP, true);
    return radioThis.status;
}

/****************************************************************************/
#if !defined(MINIMAL)

void  print_status(uint8_t _status)
{
    //printf_P(PSTR("radioThis.status\t\t= 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"), _status, (_status & _BV(RX_DR)) ? 1 : 0,
    //        (_status & _BV(TX_DS)) ? 1 : 0, (_status & _BV(MAX_RT)) ? 1 : 0, ((_status >> RX_P_NO) & 0x07), (_status & _BV(TX_FULL)) ? 1 : 0);
}

/****************************************************************************/


void  print_observe_tx(uint8_t value)
{
    //printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"), value, (value >> PLOS_CNT) & 0x0F, (value >> ARC_CNT) & 0x0F);
}

/****************************************************************************/

void  print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{

}

/****************************************************************************/

void  print_address_register(const char* name, uint8_t reg, uint8_t qty)
{

}

#endif // !defined(MINIMAL)

/****************************************************************************/

void RF24(io_pin_t _cepin, io_pin_t _cspin, uint32_t _spi_speed)
{
    _init_obj();
}

/****************************************************************************/

/****************************************************************************/

void  _init_obj()
{
    // Use a pointer on the Arduino platform

    radioThis.pipe0_reading_address[0] = 0;
    if(radioThis.spi_speed <= 35000){ //Handle old BCM2835 speed constants, default to RF24_SPI_SPEED
        radioThis.spi_speed = RF24_SPI_SPEED;
    }
}

/****************************************************************************/

void  setChannel(uint8_t channel)
{
    const uint8_t max_channel = 125;
    write_register_2(RF_CH, rf24_min(channel, max_channel));
}

uint8_t  getChannel()
{

    return read_register(RF_CH);
}

/****************************************************************************/

void  setPayloadSize(uint8_t size)
{
    // payload size must be in range [1, 32]
	radioThis.payload_size = rf24_max(1, rf24_min(32, size));

    // write static payload size setting for all pipes
    for (uint8_t i = 0; i < 6; ++i)
        write_register_2(RX_PW_P0 + i, radioThis.payload_size);
}

/****************************************************************************/

uint8_t  getPayloadSize(void)
{
    return radioThis.payload_size;
}

/****************************************************************************/

bool  begin(spi_t _spi, io_pin_t _cepin, io_pin_t _cspin)
{
    radioThis.ce_pin = _cepin;
    radioThis.csn_pin = _cspin;
    radioThis.chip_spi = &_spi;
    return _init_pins() && _init_radio();
}

/****************************************************************************/

/****************************************************************************/

bool  _init_pins()
{
    if (!isValid()) {
        // didn't specify the CSN & CE pins to c'tor nor begin()
        return false;
    }


    ce(API_LOW);
    csn(API_HIGH);

    return true; // assuming pins are connected properly
}

/****************************************************************************/

bool  _init_radio()
{
    // Must allow the radioThis time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay(5);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See datasheet for a more complete explanation.
    setRetries(5, 15);

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    setDataRate(RF24_1MBPS);

    // detect if is a plus variant & use old toggle features command accordingly
    uint8_t before_toggle = read_register(FEATURE);
    toggle_features();
    uint8_t after_toggle = read_register(FEATURE);
    radioThis._is_p_variant= before_toggle == after_toggle;
    if (after_toggle){
        if (radioThis._is_p_variant){
            // module did not experience power-on-reset (#401)
            toggle_features();
        }
        // allow use of multicast parameter and dynamic payloads by default
        write_register_2(FEATURE, 0);
    }
    radioThis.ack_payloads_enabled = false;     // ack payloads disabled by default
    write_register_2(DYNPD, 0);         // disable dynamic payloads by default (for all pipes)
    radioThis.dynamic_payloads_enabled = false;
    write_register_2(EN_AA, 0x3F);      // enable auto-ack on all pipes
    write_register_2(EN_RXADDR, 3);     // only open RX pipes 0 & 1
    setPayloadSize(32);               // set static payload size to 32 (max) bytes by default
    setAddressWidth(5);               // set default address length to (max) 5 bytes

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(76);

    // Reset current radioThis.status
    // Notice reset and flush is the last thing we do
    write_register_2(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));


    // Flush buffers
    flush_rx();
    flush_tx();

    // Clear CONFIG register:
    //      Reflect all IRQ events on IRQ pin
    //      Enable PTX
    //      Power Up
    //      16-bit CRC (CRC required by auto-ack)
    // Do not write CE API_HIGH so radioThis will remain in standby I mode
    // PTX should use only 22uA of power
    write_register_2(NRF_CONFIG, (_BV(EN_CRC) | _BV(CRCO)) );
    radioThis.config_reg = read_register(NRF_CONFIG);

    powerUp();

    // if config is not set correctly then there was a bad response from module
    return radioThis.config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)) ? true : false;
}

/****************************************************************************/

bool  isChipConnected()
{
    uint8_t setup = read_register(SETUP_AW);
    if (setup >= 1 && setup <= 3) {
        return true;
    }

    return false;
}

/****************************************************************************/

bool  isValid()
{
    //return radioThis.ce_pin != 0xFFFF && radioThis.csn_pin != 0xFFFF;
    return true;
}

/****************************************************************************/

void  startListening(void)
{
    #if !defined(RF24_TINY) && !defined(LITTLEWIRE)
    powerUp();
    #endif
    radioThis.config_reg |= _BV(PRIM_RX);
    write_register_2(NRF_CONFIG, radioThis.config_reg);
    write_register_2(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    ce(API_HIGH);

    // Restore the pipe0 address, if exists
    if (radioThis.pipe0_reading_address[0] > 0) {
        write_register_1(RX_ADDR_P0, radioThis.pipe0_reading_address, radioThis.addr_width);
    } else {
        closeReadingPipe(0);
    }
}

/****************************************************************************/


void  stopListening(void)
{
    ce(API_LOW);

    //delayMicroseconds(100);
    delayMicroseconds(radioThis.txDelay);
    if (radioThis.ack_payloads_enabled){
        flush_tx();
    }

    radioThis.config_reg &= ~_BV(PRIM_RX);
    write_register_2(NRF_CONFIG, radioThis.config_reg);

    write_register_2(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0
}

/****************************************************************************/

void  powerDown(void)
{
    ce(API_LOW); // Guarantee CE is API_LOW on powerDown
    radioThis.config_reg &= ~_BV(PWR_UP);
    write_register_2(NRF_CONFIG, radioThis.config_reg);
}

/****************************************************************************/

//Power up now. radioThis will not power down unless instructed by MCU for config changes etc.
void  powerUp(void)
{
    // if not powered up then power up and wait for the radioThis to initialize
    if (!(radioThis.config_reg & _BV(PWR_UP))) {
        radioThis.config_reg |= _BV(PWR_UP);
        write_register_2(NRF_CONFIG, radioThis.config_reg);

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set API_HIGH. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        delay(RF24_POWERUP_DELAY/1000);
        //delayMicroseconds(RF24_POWERUP_DELAY);
    }
}

/******************************************************************/

/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool  writeMulticast(const void* buf, uint8_t len, const bool multicast)
{
    //Start Writing
    startFastWrite(buf, len, multicast, 1);

    //Wait until complete or failed

    while (!(get_status() & (_BV(TX_DS) | _BV(MAX_RT)))) {
    }

    ce(API_LOW);

    write_register_2(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    //Max retries exceeded
    if (radioThis.status & _BV(MAX_RT)) {
        flush_tx(); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
        return 0;
    }
    //TX OK 1 or 0
    return 1;
}

bool  write(const void* buf, uint8_t len)
{
    return writeMulticast(buf, len, 0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool  writeBlocking(const void* buf, uint8_t len, uint32_t timeout)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radioThis will auto-clear everything in the FIFO as long as CE remains API_HIGH

    uint32_t timer = millis();                               // Get the time that the payload transmission started

    while ((get_status() & (_BV(TX_FULL)))) {                // Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

        if (radioThis.status & _BV(MAX_RT)) {                          // If MAX Retries have been reached
            reUseTX();                                       // Set re-transmit and clear the MAX_RT interrupt flag
            if (millis() - timer > timeout) {
                return 0;                                    // If this payload has exceeded the user-defined timeout, exit and return 0
            }
        }

    }

    //Start Writing
    startFastWrite(buf, len, 0, 1);                             // Write the payload if a buffer is clear

    return 1;                                                // Return 1 to indicate successful transmission
}

/****************************************************************************/

void  reUseTX()
{
    write_register_2(NRF_STATUS, _BV(MAX_RT));  //Clear max retry flag
    write_register(REUSE_TX_PL, RF24_NOP, true);
    ce(API_LOW);                                  //Re-Transfer packet
    ce(API_HIGH);
}

/****************************************************************************/

bool  writeFastMulticast(const void* buf, uint8_t len, const bool multicast)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radioThis will auto-clear everything in the FIFO as long as CE remains API_HIGH

    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((get_status() & (_BV(TX_FULL)))) {
        if (radioThis.status & _BV(MAX_RT)) {
            return 0;                                        //Return 0. The previous payload has not been retransmitted
            // From the user perspective, if you get a 0, just keep trying to send the same payload
        }
    }
    startFastWrite(buf, len, multicast, 1);                     // Start Writing

    return 1;
}

bool  writeFast(const void* buf, uint8_t len)
{
    return writeFastMulticast(buf, len, 0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE API_HIGH
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void  startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx)
{ //TMRh20

    write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    if (startTx) {
        ce(API_HIGH);
    }
}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
bool  startWrite(const void* buf, uint8_t len, const bool multicast)
{

    // Send the payload
    write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    ce(API_HIGH);
    #if !defined(F_CPU) || F_CPU > 20000000
    delayMicroseconds(10);
    #endif
    ce(API_LOW);
    return !(radioThis.status & _BV(TX_FULL));
}

/****************************************************************************/

bool  rxFifoFull()
{
    return read_register(FIFO_STATUS) & _BV(RX_FULL);
}

/****************************************************************************/

bool  txStandBy()
{

    while (!(read_register(FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (radioThis.status & _BV(MAX_RT)) {
            write_register_2(NRF_STATUS, _BV(MAX_RT));
            ce(API_LOW);
            flush_tx();    //Non blocking, flush the data
            return 0;
        }

    }

    ce(API_LOW);               //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

bool  txStandBy_2(uint32_t timeout, bool startTx)
{

    if (startTx) {
        stopListening();
        ce(API_HIGH);
    }
    uint32_t start = millis();

    while (!(read_register(FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (radioThis.status & _BV(MAX_RT)) {
            write_register_2(NRF_STATUS, _BV(MAX_RT));
            ce(API_LOW); // Set re-transmit
            ce(API_HIGH);
            if (millis() - start >= timeout) {
                ce(API_LOW);
                flush_tx();
                return 0;
            }
        }
    }

    ce(API_LOW);  //Set STANDBY-I mode
    return 1;

}

/****************************************************************************/

void  maskIRQ(bool tx, bool fail, bool rx)
{
    /* clear the interrupt flags */
    radioThis.config_reg &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
    /* set the specified interrupt flags */
    radioThis.config_reg |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
    write_register_2(NRF_CONFIG, radioThis.config_reg);
}

/****************************************************************************/

uint8_t  getDynamicPayloadSize(void)
{
    uint8_t result = read_register(R_RX_PL_WID);

    if (result > 32) {
        flush_rx();
        delay(2);
        return 0;
    }
    return result;
}

/****************************************************************************/

bool  available(void)
{
    return available2(NULL);
}

/****************************************************************************/

bool  available2(uint8_t* pipe_num)
{
    // get implied RX FIFO empty flag from radioThis.status byte
    uint8_t pipe = (get_status() >> RX_P_NO) & 0x07;
    if (pipe > 5)
        return 0;

    // If the caller wants the pipe number, include that
    if (pipe_num)
        *pipe_num = pipe;

    return 1;
}

/****************************************************************************/

void  read(void* buf, uint8_t len)
{

    // Fetch the payload
    read_payload(buf, len);

    //Clear the only applicable interrupt flags
    write_register_2(NRF_STATUS, _BV(RX_DR));

}

/****************************************************************************/

void  whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready)
{
    // Read the radioThis.status & reset the radioThis.status in one easy call
    // Or is that such a good idea?
    write_register_2(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Report to the user what happened

    *tx_ok = radioThis.status & _BV(TX_DS);
    *tx_fail = radioThis.status & _BV(MAX_RT);
    *rx_ready = radioThis.status & _BV(RX_DR);
}

/****************************************************************************/

void  openWritingPipe(uint64_t value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    write_register_1(RX_ADDR_P0, (uint8_t*)(&value), radioThis.addr_width);
    write_register_1(TX_ADDR, (uint8_t*)(&value), radioThis.addr_width);
}

/****************************************************************************/
void  openWritingPipePointer(const uint8_t* address)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    write_register_1(RX_ADDR_P0, address, radioThis.addr_width);
    write_register_1(TX_ADDR, address, radioThis.addr_width);
}

/****************************************************************************/

void  openReadingPipe(uint8_t child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(radioThis.pipe0_reading_address, &address, radioThis.addr_width);
    }

    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register_1(pgm_read_byte(&child_pipe[child]), (uint8_t*)(&address), radioThis.addr_width);
        } else {
            write_register_1(pgm_read_byte(&child_pipe[child]), (uint8_t*)(&address), 1);
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register_2(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}

/****************************************************************************/
void  setAddressWidth(uint8_t a_width)
{

    if (a_width -= 2) {
        write_register_2(SETUP_AW, a_width % 4);
        radioThis.addr_width = (a_width % 4) + 2;
    } else {
        write_register_2(SETUP_AW, 0);
        radioThis.addr_width = 2;
    }

}

/****************************************************************************/

void  openReadingPipePointer(uint8_t child, const uint8_t* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(radioThis.pipe0_reading_address, address, radioThis.addr_width);
    }
    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register_1(pgm_read_byte(&child_pipe[child]), address, radioThis.addr_width);
        } else {
            write_register_1(pgm_read_byte(&child_pipe[child]), address, 1);
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register_2(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

    }
}

/****************************************************************************/

void  closeReadingPipe(uint8_t pipe)
{
    write_register_2(EN_RXADDR, read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void  toggle_features(void)
{
    beginTransaction();
    radioThis.status = spiTransmitReceiveOnce(radioThis.chip_spi,ACTIVATE);
    spiTransmitReceiveOnce(radioThis.chip_spi,0x73);
    endTransaction();
}

/****************************************************************************/

void  enableDynamicPayloads(void)
{
    // Enable dynamic payload throughout the system

    //toggle_features();
	write_register_2(FEATURE, read_register(FEATURE) | _BV(EN_DPL));

    ////IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", read_register(FEATURE)));

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register_2(DYNPD, read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

    radioThis.dynamic_payloads_enabled = true;
}

/****************************************************************************/
void  disableDynamicPayloads(void)
{
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads

    //toggle_features();
	write_register_2(FEATURE, 0);

//    //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", read_register(FEATURE)));

    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register_2(DYNPD, 0);

    radioThis.dynamic_payloads_enabled = false;
    radioThis.ack_payloads_enabled = false;
}

/****************************************************************************/

void  enableAckPayload(void)
{
    // enable ack payloads and dynamic payload features

    if (!radioThis.ack_payloads_enabled){
    	write_register_2(FEATURE, read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

//        //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", read_register(FEATURE)));

        // Enable dynamic payload on pipes 0 & 1
        write_register_2(DYNPD, read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
        radioThis.dynamic_payloads_enabled = true;
        radioThis.ack_payloads_enabled = true;
    }
}

/****************************************************************************/

void  disableAckPayload(void)
{
    // disable ack payloads (leave dynamic payload features as is)
    if (radioThis.ack_payloads_enabled){
    	write_register_2(FEATURE, read_register(FEATURE) | ~_BV(EN_ACK_PAY));

//        //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", read_register(FEATURE)));

        radioThis.ack_payloads_enabled = false;
    }
}

/****************************************************************************/

void  enableDynamicAck(void)
{
    //
    // enable dynamic ack features
    //
    //toggle_features();
	write_register_2(FEATURE, read_register(FEATURE) | _BV(EN_DYN_ACK));

    //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n", read_register(FEATURE)));
}

/****************************************************************************/

bool  writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
    if (radioThis.ack_payloads_enabled){
        const uint8_t* current = (uint8_t*)(buf);

        write_payload(current, len, W_ACK_PAYLOAD | (pipe & 0x07));
        return !(radioThis.status & _BV(TX_FULL));
    }
    return 0;
}

/****************************************************************************/

bool  isAckPayloadAvailable(void)
{
    return available2(NULL);
}

/****************************************************************************/

bool  isPVariant(void)
{
    return radioThis._is_p_variant;
}

/****************************************************************************/

void  setAutoAck(bool enable)
{
    if (enable){
    	write_register_2(EN_AA, 0x3F);
    }else{
    	write_register_2(EN_AA, 0);
        // accomodate ACK payloads feature
        if (radioThis.ack_payloads_enabled){
            disableAckPayload();
        }
    }
}

/****************************************************************************/

void  setAutoAck_2(uint8_t pipe, bool enable)
{
    if (pipe < 6) {
        uint8_t en_aa = read_register(EN_AA);
        if (enable) {
            en_aa |= _BV(pipe);
        }else{
            en_aa &= ~_BV(pipe);
            if (radioThis.ack_payloads_enabled && !pipe){
                disableAckPayload();
            }
        }
        write_register_2(EN_AA, en_aa);
    }
}

/****************************************************************************/

bool  testCarrier(void)
{
    return (read_register(CD) & 1);
}

/****************************************************************************/

bool  testRPD(void)
{
    return (read_register(RPD) & 1);
}

/****************************************************************************/

void  setPALevel(uint8_t level, bool lnaEnable)
{

    uint8_t setup = read_register(RF_SETUP) & 0xF8;

    if (level > 3) {                            // If invalid level, go to max PA
        level = (RF24_PA_MAX << 1) + lnaEnable; // +1 to support the SI24R1 chip extra bit
    } else {
        level = (level << 1) + lnaEnable;       // Else set level as requested
    }

    write_register_2(RF_SETUP, setup |= level);   // Write it to the chip
}

/****************************************************************************/

uint8_t  getPALevel(void)
{

    return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

uint8_t  getARC(void)
{

    return read_register(OBSERVE_TX) & 0x0F;
}

/****************************************************************************/

bool  setDataRate(rf24_datarate_e speed)
{
    bool result = false;
    uint8_t setup = read_register(RF_SETUP);

    // API_HIGH and API_LOW '00' is 1Mbs - our default
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    radioThis.txDelay = 280;

    if (speed == RF24_250KBPS) {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        setup |= _BV(RF_DR_LOW);
        radioThis.txDelay = 505;
    } else {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if (speed == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
            radioThis.txDelay = 240;
        }
    }
    write_register_2(RF_SETUP, setup);

    // Verify our result
    if (read_register(RF_SETUP) == setup) {
        result = true;
    }
    return result;
}

/****************************************************************************/

rf24_datarate_e  getDataRate(void)
{
    rf24_datarate_e result;
    uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    } else if (dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    } else {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }
    return result;
}

/****************************************************************************/

void  setCRCLength(rf24_crclength_e length)
{
    radioThis.config_reg &= ~(_BV(CRCO) | _BV(EN_CRC));

    // switch uses RAM (evil!)
    if (length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above.
    } else if (length == RF24_CRC_8) {
        radioThis.config_reg |= _BV(EN_CRC);
    } else {
        radioThis.config_reg |= _BV(EN_CRC);
        radioThis.config_reg |= _BV(CRCO);
    }
    write_register_2(NRF_CONFIG, radioThis.config_reg);
}

/****************************************************************************/

rf24_crclength_e  getCRCLength(void)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t AA = read_register(EN_AA);
    radioThis.config_reg = read_register(NRF_CONFIG);

    if (radioThis.config_reg & _BV(EN_CRC) || AA) {
        if (radioThis.config_reg & _BV(CRCO)) {
            result = RF24_CRC_16;
        } else {
            result = RF24_CRC_8;
        }
    }

    return result;
}

/****************************************************************************/

void  disableCRC(void)
{
	radioThis.config_reg &= ~_BV(EN_CRC);
    write_register_2(NRF_CONFIG, radioThis.config_reg);
}

/****************************************************************************/
void  setRetries(uint8_t delay, uint8_t count)
{
    write_register_2(SETUP_RETR, rf24_min(15, delay) << ARD | rf24_min(15, count));
}

/****************************************************************************/
void  startConstCarrier(rf24_pa_dbm_e level, uint8_t channel)
{
    stopListening();
    write_register_2(RF_SETUP, read_register(RF_SETUP) | _BV(CONT_WAVE) | _BV(PLL_LOCK));
    if (isPVariant()){
        setAutoAck(0);
        setRetries(0, 0);
        uint8_t dummy_buf[32];
        for (uint8_t i = 0; i < 32; ++i)
            dummy_buf[i] = 0xFF;

        // use write_register() instead of openWritingPipe() to bypass
        // truncation of the address with the current  addr_width value
        write_register_1(TX_ADDR, (uint8_t*)(&dummy_buf), 5);
        flush_tx();  // so we can write to top level

        // use write_register() instead of write_payload() to bypass
        // truncation of the payload with the current  payload_size value
        write_register_1(W_TX_PAYLOAD, (uint8_t*)(&dummy_buf), 32);

        disableCRC();
    }
    setPALevel(level, 1);
    setChannel(channel);
//    //IF_SERIAL_DEBUG(printf_P(PSTR("RF_SETUP=%02x\r\n"), read_register(RF_SETUP)));
    ce(API_HIGH);
    if (isPVariant()){
        delay(1); // datasheet says 1 ms is ok in this instance
        ce(API_LOW);
        reUseTX();
    }
}

/****************************************************************************/
void  stopConstCarrier()
{
    /*
     * A note from the datasheet:
     * Do not use REUSE_TX_PL together with CONT_WAVE=1. When both these
     * registers are set the chip does not react when setting CE API_LOW. If
     * however, both registers are set PWR_UP = 0 will turn TX mode off.
     */
    powerDown();  // per datasheet recommendation (just to be safe)
    write_register_2(RF_SETUP, read_register(RF_SETUP) & ~_BV(CONT_WAVE) & ~_BV(PLL_LOCK));
    ce(API_LOW);
}

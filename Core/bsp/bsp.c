/*
 * bsp.c
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#include "bsp.h"

tft_display_t tft_display = {0};
nrf24_t radio = {0};
motor_t motor1 = {0};
motor_t motor_enc = {0};
debug_option_e debug_option = 0;
eeprom_at24c_t eeprom = {0};
eeprom_m24c64_t eeprom_bsp = {0};
eeprom_t eeprom2;
imu_mpu6050_t imu1 = {0};
portExpansor_t portExp = {0};
display_7seg_portExp_t disp7SegPExp = {0};

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern USART_HandleTypeDef husart3;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

void BSP_DEBUG_Init(void)
{
	debug_option = DEBUG_PC_UART | DEBUG_LEDS;
}

tft_display_status_e BSP_DISPLAY_TFT_Init(void)
{
	TFT_DISPLAY_ctor(&tft_display, &husart3,
			LCD_RESET_Pin, LCD_RESET_GPIO_Port,
			LCD_CS_Pin, LCD_CS_GPIO_Port,
			&htim3, TIM_CHANNEL_4);
	tft_display.height = ST7735_HEIGHT;
	tft_display.width = ST7735_WIDTH;

	//TODO: call other init necessary functions.
	HAL_GPIO_WritePin(tft_display.chip_select.gpio_port, tft_display.chip_select.gpio_pin, GPIO_PIN_SET);
	//LED K
	HAL_TIM_PWM_Start(tft_display.led_k.htim, tft_display.led_k.Channel);

	ST7735_SetBacklight(&tft_display, 300);

	ST7735_SetBacklight(&tft_display, 30);
	ST7735_Init(&tft_display, TFT_ROTATION_0);
	//TODO: adapt the rest of the code to fit the api style
	tft_display.status = DISPLAY_TFT_NO_ERROR;
	return tft_display.status;
}

imu_status_e BSP_IMU_Init(void)
{
	imu_mpu6050_ctor(&imu1, &hi2c1);
	imu1.imu.status = IMU_NO_ERROR;
	return imu1.imu.status;
}

eeprom_status_e BSP_EEPROM_Init(void)
{
	eeprom_m24c64_ctor(&eeprom_bsp,  EEPROM_M24C64, &hi2c1, 0, EEPROM_WR_CTRL_Pin, EEPROM_WR_CTRL_GPIO_Port);
	return eeprom_bsp.eeprom.status;
}


radio_status_e BSP_Radio_Init_Rx(void)
{

	NRF24_ctor(&radio, &hspi1,
		  NRF24_CE_Pin, NRF24_CE_GPIO_Port,
		  NRF24_CS_Pin, NRF24_CS_GPIO_Port,
		  NRF24_IRQ_Pin, NRF24_IRQ_GPIO_Port, NRF24L01p);
	uint64_t RxpipeAddrs = 0x11223344AA;
	NRF24_begin(&radio);

	//**** TRANSMIT - ACK ****//
	//NRF24_stopListening(&radio);
	//NRF24_openWritingPipe(TxpipeAddrs);

	//NRF24_setAutoAck(&radio, false);

	//IF ACK IS NECESSARY
	NRF24_setAutoAck(&radio, true);
	NRF24_setChannel(&radio, 52);
	NRF24_setPayloadSize(&radio, 32);
	NRF24_openReadingPipe(&radio, 1, RxpipeAddrs);

	//IF ACK MESSAGING IS NECESSARY
	NRF24_enableDynamicPayloads(&radio);
	NRF24_enableAckPayload(&radio);

	NRF24_startListening(&radio);
	radio.status = RADIO_NO_ERROR;
	return radio.status;
}

radio_status_e BSP_Radio_Init_Tx(void)
{
	NRF24_ctor(&radio, &hspi1,
		  NRF24_CE_Pin, NRF24_CE_GPIO_Port,
		  NRF24_CS_Pin, NRF24_CS_GPIO_Port,
		  NRF24_IRQ_Pin, NRF24_IRQ_GPIO_Port, NRF24L01p);
	uint64_t TxpipeAddrs = 0x11223344AA;
	NRF24_begin(&radio);


	//**** TRANSMIT - ACK ****//
	NRF24_stopListening(&radio);
	NRF24_openWritingPipe(&radio, TxpipeAddrs);

	//NRF24_setAutoAck(&radio, false);

	//IF ACK IS NECESSARY
	NRF24_setAutoAck(&radio, true);
	NRF24_setChannel(&radio, 52);
	NRF24_setPayloadSize(&radio, 32);

	//IF ACK MESSAGING IS NECESSARY
	NRF24_enableDynamicPayloads(&radio);
	NRF24_enableAckPayload(&radio);

	radio.status = RADIO_NO_ERROR;
	return radio.status;
}


motor_status_e BSP_Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	return motor_ctor(&motor1, 6.0, MOTOR_UNI_DIRECTIONAL, &htim4, TIM_CHANNEL_1, (uint16_t)0, (GPIO_TypeDef *)0, (uint16_t)0, (GPIO_TypeDef *)0);
}

motor_status_e BSP_Motor_w_Enc_Init(void)
{
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	return motor_ctor(&motor_enc, 6.0, MOTOR_BI_DIRECTIONAL, &htim1, TIM_CHANNEL_4,
						MTR_1A_Pin, MTR_1A_GPIO_Port,
						MTR_1B_Pin, MTR_1B_GPIO_Port);
}

void BSP_Port_Expansor_Init(void)
{
	portExpansor_ctor(&portExp, &hi2c1, 0x0);
	portExpansor_cfgPort(&portExp, port1, pinOutput, 0b11111111); /*all pins in port 1 as output*/
	portExpansor_cfgPort(&portExp, port0, pinInput, 0b11111111); /*all pins in port 0 as input*/
}

display_7seg_status_e BSP_DISPLAY_7SEG_Init(void)
{
	display_7seg_portExp_ctor(&disp7SegPExp, &portExp);
	portExpansor_cfgPort(&portExp, port1, pinOutput, 0b11111111); /*all pins in port 1 as output*/
	disp7SegPExp.pinMap[0].port = port1;
	disp7SegPExp.pinMap[1].port = port1;
	disp7SegPExp.pinMap[2].port = port1;
	disp7SegPExp.pinMap[3].port = port1;
	disp7SegPExp.pinMap[4].port = port1;
	disp7SegPExp.pinMap[5].port = port1;
	disp7SegPExp.pinMap[6].port = port1;
	disp7SegPExp.pinMap[7].port = port1;
	disp7SegPExp.pinMap[8].port = port1;

	/*
	 * pin 0 - middle horizantal
	 * pin 1 - top left straight
	 * pin 2 - top horizontal
	 * pin 3 - top right straight
	 * pin 4 - bottom left straight
	 * pin 5 - bottom horizontal
	 * pin 6 - bottom right straight
	 * pin 7 - bottom right dot
	 *
	 * */

	disp7SegPExp.pinMap[7].pinNumber = 0b1 << 0; //pin p00
	disp7SegPExp.pinMap[0].pinNumber = 0b1 << 1; //pin p01
	disp7SegPExp.pinMap[1].pinNumber = 0b1 << 2; //pin p02
	disp7SegPExp.pinMap[4].pinNumber = 0b1 << 3; //pin p03
	disp7SegPExp.pinMap[5].pinNumber = 0b1 << 4; //pin p04
	disp7SegPExp.pinMap[6].pinNumber = 0b1 << 5; //pin p05
	disp7SegPExp.pinMap[3].pinNumber = 0b1 << 6; //pin p06
	disp7SegPExp.pinMap[2].pinNumber = 0b1 << 7; //pin p07

	disp7SegPExp.super.status = DISPLAY_7SEG_NO_ERROR;
	return disp7SegPExp.super.status;
}

/*
 * bsp.c
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#include "bsp.h"

led_t led = {0};
nrf24_t radio = {0};

motor_t motorBrushes = {0};
motor_t motorWheelRight = {0};
motor_t motorWheelLeft = {0};

debug_option_e debug_option = 0;
//eeprom_m24c64_t eeprom_bsp = {0};
eeprom_at24c_t eeprom = {0};

mcuSpiSubsriber_t mcuSubs = {0};

//imu_mpu6050_t imu1 = {0};
imu_mpu9250_t brd_imu = {0};

portExpansor_t portExp = {0};
display_7seg_portExp_t disp7SegPExp = {0};

robot_t robot = {0};
control_pid_t pidWheel[ROBOT_NUM_WHEELS];
control_pid_t pidRobot[ROBOT_NUM_SPEEDS];

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

void BSP_DEBUG_Init(void)
{
	debug_option = DEBUG_PC_UART | DEBUG_LEDS;
}

imu_status_e BSP_IMU_Init(void)
{
	//imu_mpu6050_ctor(&imu1, &hi2c1);
	//imu1.imu.status = IMU_NO_ERROR;

	io_pin_t pin_cs = {IMU_CS_Pin, IMU_CS_GPIO_Port};
	io_pin_t pin_irq = {IMU_IRQ_Pin, IMU_IRQ_GPIO_Port};

	imu_mpu9250_ctor(&brd_imu, &hspi1, pin_cs, pin_irq);
	imu_mpu9250_Init(&brd_imu);
	brd_imu.imu.status = IMU_NO_ERROR;

	return brd_imu.imu.status;
}

eeprom_status_e BSP_EEPROM_Init(void)
{
	io_pin_t wp_pin = {EEPROM_WR_CTRL_Pin, EEPROM_WR_CTRL_GPIO_Port};
	eeprom_at24c_WP_ctor(&eeprom, EEPROM_AT24C16D, &hi2c1, 0, wp_pin);

	//eeprom_m24c64_ctor(&eeprom_bsp,  EEPROM_M24C64, &hi2c1, 0, EEPROM_WR_CTRL_Pin, EEPROM_WR_CTRL_GPIO_Port);
	return eeprom.super.status;
}

radio_status_e BSP_Radio_Init(uint8_t channel, uint8_t payloadSize, bool enableAutoACK, nrf24_mode_e mode, bool enableACKPayload, uint64_t address)
{
    NRF24_ctor(&radio, &hspi1,
	       NRF24_CE_Pin, NRF24_CE_GPIO_Port,
	       NRF24_CS_Pin, NRF24_CS_GPIO_Port,
	       NRF24_IRQ_Pin, NRF24_IRQ_GPIO_Port, NRF24L01p);

    NRF24_begin(&radio);

    //**** TRANSMIT - ACK ****//
    if (RF24_MODE_TX == mode)
    {
        NRF24_stopListening(&radio);
        NRF24_openWritingPipe(&radio, address);
    }

    //IF ACK IS NECESSARY
    NRF24_setAutoAck(&radio, enableAutoACK);

    NRF24_setChannel(&radio, channel);
    if (payloadSize > 32) payloadSize = 32;
    NRF24_setPayloadSize(&radio, payloadSize);
    if (RF24_MODE_RX == mode)
    {
      NRF24_openReadingPipe(&radio, 1, address);
    }

    //IF ACK MESSAGING IS NECESSARY
    if(true == enableACKPayload)
    {
      NRF24_enableDynamicPayloads(&radio);
      NRF24_enableAckPayload(&radio);
    }

    if (RF24_MODE_RX == mode)
    {
    	NRF24_startListening(&radio);
    }
    radio.status = RADIO_NO_ERROR;
    return radio.status;
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

	NRF24_setAutoAck(&radio, false);

	//IF ACK IS NECESSARY
	//NRF24_setAutoAck(&radio, true);
	NRF24_setChannel(&radio, 10);
	NRF24_setPayloadSize(&radio, 32);
	NRF24_openReadingPipe(&radio, 0, RxpipeAddrs);
	NRF24_openReadingPipe(&radio, 1, RxpipeAddrs+1);
	NRF24_openReadingPipe(&radio, 2, RxpipeAddrs+2);
	NRF24_openReadingPipe(&radio, 3, RxpipeAddrs+3);
	NRF24_openReadingPipe(&radio, 4, RxpipeAddrs+5);
	NRF24_openReadingPipe(&radio, 5, RxpipeAddrs+7);
	//IF ACK MESSAGING IS NECESSARY
	//NRF24_enableDynamicPayloads(&radio);
	//NRF24_enableAckPayload(&radio);

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

	NRF24_setChannel(&radio, 10);
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
	//return motor_ctor(&motorBrushes, 6.0, MOTOR_UNI_DIRECTIONAL, &htim4, TIM_CHANNEL_1, (uint16_t)0, (GPIO_TypeDef *)0, (uint16_t)0, (GPIO_TypeDef *)0, (TIM_HandleTypeDef *)0);
	return motor_ctorSimple(&motorBrushes, 6.0, MOTOR_MODE_UNI_DIR_OPEN_LOOP, &htim4, TIM_CHANNEL_1);

}

motor_status_e BSP_Motor_withEnc_Init(void)
{
	/* motor wheel left */
	io_pin_t motorLeftPinA = {MTR_1A_Pin, MTR_1A_GPIO_Port};
	io_pin_t motorLeftPinB = {MTR_1A_Pin, MTR_1A_GPIO_Port};
	pwm_t pwmLeft = {&htim3, TIM_CHANNEL_1};

	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(pwmLeft.htim, pwmLeft.Channel);
	motor_ctorDriveCoast(&motorWheelLeft, 6.0, MOTOR_MODE_BI_DIR_DRIVECOAST_CLOSED_LOOP,
			             &pwmLeft,
						 &motorLeftPinA,
						 &motorLeftPinB,
						 &htim2);

	/* motor wheel right */
	io_pin_t motorRightPinA = {MTR_2A_Pin, MTR_2A_GPIO_Port};
	io_pin_t motorRightPinB = {MTR_2B_Pin, MTR_2B_GPIO_Port};
	pwm_t pwmRight = {&htim3, TIM_CHANNEL_2};

	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(pwmRight.htim, pwmRight.Channel);
	motor_ctorDriveCoast(&motorWheelRight, 6.0, MOTOR_MODE_BI_DIR_DRIVECOAST_CLOSED_LOOP,
					     &pwmRight,
						 &motorRightPinA,
						 &motorRightPinB,
						 &htim1);

	return (motorWheelLeft.status | motorWheelRight.status);
}

void BSP_Port_Expansor_Init(void)
{
	portExpansor_ctor(&portExp, &hi2c1, 0x0);
	portExpansor_cfgPort(&portExp, port1, pinOutput, 0b11111111); /*all pins in port 1 as output*/
	portExpansor_cfgPort(&portExp, port0, pinInput, 0b11111111); /*all pins in port 0 as input*/
}

led_status_e BSP_LedGPIO_Init(void)
{
	return led_ctorGPIO(&led, LED_MODE_GPIO_RESET, LED_POLARITY_DIRECT, LED0_Pin, LED0_GPIO_Port);
}

led_status_e BSP_LedPWM_Init(void)
{
	return led_ctorGPIO(&led, LED_MODE_GPIO_RESET, LED_POLARITY_DIRECT, LED0_Pin, LED0_GPIO_Port);
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

void BSP_Robot_Init(void)
{
	/*distance from imu to CM*/
	robot.imuDisplacement[0] = 0.123; /*dummy numbers, replace them with correct ones*/
	robot.imuDisplacement[0] = 0.054;
	robot.imuDisplacement[0] = 0.0;
	robot.imuOrientation = PI/2;


	motor_t motors[2] = {motorWheelLeft, motorWheelRight};
	robot.controlSampleTimeMs = 1;
	//random pid parameters
	pidWheel[0].Kp = 500;
	pidWheel[0].Ki = 50;
	pidWheel[0].Kd = 5;
	robot.speedConvMatrixRobot2Wheels[0][0] = 1.0;
	robot.speedConvMatrixRobot2Wheels[0][1] = 1.0;
	robot.speedConvMatrixRobot2Wheels[1][0] = 1.0;
	robot.speedConvMatrixRobot2Wheels[1][1] = -1.0;

	robot.speedConvMatrixWheels2Robot[0][0] = 0.5;
	robot.speedConvMatrixWheels2Robot[0][1] = 0.5;
	robot.speedConvMatrixWheels2Robot[1][0] = 0.5;
	robot.speedConvMatrixWheels2Robot[1][1] = -0.5;

	/*TODO: fix kalman. pointer pointing to the wrong memory address!
	kalman_t *kf = kalman_filter_robot_init();
	kalman_measurement_t *kfmEncoder = kalman_filter_robot_measurement_encoder_init();
	kalman_measurement_t *kfmIMU = kalman_filter_robot_measurement_imu_init();*/
	kalman_t kf = {0};
	kalman_measurement_t kfmEncoder = {0};
	kalman_measurement_t kfmIMU = {0};
	control_kalman_init(&kf, &kfmEncoder, &kfmIMU);

	robot_ctor(&robot, (eeprom_t*)&eeprom, (imu_t *)&brd_imu, motors, pidWheel, pidRobot, &motorBrushes);
	robot.status = robot_status_no_error;
}

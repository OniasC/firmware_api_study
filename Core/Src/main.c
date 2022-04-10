/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../api/imu/TJ_MPU6050.h"
#include "../api/sd_spi/fatfs_sd.h"
#include "../bsp/bsp.h"
#include "../api/nrf24_radio/layer3/RF24Network.h"
#include "../api/nrf24_radio/tinyNetwork/tinyNetwork.h"
#include "string.h"

#include "../app/thirdparties/kalman/kalman_example_gravity.h"
#include "../app/control/kalman_robot.h"
#include "../api/sub_mcu/sub_slave.h"
//#define use_radio  //use radio but not network
//#define use_tiny
//#define use_sd
uint8_t tx = 1;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char myRxData[50];
char myTxData[32] = "Hello World!";
char AckPayload[32];
char myAckPayload[32] = "Ack by STMF1!";
uint64_t pipeAddrs = 0x11223344AA;
//uint64_t pipeAddrs = 0xccccccccc3;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  BSP_DEBUG_Init();
  BSP_LedGPIO_Init();
  BSP_Radio_Init(52, 32, true, RF24_MODE_RX, true, pipeAddrs);
  BSP_IMU_Init();
  BSP_EEPROM_Init();
  BSP_Motor_withEnc_Init();
  BSP_Robot_Init();

  //kalman_gravity_demo();



  mcuSpiSubsriber_t s1;
  io_pin_t cs = {SUBS_CS_Pin, SUBS_CS_GPIO_Port};
  io_pin_t irq = {SUBS_IRQ_Pin, SUBS_IRQ_GPIO_Port};
  slave_ctor(&s1, 2, &hspi1, cs, irq);
  /*
   *
  eeprom_Write((eeprom_t *)&eeprom, 127, 0, (uint8_t *)"onias", sizeof("onias"));
  char test[10];
  memset(&test, 0, sizeof(test));
  eeprom_Read((eeprom_t *)&eeprom, 127, 0, (uint8_t *)&test, sizeof(test));
  *
  */

  #ifdef use_tiny
  int count = 0;

  /*network library stuff*/
  const uint16_t tx_node = 00;       // Address of our node in Octal format
  const uint16_t rx_node = 01;     	 // Address of the other node in Octal format
  struct payload_t {                 // Structure of our payload
    unsigned long ms;
    unsigned long counter;
    char 	  str[6];
  };
  struct packet_t {                 // Structure of our packet
	  tinyNetworkHeader header;
	  struct payload_t payload;
  };
  tinyNetwork_t tinyNetwork = {0};
  tinyNetworkHeader header2;


  tinyNetwork_ctor(&tinyNetwork, &radio);
  tinyNetwork_begin(&tinyNetwork, /*channel*/ 52, /*node address*/ tx_node);

  printRadioSettings(&radio);
  struct payload_t payload_rx;
  unsigned long packets_sent = 0;          // How many have we sent already
#endif //#ifndef use_tiny
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //slave_set_value(&s1, 3.2);
	  //IMU_readAll((imu_t *)&brd_imu);
	  //imu_mpu9250_getData(&brd_imu);
	  //API_DEBUG_MESSAGE_2("%d\r\n", brd_imu.imu.Accel_X_RAW);
	  robot_getStateEstimates(&robot);
	  //API_DEBUG_MESSAGE_2("%d\r\n", robot.imu->Accel_X_RAW);
	  //HAL_Delay(1000);
	  //motor_setSpeedClosedLoop(&motorWheelRight, 6.0);

	  HAL_Delay(1000);
	  motor_setSpeedClosedLoop(&motorWheelRight, -5.0);
#ifdef use_tiny
	  tinyNetwork_update(&tinyNetwork);
	  if(tx)
	  {
		  uint32_t time = HAL_GetTick();
		  //API_DEBUG_MESSAGE_2("trying to send: %ld %ld %s\r\n", time, packets_sent, "hello");

		  header2.from_node = tx_node;
		  header2.to_node = rx_node;
		  header2.id = packets_sent;
		  header2.type = 113;
		  header2.reserved = 97;
		  struct payload_t payload_radio = { time, packets_sent++ , "hello"};

		  struct packet_t tx_packet = {0};
		  tx_packet.header = header2;
		  tx_packet.payload = payload_radio;
		  tinyNetwork_write_test(&tinyNetwork, &header2, &payload_radio, sizeof(payload_radio));
		 /* NRF24_stopListening(&radio);
		  NRF24_openWritingPipe(&radio, pipeAddrs);
		  bool ok2 = tinyNetwork_write_message(&tinyNetwork, &header2, &payload_radio, sizeof(payload_radio), 070);
		  if (ok2 == true)
		  {
			  API_DEBUG_MESSAGE("network successful Successfully\r\n");
		  }
		  if(NRF24_write(&radio, &tx_packet, sizeof(tx_packet)))
		  {
			  NRF24_read(&radio, AckPayload, 32);
			  API_DEBUG_MESSAGE("Transmitted Successfully\r\n");

			  sprintf(DEBUG_STRING, "AckPayload:  %s \r\n", AckPayload);
			  API_DEBUG_MESSAGE(DEBUG_STRING);

		  }
		  NRF24_openReadingPipe(&radio, 1, pipeAddrs);
		  NRF24_startListening(&radio);
*/
		  HAL_Delay(1000);


	  } else {
		  if(NRF24_available(&radio))
			{
			  	struct packet_t rx_packet = {0};
				NRF24_read(&radio, &rx_packet, 32);
				NRF24_writeAckPayload(&radio, 1, myAckPayload, 32);
				myRxData[32] = '\r'; myRxData[32+1] = '\n';

				API_DEBUG_MESSAGE_2("received: %ld %ld %s\r\nheader: %d %d\r\n", rx_packet.payload.ms, rx_packet.payload.counter, rx_packet.payload.str, rx_packet.header.reserved, rx_packet.header.id);
				//sprintf(DEBUG_STRING, "received: %ld %ld %s\r\nheader: %d %d\r\n", rx_packet.payload.ms, rx_packet.payload.counter, rx_packet.payload.str, rx_packet.header.reserved, rx_packet.header.id);
				//API_DEBUG_MESSAGE(DEBUG_STRING);

			}
	  }
#else
#ifdef use_radio
    if(tx)
    {
      uint32_t time = HAL_GetTick();
      char myTxData[32] = "Hello World!";
      API_DEBUG_MESSAGE_2("trying to send: %ld %ld %s\r\n", time, packets_sent, "hello");
      struct payload_t payload_radio = { time, packets_sent++ , "hello"};

	  NRF24_stopListening(&radio);
	  NRF24_openWritingPipe(&radio, pipeAddrs);

      if(NRF24_write(&radio, &myTxData, sizeof(payload_radio)))
      {
	      NRF24_read(&radio, AckPayload, 32);
	      API_DEBUG_MESSAGE_2("Transmitted Successfully\r\n");

	      API_DEBUG_MESSAGE_2("AckPayload:  %s \r\n", AckPayload);
	  }
	  NRF24_openReadingPipe(&radio, 1, pipeAddrs);
	  NRF24_startListening(&radio);
	  HAL_Delay(1000);
    } else {
      if(NRF24_available(&radio))
	    {
    	  	char myRxData[50];
    	  	struct payload_t payload_radio;
		    NRF24_read(&radio, &myRxData, 32);
		    NRF24_writeAckPayload(&radio, 1, myAckPayload, 32);
		    myRxData[32] = '\r'; myRxData[32+1] = '\n';
		    API_DEBUG_MESSAGE_2("%s", myRxData);
		    //API_DEBUG_MESSAGE_2("received: %ld %ld %s\r\n", payload_radio.ms, payload_radio.counter, payload_radio.str);

	    }
    }
#endif //#ifdef use_radio
#endif //#ifndef use_tiny


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 23;
  RCC_OscInitStruct.PLL.PLLN = 354;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

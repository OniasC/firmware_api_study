/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../api/imu/TJ_MPU6050.h"
#include "../api/sd_spi/fatfs_sd.h"
#include "../bsp/bsp.h"
#include "../api/RF24Network/RF24Network.h"
#include "../api/tinyNetwork/tinyNetwork.h"
#include "string.h"
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

uint32_t counter = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	counter = __HAL_TIM_GET_COUNTER(htim);
}

char myRxData[50];
char myTxData[32] = "Hello World!";
char AckPayload[32];
char myAckPayload[32] = "Ack by STMF1!";

/*FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t total, free_space;
#define BUFFER_SIZE 1024
char buffer[BUFFER_SIZE];  // to store strings..
UINT br, bw;*/


int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

/*void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}*/

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
  MX_USART3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t use_tiny = 0;
  uint8_t use_radio = 0; //use radio but not network
  uint8_t tx = 0;
  uint8_t use_network = 0;
  uint8_t use_sd = 0;

  BSP_DEBUG_Init();
  BSP_EEPROM_Init();
  //BSP_IMU_Init();
  if(imu1.imu.status != 0){
	  MPU_ConfigTypeDef myMpuConfig;
  	  MPU6050_Init(&hi2c1);
		//2. Configure Accel and Gyro parameters
		myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
		myMpuConfig.ClockSource = Internal_8MHz;
		myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
		myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
		myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
		MPU6050_Config(&myMpuConfig);
  }
  //BSP_DISPLAY_TFT_Init();
  //BSP_Motor_Init();
  //BSP_Motor_w_Enc_Init();
  BSP_Port_Expansor_Init();
  BSP_DISPLAY_7SEG_Init();

 portExpansor_writePins(&portExp, port1, 0b10110101, 0);/*all pins in port 1 as output HIGH*/
 HAL_Delay(300);
 portExpansor_writePins(&portExp, port1, 0b11111111, 0);/*all pins in port 1 as output HIGH*/
 uint8_t answer;
 portExpansor_readPins(&portExp, port0, &answer);/*all pins in port 1 as output HIGH*/
 if(answer && 0b10){
	 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1U);
 }
 display_7seg_WriteDigit((display_7seg_t*)&disp7SegPExp, disp_7seg_3, dot_off);
 //HAL_Delay(300);
 //display_7seg_WriteDigit((display_7seg_t*)&disp7SegPExp, disp_7Seg_1, dot_off);
 //HAL_Delay(300);
 //display_7seg_WriteDigit((display_7seg_t*)&disp7SegPExp, disp_7Seg_a, dot_off);
 //HAL_Delay(300);
/*
  uint8_t palavra[] = "Onias";
  eeprom_Write((eeprom_t *)&eeprom_bsp, 1, 0, palavra, strlen((char *)palavra));
  HAL_Delay(100);
  eeprom_Read((eeprom_t *)&eeprom_bsp, 1, 0, (uint8_t *)myRxData, 50);
*/


  if ((!use_network) & use_radio)
  {
	  if(tx) BSP_Radio_Init_Tx();
	  else BSP_Radio_Init_Rx();
  }

  /*if(use_sd)
  {
  	HAL_Delay(1000); //a short delay is important to let the SD card settle

	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations
	char myDataack[120];

	API_DEBUG_MESSAGE("Trying to read\r\n");
	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		//myprintf("f_mount error (%i)\r\n", fres);
		sprintf(myDataack, "f_mount error (%i)\r\n", fres);
		API_DEBUG_MESSAGE(myDataack);
		while(1);
	}

	//Let's get some statistics from the SD card
	DWORD free_clusters, free_sectors, total_sectors;

	FATFS* getFreeFs;

	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		sprintf(myDataack, "f_getfree error (%i)\r\n", fres);
		API_DEBUG_MESSAGE(myDataack);
		//myprintf("f_getfree error (%i)\r\n", fres);
		while(1);
	}

	//Formula comes from ChaN's documentation
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;


	//myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
	sprintf(myDataack, "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
	//API_DEBUG_MESSAGE(myDataack, strlen(myDataack));

	//Now let's try to open file "test.txt"
	fres = f_open(&fil, "test.txt", FA_READ);
	if (fres != FR_OK) {
		//myprintf("f_open error (%i)\r\n");
		sprintf(myDataack, "f_open error ()\r\n");
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
		while(1);
	}
	//myprintf("I was able to open 'test.txt' for reading!\r\n");
	sprintf(myDataack, "I was able to open 'test.txt' for reading!\r\n");
	//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));

	//Read 30 bytes from "test.txt" on the SD card
	BYTE readBuf[30];

	//We can either use f_read OR f_gets to get data out of files
	//f_gets is a wrapper on f_read that does some string formatting for us
	TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
	if(rres != 0) {
		//myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
		sprintf(myDataack, "Read string from 'test.txt' contents: %s\r\n", readBuf);
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	} else {
		//myprintf("f_gets error (%i)\r\n", fres);
		sprintf(myDataack, "f_gets error (%i)\r\n", fres);
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	}

	//Be a tidy kiwi - don't forget to close your file!
	f_close(&fil);

	//Now let's try and write a file "write.txt"
	fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if(fres == FR_OK) {
		//myprintf("I was able to open 'write.txt' for writing\r\n");
		sprintf(myDataack, "I was able to open 'write.txt' for writing\r\n");
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	} else {
		//myprintf("f_open error (%i)\r\n", fres);
		sprintf(myDataack, "f_open error (%i)\r\n", fres);
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	}

	//Copy in a string
	strncpy((char*)readBuf, "a new file is made!", 19);
	UINT bytesWrote;
	fres = f_write(&fil, readBuf, 19, &bytesWrote);
	if(fres == FR_OK) {
		//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		sprintf(myDataack, "Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	} else {
		//myprintf("f_write error (%i)\r\n");
		sprintf(myDataack, "f_write error ()\r\n");
		//API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
	}

	//Be a tidy kiwi - don't forget to close your file!
	f_close(&fil);

	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);
  }*/


  //HAL_Delay (500);
  //testAll(&tft_display);
  //fres = f_mount(&fs, "", 0);
  /*if(f_mount(&fs, "", 0) != FR_OK)
    Error_Handler();


  // Check free space /
  if(f_getfree("", &fre_clust, &pfs) != FR_OK)
    Error_Handler();

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

  // Open file to write /
   if(f_open(&fil, "abcdario.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) Error_Handler();
  // Free space is less than 1kb
  if(free_space < 1)
    Error_Handler();

  // Writing text
  f_puts("STM32 SD Card I/O Example via SPI\n", &fil);
  f_puts("Save the world!!!", &fil);

  // Close file
  if(f_close(&fil) != FR_OK)
    Error_Handler();

  // Open file to read
  //if(f_open(&fil, "first.txt", FA_READ) != FR_OK)
  //  Error_Handler();

  while(f_gets(buffer, sizeof(buffer), &fil))
  {
    //printf("%s", buffer);
  }

  // Close file
  if(f_close(&fil) != FR_OK)
    Error_Handler();

  // Unmount SDCARD
  if(f_mount(NULL, "", 1) != FR_OK)
    Error_Handler();*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int count = 0;
  //char snum[5];
  float speed = 0.0;
  float polarity = 1;

  /*network library stuff*/

  RF24Network_t network = {0};
  tinyNetwork_t tinyNetwork;
  struct payload_t {                   // Structure of our payload
    unsigned long ms;
    unsigned long counter;
    char 		  str[6];
  };
  tinyNetwork_ctor(&tinyNetwork, &radio);
  const uint16_t tx_node = 00;       // Address of our node in Octal format
  const uint16_t rx_node = 01;      // Address of the other node in Octal format
  if(use_network){
		NRF24_ctor(&radio, &hspi1,
			  NRF24_CE_Pin, NRF24_CE_GPIO_Port,
			  NRF24_CS_Pin, NRF24_CS_GPIO_Port,
			  NRF24_IRQ_Pin, NRF24_IRQ_GPIO_Port, NRF24L01p);
		RF24Network_ctor(&network, &radio);
		if (tx)
		  RF24Network_begin(&network, /*channel*/ 90, /*node address*/ tx_node);
		else{
		  RF24Network_begin(&network, /*channel*/ 90, /*node address*/ rx_node);
		  //network.frame_size = sizeof(struct payload_t);
		  //fillScreen(&tft_display, WHITE);
		}
		printRadioSettings((network.radio));
  }

  struct payload_t payload_rx;
  RF24NetworkHeader header1;
  char str_debug[256];
  unsigned long packets_sent = 0;          // How many have we sent already
  //HAL_GetTick();
  //printRadioSettings(&radio);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(imu1.imu.status != 0)
	  {
		  MPU6050_Get_Accel_Scale(&myAccelScaled);
		  MPU6050_Get_Gyro_Scale(&myGyroScaled);

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(10);
		  HAL_Delay(50);
		  //IMU_readAccelI2C((imu_t *)&imu1, &hi2c1);
		  //char str_debug[256];
		  //sprintf(str_debug, "raw_x: %hu\r\naccel_y: %hu\r\naccel_z: %hu\r\n\r\n", imu1.imu.Accel_X_RAW, imu1.imu.Accel_Y_RAW, imu1.imu.Accel_Z_RAW);
		  //API_DEBUG_MESSAGE(str_debug);
	  }
	  if(use_tiny)
	  {
		  HAL_Delay(1000);
		  struct payload_t payload2 = { HAL_GetTick(), packets_sent++ , "hello"};
		  tinyNetworkHeader header2;
		  header2.from_node = tx_node;
		  header2.to_node = rx_node;
		  bool ok = tinyNetwork_write_message(&tinyNetwork, &header2, &payload2, sizeof(payload2), 070);
		  if (ok)
		  {
			HAL_Delay(100);
			//ST7735_WriteString(&tft_display, 0, 0, itoa(count, snum, 10), Font_16x26, BLUE, GREEN);
			//fillScreen(&tft_display, WHITE);
			HAL_Delay(100);
			count++;
		  }
	  }//
	  if(use_network){
		  ///***** CODIGO TX
		  RF24Network_update(&network);
		  if(tx)
		  {
			  HAL_Delay(1000);
			  uint32_t time = HAL_GetTick();
			  sprintf(DEBUG_STRING, "trying to send: %ld %ld %s\r\n", time, packets_sent, "hello");
			  struct payload_t payload = { time, packets_sent++ , "hello"};
			  //RF24NetworkHeader header1;
			  header1.from_node = tx_node;
			  header1.to_node = rx_node;
			  //uint8_t array[] = "invalid node address\r\n";
			  //API_Debug_Messages(array, sizeof(array));
			  //printRadioSettings((network.radio));
			  API_DEBUG_MESSAGE(DEBUG_STRING);

			  bool ok = RF24Network_write_message(&network, &header1, &payload, sizeof(payload), 070);
			  if (ok)
			  {
				HAL_Delay(100);
				//fillScreen(&tft_display, WHITE);
				//ST7735_WriteString(&tft_display, 0, 0, itoa(count, snum, 10), Font_16x26, BLUE, GREEN);
				count++;
			  }//*/
		  }
		  else{
			 ///*CODIGO RX
			  //RF24Network_update(&network);
			  while(RF24Network_available(&network))
			  {
				  RF24Network_read(&network, &header1, &payload_rx, sizeof(payload_rx));
				  sprintf(DEBUG_STRING, "received: %ld %ld %s\r\n", payload_rx.ms, payload_rx.counter, payload_rx.str);
				  API_DEBUG_MESSAGE(DEBUG_STRING);
				  //API_Debug_Messages(payload.str, sizeof(payload.str));
				  //fillScreen(&tft_display, WHITE);
				  //ST7735_WriteString(&tft_display, 0, 0, itoa(payload.counter, snum, 10), Font_16x26, BLUE, GREEN);
				  //fillScreen(&tft_display, WHITE);
			  }//*/
		  }
	  }
	  else if(use_radio){
		  if(tx){

			  uint32_t time = HAL_GetTick();
			  sprintf(str_debug, "trying to send: %ld %ld %s\r\n", time, packets_sent, "hello");
			  API_DEBUG_MESSAGE(str_debug);
			  struct payload_t payload_radio = { time, packets_sent++ , "hello"};
			  if(NRF24_write(&radio, &payload_radio, sizeof(payload_radio)))
			  {
				  NRF24_read(&radio, AckPayload, 32);
				  API_DEBUG_MESSAGE("Transmitted Successfully\r\n");
				  //API_Debug_Messages((uint8_t *)"Transmitted Successfully\r\n", strlen("Transmitted Successfully\r\n"));

				  char myDataack[80];
				  sprintf(myDataack, "AckPayload:  %s \r\n", AckPayload);
				  API_DEBUG_MESSAGE(myDataack);
				  //API_Debug_Messages((uint8_t *)myDataack, strlen(myDataack));
				}
				HAL_Delay(1000);
		  }
		  else{
			  if(NRF24_available(&radio))
				{
					NRF24_read(&radio, &payload_rx, 32);
					NRF24_writeAckPayload(&radio, 1, myAckPayload, 32);
					myRxData[32] = '\r'; myRxData[32+1] = '\n';
					//fillScreen(&tft_display, WHITE);
					//ST7735_WriteString(&tft_display, 0, 0, myRxData, Font_16x26, BLUE, GREEN);
					sprintf(str_debug, "receivded: %ld %ld %s\r\n", payload_rx.ms, payload_rx.counter, payload_rx.str);
					API_DEBUG_MESSAGE(str_debug);
					//API_Debug_Messages((uint8_t *)myRxData, 32+2);
				}
		  }
	  }



	  if (motor1.status != 0)
	  {
		  if(speed > motor1.max_speed) polarity = -1;
		  else if (speed < 0) polarity = 1;
		  speed = speed + polarity*0.1;
		  motor_speed(&motor1, 5);
	  }
	  if (motor_enc.status != 0)
	  {
		  motor_speed_enc(&motor_enc, -5.0);
		  HAL_Delay(1000);
		  motor_speed_enc(&motor_enc, 5.0);
		  HAL_Delay(1000);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

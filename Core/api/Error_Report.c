/*
 * Error_Report.c
 *
 *  Created on: Oct 9, 2021
 *      Author: onias
 */

#include "api.h"
#include "Error_Report.h"

/*
 * initialize a variable that would tell, from the stuff that was innitialized what are the error report methods
 * availables: leds, 7seg_display, uart, lecd screen, etc
 *
 * The error_report functions and similars would call a function that would check that variable.
 *
 * */

void API_Debug_Messages(const char *error_message, uint16_t len)
{
	// search for the end of string and
	if (sizeof(DEBUG_STRING) == len)
	{
		for (int i = 0; i < DEBUG_STRING_BUFFER_SIZE; i++)
		{
			if(error_message[i] == '\0')
			{
				len = i;
				break;
			}
		}
	}

	if ((debug_option & DEBUG_PC_UART) == 1)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)error_message, len, HAL_MAX_DELAY);
		return;
	}
}


void API_Error_Report(error_e error, error_level_e level, const char *error_message, uint16_t len)
{
	if ((debug_option & DEBUG_PC_UART) == 1)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)error_message, len, HAL_MAX_DELAY);
		//return;
	}
	if (error == NO_ERROR && level == ERR_LVL_WARNING)
	{

	}
	if(error != NO_ERROR)
	{

		while (1) {}
	}
}

void API_Error_Log(error_e *error, bsp_status_t status, error_level_e level)
{

	if(error != NO_ERROR)
	{
		while (1) {}
	}
}

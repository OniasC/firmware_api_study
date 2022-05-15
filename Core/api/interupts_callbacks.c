/*
 * interupts_callbacks.c
 *
 *  Created on: Apr 25, 2022
 *      Author: onias
 */

#include "main.h"
#include "../api/api.h"
#include "../bsp/bsp.h"
#include "../api/switch/switch.h"

/*
 * This function is application dependent!!!
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	static const uint16_t lookup[] = {16, 1, 11, 2, 14, 12, 3, 6, 15, 10, 13, 5, 9, 4, 8, 7};
	uint16_t bitSet = lookup[(((uint16_t) GPIO_Pin*0x6F28)>>14)&15] - 1;
	if(&irqMap[bitSet].irq != NULL && irqMap[bitSet].irqPin != NULL)
	{
		irqMap[bitSet].irq(irqMap[bitSet].irqPin);
	}
	*/
	for(uint8_t i = 0U; i< IRQ_NUM_USED ; i++)
	{
		if (irqMap[i].irqPin->gpio_pin == GPIO_Pin)
		{
			switch (irqMap[i].irqMode){
				case IRQ_MODE_FALLING:
					if(gpio_read(irqMap[i].irqPin) == 0U)
					{
						irqMap[i].irq(irqMap[i].irqPin);
					}
					break;
				case IRQ_MODE_RISING:
					if(gpio_read(irqMap[i].irqPin) == 1U)
					{
						irqMap[i].irq(irqMap[i].irqPin);
					}
					break;
				case IRQ_MODE_RISING_FALLING:
					if(gpio_read(irqMap[i].irqPin) == 0U)
					{
						irqMap[i].irq(irqMap[i].irqPin);
					}
					break;
				default:
					break;
			}
			irqMap[i].value = gpio_read(irqMap[i].irqPin);
			break;
		}
	}
}

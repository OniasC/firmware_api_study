/*
 * switch.c
 *
 *  Created on: Apr 18, 2022
 *      Author: onias
 */


#include "switch.h"

void switch_irq_callback(switch_t *const sw)
{
	sw->value = gpio_read(&(sw->ioPin));

};

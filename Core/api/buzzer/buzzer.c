/*
 * buzzer.c
 *
 *  Created on: Sep 18, 2021
 *      Author: onias
 */

#include "buzzer.h"

buzzer_status_e buzzer_ctor(buzzer_t * const buzzer, TIM_HandleTypeDef *htim, uint32_t Channel)
{
	buzzer->timer_config.htim = htim;
	buzzer->timer_config.Channel = Channel;
	HAL_TIM_PWM_Start((buzzer->timer_config.htim), buzzer->timer_config.Channel);
	return BUZZER_NO_ERROR;
}

void BUZZER_play(buzzer_t * const buzzer)
{
	uint8_t value = 0; // the value for the duty cycle
	while (value<155)
	{
		BUZZER_SetVolume(buzzer, value);
	  value += 40; // increase the duty cycle by 20
	  HAL_Delay (500); // wait for 500 ms
	}
	value = 0; // reset the value
	BUZZER_SetVolume(buzzer, value);
}

void BUZZER_SetVolume(buzzer_t * const buzzer, uint32_t volume)
{
	if (buzzer->timer_config.Channel == TIM_CHANNEL_1)
		buzzer->timer_config.htim->Instance->CCR1 = volume; // vary the duty cycle
	else if (buzzer->timer_config.Channel == TIM_CHANNEL_2)
		buzzer->timer_config.htim->Instance->CCR2 = volume; // vary the duty cycle
	else if (buzzer->timer_config.Channel == TIM_CHANNEL_3)
		buzzer->timer_config.htim->Instance->CCR3 = volume; // vary the duty cycle
	else if (buzzer->timer_config.Channel == TIM_CHANNEL_4)
		buzzer->timer_config.htim->Instance->CCR4 = volume; // vary the duty cycle
}

void BUZZER_tone(buzzer_t * const buzzer, uint32_t frequency, uint32_t duration)
{
	// meu clock eh 72MHz. Freq = 72MHz/(PRESCALER*ARR)
	// meu ARR = 255, entao Freq = 282,352.9/PRESCALER ... PRESCALER = 282,352.9/Freq.
	if (frequency != 0){
		BUZZER_SetVolume(buzzer, 10);
		buzzer->timer_config.htim->Instance->PSC = 282352/frequency/2;
	}
	else
		BUZZER_SetVolume(buzzer, 0);
}

void BUZZER_Play_Pirates(buzzer_t * const buzzer)
{
	int Pirates_note[] = {
	NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4,
	NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4,
	NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4,
	NOTE_A3, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_F4,
	NOTE_F4, NOTE_G4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_D4,
	0, NOTE_A3, NOTE_C4, NOTE_B3, NOTE_D4, NOTE_B3, NOTE_E4, NOTE_F4,
	NOTE_F4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4,
	NOTE_D4, 0, 0, NOTE_A3, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_F4,
	NOTE_G4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_G4,
	NOTE_A4, NOTE_D4, 0, NOTE_D4, NOTE_E3, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_A4,
	NOTE_D4, 0, NOTE_D4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_F4, NOTE_D4
	};
	int Pirates_duration[] = {
	4,8,4,8,4,8,8,8,8,4,8,4,8,4,8,8,8,8,4,8,4,8,
	4,8,8,8,8,4,4,8,8,4,4,8,8,4,4,8,8,
	8,4,8,8,8,4,4,8,8,4,4,8,8,4,4,8,4,
	4,8,8,8,8,4,4,8,8,4,4,8,8,4,4,8,8,
	8,4,8,8,8,4,4,4,8,4,8,8,8,4,4,8,8
	};

  for (int thisNote = 0; thisNote < (sizeof(Pirates_note)/sizeof(int)); thisNote++) {

    int noteDuration = 1000 / Pirates_duration[thisNote];//convert duration to time delay
    BUZZER_tone(buzzer, Pirates_note[thisNote], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.05; //Here 1.05 is tempo, increase to play it slower
    HAL_Delay(pauseBetweenNotes);
    BUZZER_SetVolume(buzzer, 0);//noTone(8); //stop music on pin 8
    }
}

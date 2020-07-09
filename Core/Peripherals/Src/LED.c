/*
 * LED.c
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */

#include "main.h"

void led_init(void)
{
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //Red LED
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Green LED
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //Blue LED
}

void led_stop(void)
{
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); //Red LED
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); //Green LED
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); //Blue LED
}

// Input: 0 to 250
void SetRGB(int Red, int Green, int Blue)
{
	TIM4->CCR2 = Red * 8000 / 250;
	TIM4->CCR1 = Green * 8000 / 250;
	TIM4->CCR4 = Blue * 8000 / 250;
}


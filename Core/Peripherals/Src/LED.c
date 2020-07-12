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
	//  SetRGB(250, 0, 0); // Red
	//  SetRGB(0, 250, 0); // Green
	//  SetRGB(0, 0, 250); // Blue
	//  SetRGB(250, 250, 0);// Yellow
	TIM4->CCR2 = Red * 8000 / 250;
	TIM4->CCR1 = Green * 8000 / 250;
	TIM4->CCR4 = Blue * 8000 / 250;
}


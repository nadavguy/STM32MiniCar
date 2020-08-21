/*
 * PWMControl.c
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */
#include "main.h"

uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t Difference = 0;
uint32_t Frequency = 0;
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured
bool NewDiffAvailable = false;

void start_pwm1(int onTimemSec)
{
    TIM2->CCR1 = onTimemSec * 4 / 10;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void set_pwm1(int onTimemSec)
{
    TIM2->CCR1 = onTimemSec * 4 / 10;
}

void stop_pwm1(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void start_pwm2(int PercentOn)
{
    TIM1->CCR4 = PercentOn;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void set_pwm2(int PercentOn)
{
    TIM1->CCR4 = PercentOn;
}

void stop_pwm2(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}


void start_pwm3(void)
{
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

void stop_pwm3(void)
{
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
}

void start_pwm4(int onTimemSec)
{
    TIM1->CCR3 = onTimemSec * 4 / 10;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void stop_pwm4(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		if (Is_First_Captured==0)  // is the first value captured ?
		{
			IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
			Is_First_Captured =1;  // set the first value captured as true
		}

		else if (Is_First_Captured)  // if the first is captured
		{
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);//HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value

			if (IC_Value2 > IC_Value1)
			{
				Difference = IC_Value1;   // calculate the difference
			}

			else if (IC_Value2 < IC_Value1)
			{
				Difference = IC_Value2;

			}

			else
			{
				Error_Handler();
			}

			Frequency = HAL_RCC_GetSysClockFreq()/(htim->Instance->ARR * (htim->Instance->PSC + 1));  // calculate frequency
			Is_First_Captured = 0;  // reset the first captured

		}
	}
}

/*
 * PWMControl.c
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */
#include "main.h"

void start_pwm1(int onTimemSec)
{
    TIM2->CCR1 = onTimemSec * 4 / 10;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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

void stop_pwm2(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}


void start_pwm3(int onTimemSec)
{
    TIM1->CCR1 = onTimemSec * 4 / 10;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void stop_pwm3(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
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

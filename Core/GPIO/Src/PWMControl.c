/*
 * PWMControl.c
 *
 *  Created on: Jul 9, 2020
 *      Author: raing
 */
#include "main.h"

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

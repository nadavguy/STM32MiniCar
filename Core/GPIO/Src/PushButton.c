/*
 * PushButton.c
 *
 *  Created on: Jul 12, 2020
 *      Author: raing
 */

#include "main.h"
#include "LED.h"
#include "terminal.h"
#include <string.h>
#include <stdio.h>

bool ButtonIsHigh = false;
bool ButtonIsLow = false;
GPIO_PinState PA0PinState;
uint32_t ButtonPressStart = 0;
uint32_t ButtonPressCycleStart = 0;
uint32_t ButtonPressDurationmSec[5] = {0};
uint8_t ButtonCycle = 0;

void CheckButton(void)
{
	PA0PinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if (PA0PinState == GPIO_PIN_RESET)
	{
//		SetRGB(0, 250, 250);
		ButtonIsHigh = false;
		ButtonIsLow = true;
	}
	else
	{
		if (ButtonIsLow)
		{
			if (ButtonCycle < 5)
			{
				ButtonPressDurationmSec[ButtonCycle] = ((HAL_GetTick()
						- ButtonPressStart) / 100) * 100;
				ButtonCycle++;
			}
			if (ButtonCycle >= 5)
			{
				ButtonCycle = 0;
				memset(ButtonPressDurationmSec, 0, 20);
			}
			sprintf(USBTXArray, "%6.3f, Button press duration: %6.3f\r\n",
					CurrentTime(), (HAL_GetTick() - ButtonPressStart) / 1000.0);
			Print(false, true, true);
		}
		if (HAL_GetTick() - ButtonPressCycleStart > 3000)
		{
			if (ButtonPressDurationmSec[0] >= 2000)
			{
				//Do this
				sprintf(USBTXArray, "%6.3f, Do this\r\n", CurrentTime());
				Print(false, true, true);
//
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN5);
//				 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6);
//
//				  /* Clear the related wakeup pin flag */
//				  __HAL_PWR_CLEAR_WAKEUP_FLAG(PWR_WAKEUP_PIN_FLAG1);
//
//				  /* Re-enable all used wakeup sources: Pin6(PI.11) */
//				  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);



				HAL_PWR_EnterSTANDBYMode();
//				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON, PWR_STOPENTRY_WFI);
			}
			else if ((ButtonPressDurationmSec[0] >= 1000)
					&& (ButtonPressDurationmSec[1] >= 1000))
			{
				//Do that
				sprintf(USBTXArray, "%6.3f, Do that\r\n", CurrentTime());
				Print(false, true, true);
			}
			else if ((ButtonPressDurationmSec[0] >= 1000)
					&& (ButtonPressDurationmSec[1] == 0))
			{
				//Do that
				sprintf(USBTXArray, "%6.3f, Do that Single Press\r\n",
						CurrentTime());
				Print(false, true, true);
			}
			else if ((ButtonPressDurationmSec[0] >= 500)
					&& (ButtonPressDurationmSec[1] >= 1000))
			{
				//Or maybe this
				sprintf(USBTXArray, "%6.3f, Or maybe this\r\n", CurrentTime());
				Print(false, true, true);

			}
			else if ((ButtonPressDurationmSec[0] >= 500)
					&& (ButtonPressDurationmSec[1] >= 500))
			{
				//Or maybe that
				sprintf(USBTXArray, "%6.3f, Or maybe that\r\n", CurrentTime());
				Print(false, true, true);
			}
			ButtonCycle = 0;
			memset(ButtonPressDurationmSec, 0, 20);
		}
//		SetRGB(250, 0, 0);
		ButtonIsHigh = true;
		ButtonIsLow = false;
		ButtonPressStart = HAL_GetTick();
		if (ButtonCycle == 0)
		{
			ButtonPressCycleStart = HAL_GetTick();
		}
	}
}

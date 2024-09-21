#include "LED.h"

void Board_LED_ON(void)
{
	HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);
}

void Board_LED_OFF(void)
{
	HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_SET);
}

void Board_LED_Toggle(void)
{
	HAL_GPIO_TogglePin(Board_LED_GPIO_Port, Board_LED_Pin);
}

void Board_LED_task(void *argument)
{
	for (;;)
	{
		Board_LED_Toggle();
        HAL_Delay(500);
	}
}

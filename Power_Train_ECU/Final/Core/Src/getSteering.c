/**
  ******************************************************************************
  * @file           : getSteering.c
  * @brief          : Module That Responsible To Check the steering position through ADC
  ******************************************************************************/
  
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"



extern TIM_HandleTypeDef htim3;
// Global variables to keep track of the encoder state
volatile int32_t position = 0;

void getSteeringPosition(void)
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    while (1)
    {
    	position = (int32_t)(int16_t)TIM3->CNT;
        osDelay(10); // Delay to simulate periodic checking (adjust as necessary)
    }
}

 


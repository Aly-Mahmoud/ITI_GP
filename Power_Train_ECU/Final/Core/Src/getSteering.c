/**
  ******************************************************************************
  * @file           : getSteering.c
  * @brief          : Module That Responsible To Check the steering position through ADC
  ******************************************************************************/
  
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;
extern osSemaphoreId_t Semaphore3Handle;

//extern TIM_HandleTypeDef htim3;
// Global variables to keep track of the encoder state
//volatile int32_t position = 0;

//void getSteeringPosition(void)
//{
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//    while (1)
//    {
//    	if(xSemaphoreTake(Semaphore3Handle, HAL_MAX_DELAY) == pdTRUE)
//    	{
//    		/*To get the position with negative values also*/
//    		position = (int32_t)(int16_t)TIM3->CNT;
//            xSemaphoreGive(Semaphore2Handle);
//            /*
//            xSemaphoreTake(Semaphore1Handle, HAL_MAX_DELAY);
//            xSemaphoreTake(Semaphore3Handle, HAL_MAX_DELAY);*/
//    	}
//    }
//}

 


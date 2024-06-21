/**
  ******************************************************************************
  * @file           : Communication.c
  * @brief          : Module That Responsible To Transfer Data To and From Power
  *                   Train ECU
  ******************************************************************************/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"




#define BUFFER_SIZE 5
#define VALID_HEADER 0x55

typedef struct {
	uint8_t header;
    uint8_t steering;
    uint8_t pedal_gas;
    uint8_t pedal_brake;
    uint8_t buttons;
} steering_frame_t;

volatile steering_frame_t steering_data;
extern UART_HandleTypeDef huart2;
extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;
extern osSemaphoreId_t Semaphore3Handle;





uint8_t uart_rx_buffer[BUFFER_SIZE];

void Parse_Steering_Data(void)
{
    steering_data.header = uart_rx_buffer[0];
    steering_data.steering = uart_rx_buffer[1];
    steering_data.pedal_gas = uart_rx_buffer[2];
    steering_data.pedal_brake = uart_rx_buffer[3];
    //steering_data.clutch = uart_rx_buffer[5];
    steering_data.buttons = uart_rx_buffer[4];
    //steering_data.crc = uart_rx_buffer[6];
}

//void UART_Receive_Header_IT(void)
//{
//    // Start receiving the header byte in interrupt mode
//    HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
//}
//
//void UART_Receive_Data_IT(void)
//{
//    // Start receiving the rest of the frame in interrupt mode
//    HAL_UART_Receive_IT(&huart2, &uart_rx_buffer[1], BUFFER_SIZE - 1);
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        static uint8_t header_received = 0;
//
//        if (!header_received)
//        {
//            if (uart_rx_buffer[0] == VALID_HEADER)
//            {
//                // Valid header received, start receiving the rest of the data
//                header_received = 1;
//                UART_Receive_Data_IT();
//            }
//            else
//            {
//                // Invalid header, reinitialize to receive the header again
//                UART_Receive_Header_IT();
//            }
//        }
//        else
//        {
//            // Data reception complete, process received data
//            Parse_Steering_Data();
//            header_received = 0;
//            UART_Receive_Header_IT();
//        }
//    }
//}
//
//void Communication_Init(void)
//{
//    // Initialize UART reception in interrupt mode
//    UART_Receive_Header_IT();
//}

/*void Send_Steering_Data(void) {
    uint8_t tx_buffer[BUFFER_SIZE];

    tx_buffer[0] = steering_data.header;
    tx_buffer[1] = (steering_data.steering >> 8) & 0xFF;
    tx_buffer[2] = steering_data.steering & 0xFF;
    tx_buffer[3] = steering_data.pedal_gas;
    tx_buffer[4] = steering_data.pedal_brake;
    tx_buffer[5] = steering_data.clutch;
    tx_buffer[6] = steering_data.buttons;
    tx_buffer[7] = steering_data.crc;

    HAL_UART_Transmit(&huart2, tx_buffer, BUFFER_SIZE, HAL_MAX_DELAY);
}*/

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	Parse_Steering_Data();
//	UART_Receive_Data_IT();
//}

void Communication(void)
{
    HAL_StatusTypeDef uart_status;
    uint8_t header = 0; // Initialize header to a known value

    for (;;)
    {

        if (xSemaphoreTake(Semaphore1Handle, HAL_MAX_DELAY) == pdTRUE)
        {
            // Step 1: Receive the header byte
            do
            {
                uart_status = HAL_UART_Receive(&huart2, &header, 1, HAL_MAX_DELAY);
            }
            while (header != VALID_HEADER);

            if (uart_status == HAL_OK && header == VALID_HEADER)
            {
                // Step 2: Receive the remaining bytes if the header is valid
                HAL_UART_Receive(&huart2, uart_rx_buffer+1 , BUFFER_SIZE , HAL_MAX_DELAY);

                // Set the first byte to the header
                 uart_rx_buffer[0] = header;

                 // Process the received data
                 Parse_Steering_Data();
                 //Zero the buffer for the new data
                 /*for(uint8_t i = 0; i < BUFFER_SIZE; i++)
                 {
                    uart_rx_buffer[i] = 0;
                 }*/

            }
            else
            {
                /**/
            }

            // Give the semaphore to signal that the buffer is ready
            xSemaphoreGive(Semaphore2Handle);
        }
    }
}


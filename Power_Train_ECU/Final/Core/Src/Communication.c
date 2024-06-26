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
#include <string.h>




#define HEADER_SIZE 3U
#define VALID_HEADER {0xFF, 0xFF, 0xFF}
#define BUFFER_SIZE	8
#define POLYNOMINAL 	(uint8_t) 0x07
#define INITVALUE	    (uint8_t) 0x00
#define FINAL_XOR_VALUE	(uint8_t) 0x00

typedef struct {
	uint8_t header;
    uint8_t steering;
    uint8_t pedal_gas;
    uint8_t pedal_brake;
    uint8_t buttons;
} steering_frame_t;

volatile steering_frame_t steering_data;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;

uint8_t uart_rx_buffer[BUFFER_SIZE];


void Parse_Steering_Data(void)
{
    steering_data.steering = uart_rx_buffer[3];
    steering_data.pedal_gas = uart_rx_buffer[4];
    steering_data.pedal_brake = uart_rx_buffer[5];
    steering_data.buttons = uart_rx_buffer[6];
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


uint8_t calculate_crc(uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc += data[i];
    }
    return crc;
}

// Function to check the received data with CRC
uint8_t check_crc(uint8_t *data, size_t length) {
    if (length < 2) {
        // Data is too short to contain a CRC
        return 0;
    }

    // Extract the CRC from the last byte
    uint8_t received_crc = data[length - 1];

    // Calculate the CRC over the data (excluding the last byte)
    uint8_t calculated_crc = calculate_crc(data, length - 1);

    // Compare the calculated CRC with the received CRC
    return (calculated_crc == received_crc);
}

void Communication(void)
{
    HAL_StatusTypeDef uart_status;
    uint8_t header[HEADER_SIZE] = {0}; // Initialize header to a known value
    uint8_t expected_header[HEADER_SIZE] = VALID_HEADER;

    for (;;)
    {
		static uint8_t crc_check = 0;
		if (xSemaphoreTake(Semaphore1Handle, HAL_MAX_DELAY) == pdTRUE)
		{
			do
			{
    	        {
    	            // Step 1: Receive the header bytes
    	            do
    	            {
    	            	uart_status = HAL_UART_Receive(&huart6, header, HEADER_SIZE, HAL_MAX_DELAY);
    	                //uart_status = HAL_UART_Receive(&huart2, header, HEADER_SIZE, HAL_MAX_DELAY);
    	            }
    	            while (memcmp(header, expected_header, HEADER_SIZE) != 0);

    		            if (uart_status == HAL_OK && memcmp(header, expected_header, HEADER_SIZE) == 0)
    		            {
    		                // Step 2: Receive the remaining bytes if the header is valid
    		                uart_status = HAL_UART_Receive(&huart6, uart_rx_buffer + HEADER_SIZE, BUFFER_SIZE, HAL_MAX_DELAY);
    		                //uart_status = HAL_UART_Receive(&huart2, uart_rx_buffer + HEADER_SIZE, BUFFER_SIZE, HAL_MAX_DELAY);


    		                if (uart_status == HAL_OK)
    		                {
    		                    // Copy the header to the buffer
    		                    memcpy(uart_rx_buffer, header, HEADER_SIZE);
    		                    crc_check =   check_crc(uart_rx_buffer , BUFFER_SIZE);
    		                    // Process the received data
    		                    Parse_Steering_Data();
    		                }
    		            }

    		        }

    	}while (crc_check == 0);

    	crc_check = 0;
    	xSemaphoreGive(Semaphore2Handle);
    }
    }
}

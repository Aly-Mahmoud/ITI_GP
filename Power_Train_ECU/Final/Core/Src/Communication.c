/**
  ******************************************************************************
  * @file           : Communication.c
  * @brief          : Module That Responsible To Transfer Data To and From Power
  *                   Train ECU
  ******************************************************************************/


/* ============================================================================ */
/*                                  INCLUDEDS                           	    */
/* ============================================================================ */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>



/* ============================================================================ */
/*                                   MACROS                             	    */
/* ============================================================================ */
#define HEADER_SIZE 				3U
#define VALID_HEADER 		{0xFF, 0xFF, 0xFF}
#define BUFFER_SIZE					8



/* ============================================================================ */
/*                                  	TYPES                           	    */
/* ============================================================================ */
typedef struct {
	uint8_t header;
    uint8_t steering;
    uint8_t pedal_gas;
    uint8_t pedal_brake;
    uint8_t buttons;
} steering_frame_t;

/* ============================================================================ */
/*                                  	EXTERNS                           	    */
/* ============================================================================ */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;



/* ============================================================================ */
/*                                  GLOBAL VARIABLES                    	    */
/* ============================================================================ */

volatile steering_frame_t steering_data;
uint8_t uart_rx_buffer[BUFFER_SIZE];




/* ============================================================================ */
/*                                  IMPLEMENTATION                           	*/
/* ============================================================================ */


/**
 * @brief Calculates a simple checksum (CRC) for a given array of data bytes.
 *
 * This function computes a checksum by summing up all bytes in the provided
 * data array. It is used to generate a checksum for data transmission or
 * storage integrity checks.
 *
 * @param data Pointer to the array of data bytes.
 * @param length Number of bytes in the data array.
 * @return uint8_t The computed checksum (CRC) value.
 */
uint8_t calculate_crc(uint8_t *data, size_t length) {
    uint8_t crc = 0;  // Initialize CRC accumulator to 0

    // Iterate over each byte in the data array
    for (size_t i = 0; i < length; i++) {
        crc += data[i];  // Add each byte to the CRC accumulator
    }

    return crc;  // Return the computed CRC checksum
}

/**
 * @brief Checks if the received data matches the expected CRC.
 *
 * This function verifies if the received data matches the expected CRC value.
 * It compares the computed CRC over the data (excluding the last byte) with
 * the received CRC, which is assumed to be the last byte in the data array.
 * If the lengths are insufficient for CRC checking, it returns 0 (false).
 *
 * @param data Pointer to the array of data bytes including the CRC byte.
 * @param length Total number of bytes in the data array.
 * @return uint8_t Returns 1 if CRC matches, 0 otherwise.
 */
uint8_t check_crc(uint8_t *data, size_t length) {
    if (length < 2) {
        // Data is too short to contain a CRC
        return 0;
    }

    // Extract the received CRC from the last byte of data
    uint8_t received_crc = data[length - 1];

    // Calculate the CRC over the data (excluding the last byte)
    uint8_t calculated_crc = calculate_crc(data, length - 1);

    // Compare the calculated CRC with the received CRC
    return (calculated_crc == received_crc);
}


/**
 * @brief Manages communication via UART to receive and process data packets.
 *
 * This function continuously listens for incoming UART data packets. It uses a semaphore (`Semaphore1Handle`)
 * to synchronize access to the UART interface. Upon receiving the semaphore, it:
 * 1. Receives a header from UART and validates it against `expected_header`.
 * 2. If the header matches, it proceeds to receive the remaining data into `uart_rx_buffer`.
 * 3. Checks the CRC of the received data using `check_crc`.
 * 4. If CRC check passes, it copies the header to `uart_rx_buffer` and releases `Semaphore2Handle`.
 *
 * This function operates in an infinite loop, ensuring continuous communication handling.
 */
void Communication(void)
{
    HAL_StatusTypeDef uart_status;
    uint8_t header[HEADER_SIZE] = {0}; // Initialize header to a known value
    uint8_t expected_header[HEADER_SIZE] = VALID_HEADER;
    static uint8_t crc_check = 0;

    for (;;)
    {
        // Wait indefinitely until Semaphore1Handle is available
        if (xSemaphoreTake(Semaphore1Handle, HAL_MAX_DELAY) == pdTRUE)
        {
            do
            {
                // Step 1: Receive the header bytes
                do
                {
                    uart_status = HAL_UART_Receive(&huart6, header, HEADER_SIZE, HAL_MAX_DELAY);
                    //uart_status = HAL_UART_Receive(&huart2, header, HEADER_SIZE, HAL_MAX_DELAY);
                }
                while (memcmp(header, expected_header, HEADER_SIZE) != 0);

                // Step 2: Receive the remaining bytes if the header is valid
                if (uart_status == HAL_OK && memcmp(header, expected_header, HEADER_SIZE) == 0)
                {
                    uart_status = HAL_UART_Receive(&huart6, uart_rx_buffer + HEADER_SIZE, BUFFER_SIZE, HAL_MAX_DELAY);
                    //uart_status = HAL_UART_Receive(&huart2, uart_rx_buffer + HEADER_SIZE, BUFFER_SIZE, HAL_MAX_DELAY);

                    if (uart_status == HAL_OK)
                    {
                        // Copy the header to the buffer
                        memcpy(uart_rx_buffer, header, HEADER_SIZE);
                        // Perform CRC check on the received data
                        crc_check = check_crc(uart_rx_buffer, BUFFER_SIZE);
                        // Process the received data (to be implemented based on application)
                        //Parse_Steering_Data();
                    }
                }

            } while (crc_check == 0); // Repeat until CRC check passes

            // Reset CRC check flag
            crc_check = 0;
            // Release Semaphore2Handle to signal completion of data processing
            xSemaphoreGive(Semaphore2Handle);
        }
    }
}


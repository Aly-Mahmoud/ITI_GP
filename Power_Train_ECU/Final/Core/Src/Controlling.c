/**
  ******************************************************************************
  * @file           : Controlling.c
  * @brief          : Module That Responsible To Control Steering and DC Motor
  *                   for RC Car.
  ******************************************************************************
  */

/* ============================================================================ */
/*                                  INCLUDEDS                           	    */
/* ============================================================================ */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <stdio.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ============================================================================ */
/*                                   MACROS                             	    */
/* ============================================================================ */


// Configure PWM parameters
#define PWM_FREQUENCY   			10000    // PWM frequency in Hz
#define PWM_MAX_DUTY    			100      // Maximum PWM duty cycle (0-100%)
/*PORTB*/
#define BACK_RIGHT_MOTOR_DIR_PIN	GPIO_PIN_14
#define BACK_LEFT_MOTOR_DIR_PIN		GPIO_PIN_15
#define STEERING_MOTOR_DIR_PIN		GPIO_PIN_13
#define BACK_LIGHTS_PIN	            GPIO_PIN_0
#define FRONT_LIGHTS_PIN	        GPIO_PIN_2
#define HORN_PIN	                GPIO_PIN_10
#define BACK_RIGHT_MOTOR			0
#define BACK_LEFT_MOTOR				1
#define STEERING_MOTOR				2
#define DIR_FWD						0
#define DIR_BACK					1
#define STEERING_RIGHT				0
#define STEERING_LEFT				1
#define PEDAL_GAS_MAX				64
#define PEDAL_GAS_MIN				0
#define PEDAL_BRAKE_MAX				64
#define PEDAL_BRAKE_MIN				0
#define MAX_STEERING_VALUE			255
#define MIN_STEERING_VALUE			0
#define STEERING_MIN_MID			124
#define STEERING_MAX_MID			130
#define STEERING_MOTOR_SPEED		40000
#define MAX_PWM_VALUE				65535
#define BUFFER_SIZE					8
#define HEADER_IDX					0
#define STEERING_IDX				3
#define PEDAL_GAS_IDX				4
#define PEDAL_BRAKE_IDX				5
#define BUTTONS_IDX					6
#define CORRECTIVE_VALUE			500
#define HIGH						1
#define LOW							0
/* ------------------------------------- */
/*                  MASKS                */
/* ------------------------------------- */
#define CLUTCH_INCREMENT_MASK		(uint8_t) 1
#define CLUTCH_DECREMENT_MASK		(uint8_t) 2

/* ------------------------------------- */
/*                  VALIDATIONS          */
/* ------------------------------------- */

#define TUNNING_VIBRATION_VALUE	5
#define TUNNING_LEFT			80
#define MAX_TUNNING_LEFT		20
#define RIGHT_SAFETY			30


/* ============================================================================ */
/*                                  	TYPES                           	    */
/* ============================================================================ */
enum Clutch_Values {
	BACK = 0,
	STOP,
	ONE,
	TWO,
	THREE,
	FOUR,
};

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

extern TIM_HandleTypeDef htim1;
extern volatile steering_frame_t steering_data;
extern int32_t Max_Steering_Right;
extern int32_t Max_Steering_Left;
extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;
extern osSemaphoreId_t Semaphore3Handle;
extern TIM_HandleTypeDef htim3;
extern uint8_t uart_rx_buffer[BUFFER_SIZE];
extern uint8_t ultrasonic_flag;

/* ============================================================================ */
/*                                  GLOBAL VARIABLES                    	    */
/* ============================================================================ */
TIM_OC_InitTypeDef sConfigOC[3];
uint8_t clutch_value = 1;
uint8_t pwm_value_Back_Right = 0;
uint8_t pwm_value_Back_Left = 0;
uint8_t pwm_value_steering = 0;




/* ============================================================================ */
/*                                  	APIS                           	    */
/* ============================================================================ */

static uint32_t abs_diff(uint32_t a, uint32_t b);
static void pwm_init(void);
static void setMotorDirection(uint8_t dir);
static void controllingSM(void);
static void checkClutchValue(void);
static int32_t map_value(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
static void setSteeringAngle(void);
static void checkSteering(void);
void stopMotorUltrasonic(void);
static void ButtonHandling (void);


/* ============================================================================ */
/*                                  IMPLEMENTATION                           	*/
/* ============================================================================ */

/**
 * @brief Main control function for managing PWM and encoder operations.
 *
 * This function initializes PWM channels and encoder interface, then enters
 * an infinite loop where it waits for a semaphore to control execution flow.
 * Upon receiving the semaphore, it calls state machine functions for control
 * and checks steering conditions. It then releases a semaphore to signal completion
 * of its tasks.
 */
void Controlling(void)
{
    pwm_init(); // Initialize PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM for channel 1 of TIM1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Start PWM for channel 2 of TIM1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM for channel 3 of TIM1
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Start encoder interface for TIM3

    while (1) // Infinite loop for continuous operation
    {
        // Wait indefinitely for Semaphore2Handle to be taken
        if (xSemaphoreTake(Semaphore2Handle, HAL_MAX_DELAY) == pdTRUE)
        {
            controllingSM(); // Execute state machine for controlling actions
            checkSteering(); // Check steering conditions
            xSemaphoreGive(Semaphore1Handle); // Release Semaphore1Handle to signal completion
        }
    }
}

static void controllingSM(void)
{


	/*
	 * maps the gas pedal index from its minimum and maximum values to a range of 0 to 100.
	 * maps the brake pedal index from its minimum and maximum values to a range of 0 to 100.
	 * calculate The absolute difference between two values in the range [0, 100].
	 * */
	uint32_t pedal_value = abs_diff(map_value(uart_rx_buffer[PEDAL_GAS_IDX],PEDAL_GAS_MIN,PEDAL_GAS_MAX,0,100)\
						,map_value(uart_rx_buffer[PEDAL_BRAKE_IDX],PEDAL_BRAKE_MIN,PEDAL_BRAKE_MAX,0,100));

	/*
	 *  Check if the brake pedal is pressed but not fully, and the gas pedal is not pressed
	 *  OR
	 *  Check if the brake pedal is fully pressed
	 *  Set the pedal value to 0, ensuring a FULL_STOP
	 *
	 * */

	if(uart_rx_buffer[PEDAL_BRAKE_IDX] < PEDAL_BRAKE_MAX && uart_rx_buffer[PEDAL_GAS_IDX] == 0)
	{
		pedal_value = 0;
	    HAL_GPIO_WritePin(GPIOB, BACK_LIGHTS_PIN, HIGH);

	}

	else if(uart_rx_buffer[PEDAL_BRAKE_IDX] == PEDAL_BRAKE_MAX)
	{
		pedal_value = 0;
		 HAL_GPIO_WritePin(GPIOB, BACK_LIGHTS_PIN, HIGH);

	}

	if (uart_rx_buffer[PEDAL_BRAKE_IDX] > 20  )
		{
			 HAL_GPIO_WritePin(GPIOB, BACK_LIGHTS_PIN, HIGH);

		}
	else
	{
		 HAL_GPIO_WritePin(GPIOB, BACK_LIGHTS_PIN, LOW);

	}




	/*
	 * if((uart_rx_buffer[PEDAL_BRAKE_IDX] < 255 && uart_rx_buffer[PEDAL_GAS_IDX] == 0) || uart_rx_buffer[PEDAL_BRAKE_IDX] == 255)
	{
		pedal_value = 0;
	}
	 *
	 * */



	checkClutchValue();

	/*
	 * Setting Motor Direction: Depending on the clutch value, the motor direction is set to either forward
	 *   or backward.
	 * Setting PWM Values: The PWM values for the back-right and back-left motors are adjusted
	 *   based on the clutch value. This determines the speed of the motors as a percentage of the pedal_value.
	 * Handling Edge Cases: The default case is included to handle unexpected clutch values,
	 *    which is important for safety or error handling.
	 * */
	switch (clutch_value)
	{
	    case BACK:
	        /* Set motor direction to reverse */
	        setMotorDirection(DIR_BACK);
	        /* Set the PWM values for both back motors to 75% of the pedal value */
	        pwm_value_Back_Right = 75 * (pedal_value / 100.0);
	        pwm_value_Back_Left = 75 * (pedal_value / 100.0);
	        break;

	    case STOP:
	        /* Set motor direction to forward (neutral state for stopping) */
	        setMotorDirection(DIR_FWD);
	        /* Set the PWM values for both back motors to 0, stopping the motors */
	        pwm_value_Back_Right = 0;
	        pwm_value_Back_Left = 0;
	        break;

	    case ONE:
	        /* Set motor direction to forward */
	        setMotorDirection(DIR_FWD);
	        /* Set the PWM values for both back motors to 25% of the pedal value */
	        pwm_value_Back_Right = 25 * (pedal_value / 100.0);
	        pwm_value_Back_Left = 25 * (pedal_value / 100.0);
	        break;

	    case TWO:
	        /* Set motor direction to forward */
	        setMotorDirection(DIR_FWD);
	        /* Set the PWM values for both back motors to 50% of the pedal value */
	        pwm_value_Back_Right = 50 * (pedal_value / 100.0);
	        pwm_value_Back_Left = 50 * (pedal_value / 100.0);
	        break;

	    case THREE:
	        /* Set motor direction to forward */
	        setMotorDirection(DIR_FWD);
	        /* Set the PWM values for both back motors to 75% of the pedal value */
	        pwm_value_Back_Right = 75 * (pedal_value / 100.0);
	        pwm_value_Back_Left = 75 * (pedal_value / 100.0);
	        break;

	    case FOUR:
	        /* Set motor direction to forward */
	        setMotorDirection(DIR_FWD);
	        /* Set the PWM values for both back motors to 100% of the pedal value */
	        pwm_value_Back_Right = 100 * (pedal_value / 100.0);
	        pwm_value_Back_Left = 100 * (pedal_value / 100.0);
	        break;

	    default:
	        /* Handle unexpected clutch values (safety or error handling) */
	        break;
	}


//	ButtonHandling ();
	/* Calculate the PWM pulse value for the back-right motor as a percentage of the maximum PWM value */
	sConfigOC[BACK_RIGHT_MOTOR].Pulse = ((pwm_value_Back_Right / 100.0) * MAX_PWM_VALUE);
	/* Configure the PWM channel for the back-right motor */
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);

	/* Calculate the PWM pulse value for the back-left motor as a percentage of the maximum PWM value */
	sConfigOC[BACK_LEFT_MOTOR].Pulse = ((pwm_value_Back_Left / 100.0) * MAX_PWM_VALUE);
	/* Configure the PWM channel for the back-left motor */
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);

	/* Start the PWM signal generation for the back-right motor */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	/* Start the PWM signal generation for the back-left motor */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

}






/**
 * @brief Sets the direction of both back motors.
 *
 * This function controls the direction of two motors connected to GPIO pins.
 * It writes the specified direction (`dir`) to both the BACK_LEFT_MOTOR_DIR_PIN
 * and BACK_RIGHT_MOTOR_DIR_PIN GPIO pins on GPIOB.
 *
 * @param dir The direction to set for both motors.
 *            - 0: Motor direction depends on the specific pin logic (assuming LOW or HIGH logic).
 *            - 1: Motor direction depends on the specific pin logic (assuming HIGH or LOW logic).
 */
static void setMotorDirection(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOB, BACK_LEFT_MOTOR_DIR_PIN, dir); // Set direction for back left motor
    HAL_GPIO_WritePin(GPIOB, BACK_RIGHT_MOTOR_DIR_PIN, dir); // Set direction for back right motor
}


/**
 * @brief Initializes PWM configuration for specified channels.
 *
 * This function initializes PWM configuration parameters for three PWM channels
 * using the HAL library. It sets the PWM mode, initial pulse width, polarity,
 * and fast mode settings for each channel.
 */
static void pwm_init(void)
{
    uint8_t i = 0; // Initialize loop counter variable

    // Loop through each PWM channel configuration
    for (i = 0; i < 3; i++)
    {
        // Set PWM mode to PWM1 (upcounting)
        sConfigOC[i].OCMode = TIM_OCMODE_PWM1;
        // Set initial pulse width to 0 (starting with 0 duty cycle)
        sConfigOC[i].Pulse = 0;
        // Set output polarity to high (active-high PWM signal)
        sConfigOC[i].OCPolarity = TIM_OCPOLARITY_HIGH;
        // Disable fast mode (no fast mode needed)
        sConfigOC[i].OCFastMode = TIM_OCFAST_DISABLE;
    }

    // Configure PWM channels for TIM1
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
}


static uint32_t abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}



/**
 * @brief Updates the clutch value based on button inputs.
 *
 * This function reads the states of the clutch increment and decrement buttons from the UART buffer.
 * It adjusts the clutch value accordingly, ensuring it remains within the valid range of 0 to 5.
 * The function detects button releases to avoid multiple unintended increments or decrements.
 *
 * The tick of the clutch is considered during releasing to avoid multiple wrong ticks.
 * Specifically, the function:
 * - Increments the clutch value when the increment button is pressed and released.
 * - Decrements the clutch value when the decrement button is pressed and released.
 * - Ensures the clutch value does not exceed the maximum value of 5.
 * - Resets the clutch value to 0 if an underflow condition occurs during decrement.
 *
 * Persistent variables are used to track the previous state of the increment and decrement buttons
 * to detect button release events.
 */


static void checkClutchValue(void)
{
    uint8_t current_increment;
    uint8_t current_decrement;
    static uint8_t prev_increment = 0; /* Persistent variable to track previous increment state*/
    static uint8_t prev_decrement = 0; /*Persistent variable to track previous decrement state*/

    // Check for clutch increment
    current_increment = uart_rx_buffer[BUTTONS_IDX] & CLUTCH_INCREMENT_MASK;

    if (current_increment == CLUTCH_INCREMENT_MASK) // Assuming CLUTCH_INCREMENT_MASK is 1 when the increment button is pressed
    {
        // Check for button release
        if (current_increment != prev_increment)
        {
            clutch_value++;
            if (clutch_value > 5)
            {
                clutch_value = 5;
            }
        }
        prev_increment = current_increment;
    }
    else
    {
        prev_increment = 0; // Reset prev_increment if the increment button is not pressed
    }

    // Check for clutch decrement
    current_decrement = (uart_rx_buffer[BUTTONS_IDX] & CLUTCH_DECREMENT_MASK) >> 1;

    if (current_decrement == (CLUTCH_DECREMENT_MASK >> 1)) // Assuming CLUTCH_DECREMENT_MASK is 2 when the decrement button is pressed
    {
        // Check for button release
        if (current_decrement != prev_decrement)
        {
            clutch_value--;
            if (clutch_value > 5) // assuming clutch_value is unsigned and will wrap around to 255 when decremented from 0
            {
                clutch_value = 0;
            }
        }
        prev_decrement = current_decrement;
    }
    else
    {
        prev_decrement = 0; // Reset prev_decrement if the decrement button is not pressed
    }
}


/**
 * @brief Maps an input value from one range to another using linear interpolation.
 *
 * This function maps the input value `value` from the input range defined by `in_min`
 * and `in_max` to the output range defined by `out_min` and `out_max` using a linear
 * interpolation formula. If `value` exceeds the input range, it is clipped to `in_min`
 * or `in_max` accordingly.
 *
 * @param value Input value to be mapped.
 * @param in_min Minimum value of the input range.
 * @param in_max Maximum value of the input range.
 * @param out_min Minimum value of the output range.
 * @param out_max Maximum value of the output range.
 * @return Mapped value within the output range.
 */
static int32_t map_value(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    // Ensure the input value is within the expected range
    if (value < in_min)
    {
        value = in_min; // Clip value to minimum input range if below minimum
    }
    else if (value > in_max)
    {
        value = in_max; // Clip value to maximum input range if above maximum
    }

    // Apply linear transformation formula to map value from input range to output range
    int32_t mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    return mapped_value; // Return mapped value within the output range
}



/**
 * @brief Sets the steering angle based on received UART data and adjusts using an encoder.
 *
 * This function maps the received steering angle from UART (`uart_rx_buffer[STEERING_IDX]`)
 * to a corresponding encoder value using `map_value`. It then adjusts the steering position
 * by controlling a motor connected to GPIOB and TIM1 based on the difference between the current
 * encoder value (`TIM3->CNT`) and the mapped steering value. The function supports both left
 * and right steering movements and includes timeout handling to prevent continuous operation.
 */
static void setSteeringAngle(void)
{
    const TickType_t timeoutTicks = pdMS_TO_TICKS(1000); // Convert 1 second to tick count
    TickType_t startTickCount = xTaskGetTickCount();

    // Map the received steering value to an encoder value
    int32_t steering_mapped_value = map_value(uart_rx_buffer[STEERING_IDX], MIN_STEERING_VALUE, MAX_STEERING_VALUE, Max_Steering_Left + TUNNING_LEFT, Max_Steering_Right);
    int32_t currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;

    // Adjust steering position based on current encoder value and mapped steering value
    if (currentEncoderValue < steering_mapped_value)
    {
        /* Move left */
        while (currentEncoderValue < (steering_mapped_value - TUNNING_VIBRATION_VALUE) && currentEncoderValue <= (Max_Steering_Left + MAX_TUNNING_LEFT))
        {
            HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_LEFT); // Set direction to left
            sConfigOC[STEERING_MOTOR].Pulse = STEERING_MOTOR_SPEED; // Set motor speed
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3); // Configure PWM channel
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM for steering motor
            currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE; // Update current encoder value

            // Check for timeout
            if ((xTaskGetTickCount() - startTickCount) > timeoutTicks)
            {
                break;
            }
        }
    }
    else if (currentEncoderValue > steering_mapped_value)
    {
        /* Move right */
        while (currentEncoderValue > (steering_mapped_value + TUNNING_VIBRATION_VALUE) && currentEncoderValue >= (Max_Steering_Right + RIGHT_SAFETY))
        {
            HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_RIGHT); // Set direction to right
            sConfigOC[STEERING_MOTOR].Pulse = STEERING_MOTOR_SPEED; // Set motor speed
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3); // Configure PWM channel
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM for steering motor
            currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE; // Update current encoder value

            // Check for timeout
            if ((xTaskGetTickCount() - startTickCount) > timeoutTicks)
            {
                break;
            }
        }
    }

    // Stop the motor
    sConfigOC[STEERING_MOTOR].Pulse = 0; // Set PWM pulse to 0 to stop motor
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3); // Configure PWM channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM with 0 pulse (motor stop)
}


/**
 * @brief Checks the steering angle from UART input and adjusts if necessary.
 *
 * This function monitors the steering angle received from UART (`uart_rx_buffer[STEERING_IDX]`).
 * If the steering angle is outside predefined mid-range thresholds (`STEERING_MAX_MID` or `STEERING_MIN_MID`),
 * it calls `setSteeringAngle()` to adjust the steering angle using a motor. It also ensures that the motor
 * stops if the steering angle remains unchanged to prevent continuous operation.
 */
static void checkSteering(void)
{
    static uint8_t prev = 0; // Persistent variable to track previous steering angle

    // Check if the steering angle is outside mid-range thresholds
    if (uart_rx_buffer[STEERING_IDX] >= STEERING_MAX_MID || uart_rx_buffer[STEERING_IDX] <= STEERING_MIN_MID)
    {
        // Check if the current steering angle is different from the previous one
        if (uart_rx_buffer[STEERING_IDX] != prev)
        {
            setSteeringAngle(); // Adjust steering angle using motor
            prev = uart_rx_buffer[STEERING_IDX]; // Update previous steering angle
        }
        else
        {
            // Stop the motor
            sConfigOC[STEERING_MOTOR].Pulse = 0; // Set PWM pulse to 0 to stop motor
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3); // Configure PWM channel
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Start PWM with 0 pulse (motor stop)
        }
    }
}


/**
 * @brief Stops the motors controlling the ultrasonic module.
 *
 * This function checks if the `ultrasonic_flag` is set. If `ultrasonic_flag` is true,
 * it sets the PWM pulse to 0 for both back right and back left motors (`BACK_RIGHT_MOTOR` and `BACK_LEFT_MOTOR`).
 * It then configures and starts PWM signals for both motors to stop them from running.
 * Finally, it clears the `ultrasonic_flag` to indicate that the motors have been stopped.
 */
void stopMotorUltrasonic(void)
{
    if (ultrasonic_flag)
    {
        // Set PWM pulse to 0 for back right motor
        sConfigOC[BACK_RIGHT_MOTOR].Pulse = 0;
        HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);

        // Set PWM pulse to 0 for back left motor
        sConfigOC[BACK_LEFT_MOTOR].Pulse = 0;
        HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);

        // Start PWM signals to stop both motors
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    }

    ultrasonic_flag = 0; // Clear ultrasonic_flag to indicate motors have been stopped
}


static void ButtonHandling (void)
{
	if ((uart_rx_buffer[BUTTONS_IDX] & 0x08)>>3 )
	{
		HAL_GPIO_WritePin(GPIOB, BACK_LEFT_MOTOR_DIR_PIN, HIGH); // Set direction for back left motor to be forward
		HAL_GPIO_WritePin(GPIOB, BACK_RIGHT_MOTOR_DIR_PIN, LOW); // Set direction for back right motor to be backward

	}
	else if ((uart_rx_buffer[BUTTONS_IDX] & 0x20)>>5)
	{
		HAL_GPIO_WritePin(GPIOB, BACK_LEFT_MOTOR_DIR_PIN, LOW); // Set direction for back left motor  to be backward
		HAL_GPIO_WritePin(GPIOB, BACK_RIGHT_MOTOR_DIR_PIN,HIGH); // Set direction for back right motor to be forward

	}
	else
	{

	}



	//button 16  >> front light  0x10
	// button 4 >>  horn 		 0x04
	if ( (uart_rx_buffer[BUTTONS_IDX] & 0x10)>>4 )
	{
		static uint8_t flag =0;
		if (flag ==0)
		{
		 HAL_GPIO_WritePin(GPIOB, FRONT_LIGHTS_PIN, HIGH);
		 flag = 1;
		}
		else if (flag == 1)
		{
		HAL_GPIO_WritePin(GPIOB, FRONT_LIGHTS_PIN, LOW);
		flag = 0;

		}

	}



	if ( (uart_rx_buffer[BUTTONS_IDX] & 0x04)>>2 )
		{
			 HAL_GPIO_WritePin(GPIOB, HORN_PIN, HIGH);

		}
		else
		{
			 HAL_GPIO_WritePin(GPIOB, HORN_PIN, LOW);

		}



}



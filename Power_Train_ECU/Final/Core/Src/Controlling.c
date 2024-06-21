/**
  ******************************************************************************
  * @file           : Controlling.c
  * @brief          : Module That Responsible To Control Steering and DC Motor
  *                   for RC Car.
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <stdio.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"



// Configure PWM parameters
#define PWM_FREQUENCY   			10000    // PWM frequency in Hz
#define PWM_MAX_DUTY    			100      // Maximum PWM duty cycle (0-100%)
#define BACK_RIGHT_MOTOR_DIR_PIN	GPIO_PIN_14
#define BACK_LEFT_MOTOR_DIR_PIN		GPIO_PIN_15
#define STEERING_MOTOR_DIR_PIN		GPIO_PIN_13
#define BACK_RIGHT_MOTOR			0
#define BACK_LEFT_MOTOR				1
#define STEERING_MOTOR				2
#define DIR_FWD						0
#define DIR_BACK					1
#define STEERING_RIGHT				0
#define STEERING_LEFT				1
#define CLUTCH_INCREMENT_MASK		(uint8_t) 1
#define CLUTCH_DECREMENT_MASK		(uint8_t) 2
#define PEDAL_GAS_MAX				255
#define PEDAL_GAS_MIN				0
#define PEDAL_BRAKE_MAX				255
#define PEDAL_BRAKE_MIN				0
#define MAX_STEERING_VALUE			255
#define MIN_STEERING_VALUE			0
#define STEERING_MIN_MID			124
#define STEERING_MAX_MID			130
#define STEERING_MOTOR_SPEED		20000
#define MAX_PWM_VALUE				65535
#define BUFFER_SIZE					5
#define HEADER_IDX					0
#define STEERING_IDX				1
#define PEDAL_GAS_IDX				2
#define PEDAL_BRAKE_IDX				3
#define BUTTONS_IDX					4
#define CORRECTIVE_VALUE			500


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
    uint16_t steering;
    uint8_t pedal_gas;
    uint8_t pedal_brake;
    uint8_t clutch;
    uint8_t buttons;
    uint8_t crc;
} steering_frame_t;

TIM_OC_InitTypeDef sConfigOC[3];
extern TIM_HandleTypeDef htim1;
extern volatile steering_frame_t steering_data;
uint8_t pwm_value_Back_Right = 0;
uint8_t pwm_value_Back_Left = 0;
uint8_t pwm_value_steering = 0;
extern int32_t Max_Steering_Right;
extern int32_t Max_Steering_Left;
uint8_t clutch_value = 1;
extern osSemaphoreId_t Semaphore1Handle;
extern osSemaphoreId_t Semaphore2Handle;
extern osSemaphoreId_t Semaphore3Handle;
extern TIM_HandleTypeDef htim3;
extern uint8_t uart_rx_buffer[BUFFER_SIZE];

// Global variables to keep track of the encoder state

static uint32_t abs_diff(uint32_t a, uint32_t b);
static void pwm_init(void);
void setMotorDirection(uint8_t dir);
void controllingSM(void);
static void checkClutchValue(void);
static int32_t map_value(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
static void setSteeringAngle(void);


void controllingSM(void)
{

	/* Set the Direction of Motors to FWD */
	//setMotorDirection(DIR_FWD);

	//pwm_value_steering = map_value(steering_data.steering, MIN_STEERING_VALUE, MAX_STEERING_VALUE, 0, 100);
	uint32_t pedal_value = abs_diff(map_value(uart_rx_buffer[PEDAL_GAS_IDX],PEDAL_GAS_MIN,PEDAL_GAS_MAX,0,100)\
						,map_value(uart_rx_buffer[PEDAL_BRAKE_IDX],PEDAL_BRAKE_MIN,PEDAL_BRAKE_MAX,0,100));
	if(uart_rx_buffer[PEDAL_BRAKE_IDX] < 255 && uart_rx_buffer[PEDAL_GAS_IDX] == 0)
	{
		pedal_value = 0;
	}
	else if(uart_rx_buffer[PEDAL_BRAKE_IDX] == 255)
	{
		pedal_value = 0;
	}
	/*sConfigOC[BACK_RIGHT_MOTOR].Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);
	sConfigOC[BACK_LEFT_MOTOR].Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);*/
//	checkClutchValue();
//	switch (clutch_value)
//	{
//		case BACK:
//			setMotorDirection(DIR_BACK);
//			pwm_value_Back_Right = 75 * (pedal_value / 100.0);
//			pwm_value_Back_Left = 75 * (pedal_value / 100.0);
//			break;
//		case STOP:
//			setMotorDirection(DIR_FWD);
//			pwm_value_Back_Right = 0;
//			pwm_value_Back_Left = 0;
//			break;
//		case ONE:
//			setMotorDirection(DIR_FWD);
//			pwm_value_Back_Right = 25 * (pedal_value / 100.0);
//			pwm_value_Back_Left = 25 * (pedal_value / 100.0);
//			break;
//		case TWO:
//			setMotorDirection(DIR_FWD);
//			pwm_value_Back_Right = 50 * (pedal_value / 100.0);
//			pwm_value_Back_Left = 50 * (pedal_value / 100.0);
//			break;
//		case THREE:
//			setMotorDirection(DIR_FWD);
//			pwm_value_Back_Right = 75 * (pedal_value / 100.0);
//			pwm_value_Back_Left = 75 * (pedal_value / 100.0);
//			break;
//		case FOUR:
//			setMotorDirection(DIR_FWD);
//			pwm_value_Back_Right = 100 * (pedal_value / 100.0);
//			pwm_value_Back_Left = 100 * (pedal_value / 100.0);
//			break;
//		default:
//			/* Handle unexpected clutch values*/
//			break;
//	}

	setMotorDirection(DIR_FWD);
	//pwm_value_Back_Right = 100 * (pedal_value / 100.0);
	//pwm_value_Back_Left = 100 * (pedal_value / 100.0);
	sConfigOC[BACK_RIGHT_MOTOR].Pulse = MAX_PWM_VALUE;//((pwm_value_Back_Right / 100.0))* MAX_PWM_VALUE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);

	sConfigOC[BACK_LEFT_MOTOR].Pulse = MAX_PWM_VALUE;//((pwm_value_Back_Left / 100.0))* MAX_PWM_VALUE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    //setSteeringAngle();
}


void Controlling(void)
{
    pwm_init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    while (1)
    {
    	if( xSemaphoreTake(Semaphore2Handle, HAL_MAX_DELAY) == pdTRUE)
    	{
    		controllingSM();
            xSemaphoreGive(Semaphore1Handle);
    	}
    }
}

void setMotorDirection(uint8_t dir)
{
	HAL_GPIO_WritePin(GPIOB, BACK_LEFT_MOTOR_DIR_PIN, dir);
	HAL_GPIO_WritePin(GPIOB, BACK_RIGHT_MOTOR_DIR_PIN, dir);
}

static void pwm_init(void)
{
    /* Configure PWM channels*/
	uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
        sConfigOC[i].OCMode = TIM_OCMODE_PWM1;
        sConfigOC[i].Pulse = 0;
        sConfigOC[i].OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC[i].OCFastMode = TIM_OCFAST_DISABLE;
    }
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
}

static uint32_t abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}


static void checkClutchValue(void)
{
	if((uart_rx_buffer[BUTTONS_IDX] & CLUTCH_INCREMENT_MASK) == 1)
	{
		clutch_value++;
		if(clutch_value == 6)
		{
			clutch_value = 5;
		}
		else
		{
			/**/
		}

	}
	else if((uart_rx_buffer[BUTTONS_IDX] & CLUTCH_DECREMENT_MASK) >> 1 == 1)
	{
		clutch_value--;
		if(clutch_value == 255)
		{
			clutch_value = 0;
		}
		else
		{
			/**/
		}

	}
	else
	{
		/*Do Nothing*/
	}


}

static int32_t map_value(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    /* Ensure the input value is within the expected range */
    if (value < in_min)
    {
        value = in_min;
    }
    else if (value > in_max)
    {
        value = in_max;
    }

    /* Apply the linear transformation formula */
    int32_t mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return mapped_value;
}

static void setSteeringAngle(void)
{
    int32_t steering_mapped_value = map_value(uart_rx_buffer[STEERING_IDX], MIN_STEERING_VALUE, MAX_STEERING_VALUE, Max_Steering_Left, Max_Steering_Right);
    int32_t currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;

    if (currentEncoderValue < steering_mapped_value)
    {        /* Move left */
        while (currentEncoderValue < steering_mapped_value && currentEncoderValue <= (Max_Steering_Left - 20))
        {
        	currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;
            HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_LEFT);
            sConfigOC[STEERING_MOTOR].Pulse = STEERING_MOTOR_SPEED;
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;
        }
    }
    else if (currentEncoderValue > steering_mapped_value)
    {
        /* Move right */
        while (currentEncoderValue > steering_mapped_value && currentEncoderValue >= (Max_Steering_Right + 20))
        {
        	currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;
            HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_RIGHT);
            sConfigOC[STEERING_MOTOR].Pulse = STEERING_MOTOR_SPEED;
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            currentEncoderValue = ((int32_t)(int16_t)TIM3->CNT) + CORRECTIVE_VALUE;
        }
    }

    // Stop the motor
    sConfigOC[STEERING_MOTOR].Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}



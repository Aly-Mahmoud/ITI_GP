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

enum Clutch_Values {
	BACK,
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
extern volatile uint32_t adcValue;
uint8_t pwm_value_Back_Right = 0;
uint8_t pwm_value_Back_Left = 0;
uint8_t pwm_value_steering = 0;

static uint32_t abs_diff(uint32_t a, uint32_t b);
static void pwm_init(void);
void setMotorDirection(uint8_t dir);
void controllingSM(void);

void controllingSM(void)
{
	/* Set the Direction of Motors to FWD */
	setMotorDirection(DIR_FWD);
	pwm_value_steering = steering_data.steering;
	uint32_t pedal_value = abs_diff(steering_data.pedal_gas, steering_data.pedal_brake);
	switch (steering_data.clutch)
	{
		case BACK:
			setMotorDirection(DIR_BACK);
			pwm_value_Back_Right = 50 * (pedal_value / 100);
			pwm_value_Back_Left = 50 * (pedal_value / 100);
			break;
		case ONE:
			pwm_value_Back_Right = 25 * (pedal_value / 100);
			pwm_value_Back_Left = 25 * (pedal_value / 100);
			break;
		case TWO:
			pwm_value_Back_Right = 50 * (pedal_value / 100);
			pwm_value_Back_Left = 50 * (pedal_value / 100);
			break;
		case THREE:
			pwm_value_Back_Right = 75 * (pedal_value / 100);
			pwm_value_Back_Left = 75 * (pedal_value / 100);
			break;
		case FOUR:
			pwm_value_Back_Right = 100 * (pedal_value / 100);
			pwm_value_Back_Left = 100 * (pedal_value / 100);
			break;
		default:
			/* Handle unexpected clutch values*/
			break;
	}
}

// Define PrevFlag outside the loop
char PrevFlag = 0;

void Controlling(void)
{
    pwm_init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    while (1)
    {
//        controllingSM();
        osDelay(1);

        // Set PWM pulse width for back motors
        sConfigOC[BACK_RIGHT_MOTOR].Pulse = 65000;
        //HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_RIGHT_MOTOR], TIM_CHANNEL_1);

        sConfigOC[BACK_LEFT_MOTOR].Pulse = 65000;
        //HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[BACK_LEFT_MOTOR], TIM_CHANNEL_2);

        // Adjust steering based on ADC value
//        if (adcValue >= 4000 )
//        {
//            // Right movement
//            if (1)
//            {
//                HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_RIGHT);
//                sConfigOC[STEERING_MOTOR].Pulse = 2000;
//                HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
//                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//
//                // Delay to allow visible steering movement
//                //osDelay(1000);
//                if(adcValue <= 3800)
//                {
//                	HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_LEFT);
//                	sConfigOC[STEERING_MOTOR].Pulse = 2000;
//                	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
//                	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//
//                	                // Delay to allow visible steering movement
//                	                //osDelay(1000);
//                }
//
//            }

//        }
        while(1)
        {
        	HAL_GPIO_WritePin(GPIOB, STEERING_MOTOR_DIR_PIN, STEERING_RIGHT);
        	sConfigOC[STEERING_MOTOR].Pulse = 20000;
        	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
        	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        }
			sConfigOC[STEERING_MOTOR].Pulse = 0;
			HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC[STEERING_MOTOR], TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			osDelay(1);
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

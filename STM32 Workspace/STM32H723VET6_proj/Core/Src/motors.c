/*
 * motors.c
 *
 *  Created on: Apr 15, 2023
 *      Author: tntcl
 */

// Includes
#include "stm32h7xx_hal.h"
#include "wizchip_conf.h"
#include "motors.h"
#include <stdlib.h>

// Defines
#define SPEED 500

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;

/*
 * Function: 	mv_RotMotor
 * Description: Starts moving the rotation motor in the direction specified by dir.
 * Parameters:	uint8_t 	dir,	1 for left 0 for right
 */
void mv_RotMotor(uint8_t dir) {
	if(dir == LEFT) {	// If direction is left set direction pin
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	} else {	// If direction is right reset direction pin
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	HAL_Delay(10);

	TIM1->ARR = SPEED;		// Set registers to achieve necessary PWM control signal
	TIM1->CCR4 = SPEED/2;	// Set 50% duty cycle
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Start timer 1 channel 4 to send control PWM signal
}

/*
 * Function: 	st_RotMotor
 * Description: Stops the rotation motor
 */
void st_RotMotor(){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);	// Stop PWM control signal
}

/*
 * Function: 	set_LiftMotor
 * Description: Sets the lift motors elevation based on the provided value
 * Parameters:	uint8_t 	pos,	0 to 70 value for degrees of elevation
 */
void set_LiftMotor(uint8_t pos) {
	float percent; 					// Variable to store floating percent value
	percent = (float)pos * 1.42857;	// Calculate percent extension based on desired elevation angle


	TIM8->CCR4 = (pos * -10) + 1000;			// Set compare contrast register for timer 8
												// channel 4 to to achieve necessary duty cycle
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	// Start timer 8 channel 4 to control motor
}

/*
 * Function: 	home_LiftMotor
 * Description: Sets the lift motor to the home position.
 */
void home_LiftMotor() {
	set_LiftMotor(0);
}

/*
 * Function: 	move_To_Position
 * Description: Move both motors to achieve a desired position.
 * 				Dynamically waits to make sure elevation motor has settled.
 * Parameters:	int 		rot,	Value for number of pulses to be sent to motor.
 * 									+ is to the right
 * 									- is to the left
 * 				uint8_t		elev,	0 to 70 for degrees of elevation
 */
void move_To_Position(int rot, uint8_t elev) {
	int time;				// Variable to store millisecond to achieve desired pulses
	time = abs(rot);		// Set time based on number of desired pulses
	set_LiftMotor(elev);	// Set elevation motors position
	if(rot < 0) {			// If rot is negative
		mv_RotMotor(LEFT);	// start rotation motor to the left
	} else {				// Else rot is positive
		mv_RotMotor(RIGHT);	// start rotation to the right
	}
	HAL_Delay(time);		// Wait for correct number of pulse to be sent
	st_RotMotor();			// Stop rotation motor
	HAL_Delay(2000 - time);	// Wait to make sure elevation motor has reached target
}

/*
 * motors.h
 *
 *  Created on: Apr 15, 2023
 *      Author: tntcl
 */

#ifndef SRC_MOTORS_MOTORS_H_
#define SRC_MOTORS_MOTORS_H_

// Function declaration
void mv_RotMotor(uint8_t dir);
void st_RotMotor(void);
void set_LiftMotor(uint8_t pos);
void home_RotMotor(void);
void move_To_Position(int rot, uint8_t elev);

// Defines
#define LEFT 1
#define RIGHT 0

#endif /* SRC_MOTORS_MOTORS_H_ */

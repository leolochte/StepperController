/*
 * stepper.c
 *
 *  Created on: 05.04.2021
 *      Author: Leo
 *
 *
 *
 */

/*INCLUDES*/
#include "stepper.h"
#include "main.h"

/*FUNCTIONS*/
void moveSteppers_micro_sync(unsigned int time) {
	arraylength = sizeof(steppersArray)/sizeof(steppersArray[0]);
	for (int i = 0; i < arraylength; ++i) {
		steppersArray[i].counter = 0;
		steppersArray[i].current_step = 0;
		steppersArray[i].required_delay = 0.676 * 1000000 * 10 * sqrt(2*(2*3.14/1600)/(3*3.14/2*steppersArray[i].steps*pow((double)2000/time, 2)));
		HAL_GPIO_WritePin(steppersArray[i].DIRGPIOPORT, steppersArray[i].DIRGPIOPIN, steppersArray[i].direction);
	}
}

void moveSteppers_micro_single(unsigned int time, unsigned int array_pos, float direction, float degrees) {
	arraylength = sizeof(steppersArray)/sizeof(steppersArray[0]);
	steppersArray[array_pos].steps = degrees/(1.8/16);
	steppersArray[array_pos].direction = direction;
	steppersArray[array_pos].counter = 0;
	steppersArray[array_pos].current_step = 0;

	HAL_GPIO_WritePin(steppersArray[array_pos].DIRGPIOPORT, steppersArray[array_pos].DIRGPIOPIN, steppersArray[array_pos].direction);

	if (time == 0) {
		steppersArray[array_pos].required_delay = 0;
	} else {
		steppersArray[array_pos].required_delay = 0.676 * 1000000 * 10 * sqrt(2*(2*3.14/1600)/(3*3.14/2*steppersArray[array_pos].steps*pow((double)2000/time, 2)));
	}
}

void make_movement(movement_t movement) {
	arraylength = sizeof(steppersArray)/sizeof(steppersArray[0]);
	steppersArray[movement.stepper].steps = movement.degrees/(1.8/16);
	steppersArray[movement.stepper].direction = movement.direction;
	steppersArray[movement.stepper].counter = 0;
	steppersArray[movement.stepper].current_step = 0;

	HAL_GPIO_WritePin(steppersArray[movement.stepper].DIRGPIOPORT, steppersArray[movement.stepper].DIRGPIOPIN, steppersArray[movement.stepper].direction);

	if (movement.degrees == 0) {
		steppersArray[movement.stepper].required_delay = movement.time * 200;
	} else {
		steppersArray[movement.stepper].required_delay = 0.676 * 1000000 * 10 * sqrt(2*(2*3.14/1600)/(3*3.14/2*steppersArray[movement.stepper].steps*pow((double)2000/movement.time, 2)));
	}
}

void stepper_Init(void) {

	stepper_t stepper_A;
	stepper_t stepper_B;

	stepper_A.position = 0;
	stepper_A.STEPGPIOPORT = A_STEP_GPIO_Port;
	stepper_A.STEPGPIOPIN = A_STEP_Pin;
	stepper_A.DIRGPIOPORT = A_DIR_GPIO_Port;
	stepper_A.DIRGPIOPIN = A_DIR_Pin;

	stepper_B.position = 1;
	stepper_B.STEPGPIOPORT = B_STEP_GPIO_Port;
	stepper_B.STEPGPIOPIN = B_STEP_Pin;
	stepper_B.DIRGPIOPORT = B_DIR_GPIO_Port;
	stepper_B.DIRGPIOPIN = B_DIR_Pin;

	steppersArray[stepper_A.position] = stepper_A;
	steppersArray[stepper_B.position] = stepper_B;

	// Movements for stepper  B
	movementArray1[0][0].stepper	= 0;
	movementArray1[0][0].direction	= 0;
	movementArray1[0][0].degrees	= 1080;
	movementArray1[0][0].time		= 1500;

	movementArray1[0][1].stepper	= 0;
	movementArray1[0][1].direction	= 1;
	movementArray1[0][1].degrees	= 0;
	movementArray1[0][1].time		= 500;

	movementArray1[0][2].stepper	= 0;
	movementArray1[0][2].direction	= 0;
	movementArray1[0][2].degrees	= 0;
	movementArray1[0][2].time		= 3250;

	// Movements for stepper A
	movementArray1[1][0].stepper	= 1;
	movementArray1[1][0].direction	= 1;
	movementArray1[1][0].degrees	= 90;
	movementArray1[1][0].time		= 250;

	movementArray1[1][1].stepper	= 1;
	movementArray1[1][1].direction	= 0;
	movementArray1[1][1].degrees	= 0;
	movementArray1[1][1].time		= 1000;

	movementArray1[1][2].stepper	= 1;
	movementArray1[1][2].direction	= 0;
	movementArray1[1][2].degrees	= 90;
	movementArray1[1][2].time		= 250;
}

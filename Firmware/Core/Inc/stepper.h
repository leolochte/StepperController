/*
 * stepper.h
 *
 *  Created on: 05.04.2021
 *      Author: Leo
 */

/*INCLUDES*/
#include "stm32l4xx_hal.h"

/*FUNCTION DECLARATIONS*/
void moveSteppers_micro_sync(unsigned int time);

/*DEFINITIONS*/
struct stepper {	//contains every variable and info about one stepper, including ones being used to coordinate steps etc.
	volatile unsigned int mindelay;
	volatile unsigned int accel;
	volatile unsigned int direction;
	volatile unsigned int steps;

	volatile unsigned int counter;
	volatile unsigned int currentstep;
	volatile unsigned int nextDelay;

	GPIO_TypeDef* STEPGPIOPORT;
	uint16_t STEPGPIOPIN;

	GPIO_TypeDef* DIRGPIOPORT;
	uint16_t DIRGPIOPIN;
};

/*VARIABLES*/
volatile unsigned int steppersDone;
struct stepper steppersArray[2];

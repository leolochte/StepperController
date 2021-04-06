/*
 * stepper.h
 *
 *  Created on: 05.04.2021
 *      Author: Leo
 */

/*INCLUDES*/
#include "stm32l4xx_hal.h"
#include "math.h"

/*FUNCTION DECLARATIONS*/
void moveSteppers_micro_sync(unsigned int time);

/*DEFINITIONS*/
struct stepper {		// contains every variable and info about one stepper, including ones being used to coordinate steps etc.
	volatile float counter; 			// counts from 0 up on timer increments till the desired delay
	volatile float required_delay; 	// stores the delay till the next step
	volatile float current_step;
	volatile float direction;
	volatile float steps;

	GPIO_TypeDef* STEPGPIOPORT;
	uint16_t STEPGPIOPIN;

	GPIO_TypeDef* DIRGPIOPORT;
	uint16_t DIRGPIOPIN;
};

/*VARIABLES*/
volatile unsigned int steppersDone;
struct stepper steppersArray[2];
volatile uint32_t arraylength;

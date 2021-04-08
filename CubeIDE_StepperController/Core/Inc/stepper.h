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
void moveSteppers_micro_single(unsigned int time, unsigned int array_pos, float direction, float degrees);

/*DEFINITIONS*/
typedef struct {		// contains every variable and info about one stepper, including ones being used to coordinate steps etc.
	volatile float counter; 			// counts from 0 up on timer increments till the desired delay
	volatile float required_delay; 	// stores the delay till the next step
	volatile float current_step;
	volatile float direction;
	volatile float steps;

	GPIO_TypeDef* STEPGPIOPORT;
	uint16_t STEPGPIOPIN;

	GPIO_TypeDef* DIRGPIOPORT;
	uint16_t DIRGPIOPIN;
} stepper_t;

/*VARIABLES*/
volatile unsigned int steppersDone;
stepper_t steppersArray[2];
volatile uint32_t arraylength;

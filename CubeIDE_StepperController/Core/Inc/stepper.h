/*
 * stepper.h
 *
 *  Created on: 05.04.2021
 *      Author: Leo
 */

/*INCLUDES*/
#include "stm32l4xx_hal.h"
#include "math.h"

/*DEFINITIONS*/
typedef struct {		// contains every variable and info about one stepper, including ones being used to coordinate steps etc.
	volatile float counter; 			// counts from 0 up on timer increments till the desired delay
	volatile float required_delay; 	// stores the delay till the next step
	volatile float current_step;
	volatile float direction;
	volatile float steps;
	volatile int state;				// 0 = first base state, 1 to n = individual movement busy states
	volatile unsigned int position;

	GPIO_TypeDef* STEPGPIOPORT;
	uint16_t STEPGPIOPIN;

	GPIO_TypeDef* DIRGPIOPORT;
	uint16_t DIRGPIOPIN;
} stepper_t;

typedef struct {
	unsigned int stepper;
	unsigned int time;
	float degrees;
	float direction;
} movement_t;

/*FUNCTION DECLARATIONS*/
void moveSteppers_micro_sync(unsigned int time);
void moveSteppers_micro_single(unsigned int time, unsigned int array_pos, float direction, float degrees);
void stepper_Init(void);
void make_movement(movement_t movement);

/*VARIABLES*/
volatile unsigned int steppersDone;
stepper_t steppersArray[2];
volatile uint32_t arraylength;
movement_t movementArray1[2][3];			// First position corresponds to certain stepper, second to movement



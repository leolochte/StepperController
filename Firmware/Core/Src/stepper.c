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

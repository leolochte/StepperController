/*
 * stepper.c
 *
 *  Created on: 05.04.2021
 *      Author: Leo
 */

/*INCLUDES*/
#include "stepper.h"
#include "main.h"

/*FUNCTIONS*/
void moveSteppers_micro_sync(unsigned int time) {
	for (int i = 0; i < sizeof(steppersArray)/sizeof(steppersArray[0]) ;++i) {
		steppersArray[i].currentstep = 0;
		steppersArray[i].counter = 0;

		steppersArray[i].mindelay = 400*0.66*time/steppersArray[i].steps;
		HAL_GPIO_WritePin(steppersArray[i].DIRGPIOPORT, steppersArray[i].DIRGPIOPIN, steppersArray[i].direction);
	}
	HAL_TIM_Base_Start_IT(&htim6);
}

/*
 * motor.h
 *
 *  Created on: Mar 5, 2025
 *      Author: marou
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "stm32f0xx.h"

#define FALSE	0
#define TRUE	1

/* MOTOR driver functions */
void MOTOR_Enable_Init  (void);
void MOTOR_Direction_Set   (uint8_t enable);
void MOTOR_Cmd_Update   (int16_t cmd);
uint32_t abs(uint32_t integer);

#endif /* INC_MOTOR_H_ */

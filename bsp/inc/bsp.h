/*
 * bsp.h

 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f0xx.h"

/*
 * LED driver functions
 */
void	BSP_LED_Init	(void);
void	BSP_LED_On	(void);
void	BSP_LED_Off	(void);
void	BSP_LED_Toggle	(void);
/*
 * Push-Button driver functions
 */
void       BSP_PB_Init		(void);
uint8_t    BSP_PB_GetState	(void);
/*
 * ADC functions
 */
void BSP_ADC_Init		(void);
/*
 * Debug Console init
 */
void	BSP_Console_Init	(void);
/* Timer functions */
void    BSP_TIMER_Timebase_Init (uint16_t period_ms);
void    BSP_TIMER_IC_Init       (void);
void    BSP_TIMER_PWM_Init      (void);
/*math functions */
uint32_t abs_diff(uint32_t a, uint32_t b);
/* GPS functions */
void BSP_GPS_Init(void);
char BSP_GPS_ReadChar(void);
void BSP_GPS_ReadLine(char *buffer, int max_len);
#endif /* BSP_INC_BSP_H_ */



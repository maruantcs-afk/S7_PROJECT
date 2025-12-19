/*
 * motor.c
 *
 *  Created on: Mar 5, 2025
 *      Author: marou
 */


#include "motor.h"

void MOTOR_Enable_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//configure PA6 as output
	GPIOA->MODER &= ~(GPIO_MODER_MODER6_Msk);
	GPIOA->MODER |=  (0x01 <<GPIO_MODER_MODER6_Pos);
	// Configure PA6 as Push-Pull output
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_6);
	// Configure PA6 as High-Speed Output
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6_Msk );
	GPIOA->OSPEEDR |=  (0x03 <<GPIO_OSPEEDR_OSPEEDR6_Pos);
	// Disable PA6 Pull-up/Pull-down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk);
	// 2. Configurer PA5 comme sortie (01 dans MODER)
	GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk);
	GPIOA->MODER |=  (0x01 << GPIO_MODER_MODER5_Pos);

	// 3. Configurer PA5 comme sortie Push-Pull (0 dans OTYPER)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5);

	// 4. Configurer PA5 en Haute Vitesse (11 dans OSPEEDR)
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR5_Msk);
	GPIOA->OSPEEDR |=  (0x03 << GPIO_OSPEEDR_OSPEEDR5_Pos);

	// 5. Désactiver les résistances de Pull-up/Pull-down (00 dans PUPDR)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5_Msk);
	GPIOA->BSRR |= GPIO_BSRR_BS_5;


}




/*
 * MOTOR_Enable_Set()
 * Enable (TRUE) or Disable (FALSE) the power stage
 */

void MOTOR_Direction_Set(uint8_t enable)
{
	switch(enable)
	{
		case TRUE:
		{
			// Disable Motor Control
			GPIOA->BSRR |= GPIO_BSRR_BS_6;
			break;
		}

		case FALSE:
		{
			// Enable Motor Control
			GPIOA->BSRR |= GPIO_BSRR_BR_6;
			break;
		}
	}
}

/*void MOTOR_Enable_Set(uint8_t enable)
{
    switch(enable)
    {
        case FALSE:
            // Désactiver le contrôle du moteur : mettre PA6 à l'état bas
            GPIOB->BSRR = GPIO_BSRR_BR_3;
            break;
        case TRUE:
            // Activer le contrôle du moteur : mettre PA6 à l'état haut
            GPIOB->BSRR = GPIO_BSRR_BS_3;
            break;
        default:
            break;
    }
}
*/


/*
 * MOTOR_Cmd_Update
 * Update PWM1 & PWM2 duty-cycles driving the motor
 *
 * cmd should be between -1000 and +1000 but the function implements clamps
 * to avoid overloading PWMs.
 *
 * cmd > 0 drives motor CCW (trigo)
 * cmd < 0 drives motor CW  (inverse trigo)
 *
 * For small cmd values, motor will not turn because of friction forces (dead zone)
 */

// Exemple corrigé pour une seule sortie PWM + direction
/*void MOTOR_Cmd_Update(int16_t cmd) {
	if (cmd >  1000) cmd =  1000;
	if (cmd < -1000) cmd = -1000;

    if (cmd >= 0) {
        TIM2->CCR2 = cmd;               // Appliquer le PWM sur D3
        MOTOR_Direction_Set(TRUE);
    }
    else
    {
    	TIM2->CCR2 = -cmd;              // Appliquer le PWM sur D3
        MOTOR_Direction_Set(FALSE);
    }
}
*/
void MOTOR_Cmd_Update(int16_t cmd) {
    // 1. Clamp de la commande entre -1000 et 1000
    if (cmd >  1000) cmd =  1000;
    if (cmd < -1000) cmd = -1000;

    // 2. Calcul du PWM pour l'ESCON (Plage 10% - 90%)
    // On transforme 0-1000 en 100-900 (pour ARR=1000)
    uint16_t pwm_value = 100 + (abs(cmd) * 800 / 1000);

    if (cmd >= 0) {
        TIM2->CCR2 = pwm_value;
        MOTOR_Direction_Set(TRUE);  // PA6 -> High
    } else {
        TIM2->CCR2 = pwm_value;
        MOTOR_Direction_Set(FALSE); // PA6 -> Low
    }
}

void MOTOR_Brake_Init(void)
{
    // Par exemple, si D9 est connecté sur GPIOC, pin 7
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                      // Activer l'horloge de GPIOC
    GPIOC->MODER &= ~(GPIO_MODER_MODER7_Msk);
    GPIOC->MODER |= (0x01 << GPIO_MODER_MODER7_Pos);         // Configurer PC7 en sortie
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);                    // Push-pull
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_Msk);                // Pas de pull-up/pull-down
    // Par défaut, on désactive le frein (mettre le frein à LOW)
    GPIOC->BSRR = GPIO_BSRR_BR_7;
}





uint32_t abs(uint32_t integer)
{
	if (integer >= 0)
	{
		return integer;
	}

	else
	{
		return -integer;
	}
}

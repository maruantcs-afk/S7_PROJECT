/*
 * main.c
 *
 * Created on: Feb 6, 2025
 * Author: marou
 */

#include "stm32f0xx.h"
#include "main.h"
#include "bsp.h"
#include "motor.h"
#include "delay.h"

// Prototype de ta fonction my_printf pour pouvoir l'utiliser ici
int my_printf(const char *format, ...);

#define CONVERSION_FACTOR (uint32_t)37 // in /m
#define COEFF_0 (uint32_t)7 //l'ADC renvoie 0 pour 7 mm

static void SystemClock_Config(void);
uint8_t 	timebase_irq;
uint16_t extensionLength;
int32_t		 eps ;

int main(void)
{
	uint32_t consigne = 30; // en mm

    // Configuration de l'horloge système
    SystemClock_Config();
    while(SysTick_Config(SystemCoreClock/10) != 0);

    // Initialisation de la console de debug
    BSP_Console_Init();
    my_printf("Console ready!\r\n");

    // Initialisation de l'ADC
    BSP_ADC_Init();
    my_printf("ADC ready!\r\n");

    // --- INITIALISATION GPS ---
    BSP_GPS_Init();
    my_printf("GPS Init OK\r\n");
    char gps_buffer[128]; // Tableau pour stocker la ligne lue
    // --------------------------

    // Initialisation moteur
    my_printf("Enabling motors\r\n");
    MOTOR_Enable_Init();
    BSP_DELAY_ms(50);

    // Initialisation PWM
    my_printf("Staring PWM\r\n");
    BSP_TIMER_PWM_Init();
    BSP_DELAY_ms(50);

	// Enable sampling timer interrupts
	NVIC_SetPriority(TIM6_DAC_IRQn, 1);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	while (1)
	{
		// 1. Lire une ligne du GPS
		BSP_GPS_ReadLine(gps_buffer, 128);

		// 2. Vérifier si c'est une ligne de position ($GPGGA)
		// On regarde "manuellement" les caractères 3, 4 et 5
		if (gps_buffer[3] == 'G' && gps_buffer[4] == 'G' && gps_buffer[5] == 'A')
		{
			// 3. Affichage my_printf
			// On affiche le buffer comme une string (%s)
			my_printf("Donnee GPS: %s\r\n", gps_buffer);
		}

		/* test moteur */
		MOTOR_Cmd_Update(100*consigne);
		BSP_DELAY_ms(1000);

		MOTOR_Cmd_Update(-100*consigne);
		BSP_DELAY_ms(1000);
	}
}

/*
 * Clock configuration for the Nucleo STM32F072RB board
 * HSE input Bypass Mode            -> 8MHz
 * SYSCLK, AHB, APB1                -> 48MHz
 * PA8 as MCO with /16 prescaler    -> 3MHz
 */
static void SystemClock_Config()
{
	uint32_t	HSE_Status;
	uint32_t	PLL_Status;
	uint32_t	SW_Status;
	uint32_t	timeout = 0;
	timeout = 1000000;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	do
	{
		HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((HSE_Status == 0) && (timeout > 0));

	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;
	// Wait until PLL is ready
	do
	{
		PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((PLL_Status == 0) && (timeout > 0));

	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	do
	{
		SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
}

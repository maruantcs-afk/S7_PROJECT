/*
 * bsp.c

 */

#include "bsp.h"

/*
 * BSP_LED_Init()
 * Initialize LED pin (PA5) as a High-Speed Push-Pull output
 * Set LED initial state to OFF
 */


void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR |= GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_On()
 * Turn ON LED on PA5
 */

void BSP_LED_On()
{
	GPIOA->BSRR = GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}


/*
 * BSP_Console_Init()
 * USART2 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PA2 (AF1)
 * RX -> PA3 (AF1)
 */

void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);

	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Baud Rate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Baud Rate = 115246.0984 -> 0.04% error (better)
	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 833;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}




/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t BSP_PB_GetState()
{
	uint8_t state;
	if ((GPIOC->IDR & GPIO_IDR_13) == GPIO_IDR_13)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}
	return state;
}


/*
 * ADC_Init()
 * Initialize ADC for a single channel conversion
 * on channel 11 -> pin PC1
 */

void BSP_ADC_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure pin PC1 as analog
	GPIOC->MODER &= ~GPIO_MODER_MODER1_Msk;
	GPIOC->MODER |= (0x03 <<GPIO_MODER_MODER1_Pos);

	// Enable ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Reset ADC configuration
	ADC1->CR 	= 0x00000000;
	ADC1->CFGR1  = 0x00000000;
	ADC1->CFGR2  = 0x00000000;
	ADC1->CHSELR = 0x00000000;

	// Enable continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;

	// 12-bit resolution
	ADC1->CFGR1 |= (0x00 <<ADC_CFGR1_RES_Pos);

	// Select PCLK/2 as ADC clock
	ADC1->CFGR2 |= (0x01 <<ADC_CFGR2_CKMODE_Pos);

	// Set sampling time to 28.5 ADC clock cycles
	ADC1->SMPR = 0x03;

	// Select channel 11
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;

	// Start conversion
	ADC1->CR |= ADC_CR_ADSTART;
}

/*

void BSP_TIMER_PWM_Init(void) {
    // Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB3 as Alternate Function (AF2 = TIM2_CH2)
    GPIOB->MODER &= ~GPIO_MODER_MODER3_Msk;          // Clear mode bits
    GPIOB->MODER |=  (0x02 << GPIO_MODER_MODER3_Pos); // Alternate Function mode
    GPIOB->AFR[0] &= ~(0xF << (4 * 3));             // Clear AF bits for PB3 (pin 3)
    GPIOB->AFR[0] |=  (0x2 << (4 * 3));             // Set AF2 (TIM2_CH2)

    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Reset TIM2 configuration
    TIM2->CR1  = 0x0000;    // Disable timer
    TIM2->CR2  = 0x0000;
    TIM2->CCER = 0x0000;    // Disable all capture/compare

    // Configure TIM2
    TIM2->PSC = 4 - 1;      // 48 MHz → 12 MHz (prescaler = 4)
    TIM2->ARR = 1000;       // PWM period = 1000 ticks → 12 kHz
    TIM2->CR1 |= TIM_CR1_ARPE; // Enable auto-reload preload

    // PWM Mode 1 for Channel 2
    TIM2->CCMR1 |= (0x06 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // PWM mode 1 + preload
    TIM2->CCR2 = 0;       // Initial duty cycle = 50% (500/1000)
    TIM2->CCER |= TIM_CCER_CC2E; // Enable PWM output

    TIM2->CR1 |= TIM_CR1_CEN; // Start TIM2
}*/
void BSP_TIMER_PWM_Init(void) {
    // 1. Horloge GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // 2. Configuration PB3 (D3) en AF2 (TIM2_CH2)
    GPIOB->MODER &= ~GPIO_MODER_MODER3_Msk;
    GPIOB->MODER |= (0x02 << GPIO_MODER_MODER3_Pos);
    GPIOB->AFR[0] &= ~(0xF << (4 * 3));
    GPIOB->AFR[0] |= (0x2 << (4 * 3)); // AF2

    // 3. Horloge TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 4. Configuration Fréquence PWM = 1 kHz
    // Fck (48MHz) / 48 = 1 MHz
    TIM2->PSC = 48 - 1;
    // 1 MHz / 1000 = 1000 Hz (1 kHz)
    TIM2->ARR = 1000;

    TIM2->CR1 |= TIM_CR1_ARPE;

    // 5. PWM Mode 1 sur CH2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
    TIM2->CCMR1 |= (0x06 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    // Duty cycle initial à 0%
    TIM2->CCR2 = 0;

    // Activer la sortie et le compteur
    TIM2->CCER |= TIM_CCER_CC2E;
    TIM2->CR1 |= TIM_CR1_CEN;
}


uint32_t abs_diff(uint32_t a, uint32_t b) {
    return (a > b) ? (a - b) : (b - a);
}


/*
 * BSP_GPS_Init()
 * USART1 @ 9600 Baud (Standard Adafruit GPS)
 * 1 start - 8-bit - 1 stop
 * TX -> PA9  (AF1)
 * RX -> PA10 (AF1)
 */
void BSP_GPS_Init()
{
    // 1. Enable GPIOA clock (si ce n'est pas déjà fait ailleurs)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // 2. Configure PA9 (TX) and PA10 (RX) as Alternate Function
    // On nettoie les bits pour PA9 et PA10
    GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
    // On met '10' (Alternate Function) pour PA9 et PA10
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODER9_Pos) | (0x02 << GPIO_MODER_MODER10_Pos);

    // 3. Set PA9 and PA10 to AF1 (USART1)
    // Attention: PA9 et PA10 sont dans le registre AFR[1] (High Register) car pin > 7
    // Pin 9 est sur les bits 4-7, Pin 10 sur les bits 8-11
    GPIOA->AFR[1] &= ~(0x00000FF0); // Clear bits for pin 9 and 10
    GPIOA->AFR[1] |=  (0x00000110); // Set AF1 (0001) for pin 9 and 10

    // 4. Enable USART1 clock
    // Note: USART1 est souvent sur APB2 sur le F0
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 5. Clear USART1 configuration (Reset state)
    USART1->CR1 = 0x00000000;
    USART1->CR2 = 0x00000000;
    USART1->CR3 = 0x00000000;

    // 6. Configure Baud Rate to 9600
    // Clock source PCLK = 48 MHz (comme dans votre BSP_Console)
    // Baud Rate = 9600
    // USARTDIV = 48,000,000 / 9600 = 5000
    USART1->BRR = 5000;

    // 7. Enable Transmitter and Receiver
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // 8. Enable USART1
    USART1->CR1 |= USART_CR1_UE;
}

/*
 * BSP_GPS_ReadChar()
 * Blocking function that waits for a character from GPS
 */
char BSP_GPS_ReadChar()
{
    // Wait until RXNE (Read Data Register Not Empty) bit is set
    // ISR: Interrupt and Status Register
    while ((USART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE)
    {
        // On attend (Busy wait)
    }

    // Read data from RDR (Receive Data Register)
    // Reading clears the RXNE flag automatically
    return (char)(USART1->RDR);
}

/*
 * BSP_GPS_ReadLine()
 * Fills the buffer with a full NMEA line (starts with '$', ends with '\n')
 * Blocking function.
 */
void BSP_GPS_ReadLine(char *buffer, int max_len)
{
    char c;
    int index = 0;

    // 1. Wait for start of sentence '$'
    // On ignore tout tant qu'on n'a pas vu le début d'une trame
    do {
        c = BSP_GPS_ReadChar();
    } while (c != '$');

    // On stocke le '$' au début
    buffer[index++] = c;

    // 2. Read until end of line ('\n')
    while (index < max_len - 1)
    {
        c = BSP_GPS_ReadChar();

        // Si on reçoit \r (Carriage Return), on l'ignore souvent ou on le garde,
        // mais la fin réelle est \n. Ici on garde tout sauf \n pour fermer la chaine.
        if (c == '\n')
        {
            break; // Fin de la ligne
        }

        buffer[index++] = c;
    }

    // 3. Null-terminate string
    buffer[index] = '\0';
}



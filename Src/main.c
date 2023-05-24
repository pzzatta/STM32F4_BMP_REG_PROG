/*
 *********************************************************************************************************************************************
 * File             : main.c
 * Board            : STM32F407 DISCO Board
 * MCU              : STM32F407VG,
 * IDE              : STM32CUBEIDE Version 1.8.0
 * Datasheet        : Arm Cortex-M4 32b MCU+FPU, 168MHz
 * Author           : Philip Zatta, PhD, Software Consultant, Embedded Software, LabView Software Consultant
 * Company          : EMBEDDED SOFTWARE AND SYSTEMS LLC
 * Description      : Set CPU maximum frequency to 168 MHz using HSE or HSI clock as PLL source (Board does not have an external crystal oscillator)
 *                    Output HSI clock on MCO1 (PA8) and System Clock on MCO2 (PC9)
 * Operating System : Microsoft Windows 11
 * Date             : May 19, 2022
 *******************************************************************************************************************************************
 *
 * Comments/Explanations:
 *
 * Bare metal programming of ARM Cortex M4, ST STM32F407 Discovery Board.
 *  - Set CPU clock frequency to maximum frequency of 100 MHz.
    - Output HSI and System Clock sources onto MCO1 (PA8) and MCO2 (PC9), respectively.
 *
 * Corresponding laboratory activities: Lab00_measurement_cpu_frequency_max
 *  - Display HSE and SYSCLK on an oscilloscope to verify right values of clocks.
 *  - Monitor clock related parameters from RCC registers and display them on PC via UART/RS232 port
 *  - Monitor clock related parameters from RCC registers and display them on LCD display.
 *
 *   After system clock initialization
 *
 *   Note: HSE clock is preferably selected as the source clock for the PLL clock.
 *   However, HSI clock will be selected here because the STM32F411 Nucleo board does not have an
 *   external crystal oscillator as HSE clock. They will not be a need for system clock initialization.
 *
 *  Programming steps or sub-steps for setting GPIOA pin PA8 as MCO1 and GPIOC PC9 pin as MCO2.
 *    Step 1: Enable clock access to GPIOA and GPIOC using RCC_AHB1ENR register.
 *    Step 2: Set GPIOA PA8 and GPIOC PC9 pins to alternate function mode using GPIOx_MODER register.
 *    Step 3: Set GPIOA PA8 and GPIOC PC9 pins to alternate function of type system  using GPIOx_AFR register (MCO  = system function), AF = 0000.
 *    Step 4: Select HSI as clock source for MCO1 (PA8) and set the prescaler using RCC_CFGR register.
 *    Step 5: Select System Clock as clock source for MCO2 (PC9) and set the prescaler using RCC_CFGR register.
 *
 *
 *
 * Programming steps or sub-steps for setting :
 *   1- Enable HSI clock and wait for HSI clock to be ready using RCC_CR register.
 *   2- Set the Power Enable Clock using RCC_APB1ENR register and set voltage regulator using PWR_CR register.
 *   3- Set the Flash Latency, Prefetch, Instruction Cache Enable and Data Cache Enable parameters using FLASH_ACR register.
 *      For Latency and Number of  CPU Wait Cycles, see Table 10 in Reference Manual RM 0090 version 9, Page 80. It should be 3 for 100 MHz
 *   4- Select HSI as the source of the PLL using RCC_PLLCFG.
 *   5- Set the pre-scalers for AHB1, APB1 and APB2.
 *   6- Configure the PLL parameters (PLL_M, PLL_N, PLL_P, PLL_Q).
 *   7- Select the PLL as the system clock source and wait for PLL clock to be ready.
 ********************************************************************************************************************************************
 */



/******************************************************** PREPROCESSOR DIRECTIVES **********************************************************/
#include "stm32f4xx.h"
#include <stdint.h>



/*************************************** MACROS *****************************************************************************************************/

// Setting GPIOA PA8 and GPIOC PC9 pins to MCO1 and MCO2, respectively.
#define   GPIOAEN                  (   1UL  << 0   )    // Enable clock access to GPIOA using RCC_AHB1ENR
#define   GPIOCEN                  (   1UL  << 2   )    // Enable clock access to GPIOC using RCC_AHB1ENR
#define   MODER8_0                 (   1UL  << 16  )    // Bit 16 must be
#define   MODER8_1                 (   1UL  << 17  )    // Bit 16 must be 1 to set GPIOC PC9 to alternate function mode
#define   MODER9_0                 (   1UL  << 18  )    // Bit 18 must be
#define   MODER9_1                 (   1UL  << 19  )    // Bit 19 must be 1 to set GPIOC PC9 to alternate function mode
#define   AFRH8_0                  (   1UL  << 0   )    // Clear bit 0
#define   AFRH8_1                  (   1UL  << 1   )    // Clear bit 1
#define   AFRH8_2                  (   1UL  << 2   )    // Clear bit 2
#define   AFRH8_3                  (   1UL  << 3   )    // Clear bit 3 for AF0 = 0000 for system function
#define   AFRH9_0                  (   1UL  << 4   )    // Clear bit 0
#define   AFRH9_1                  (   1UL  << 5   )    // Clear bit 1
#define   AFRH9_2                  (   1UL  << 6   )    // Clear bit 2
#define   AFRH9_3                  (   1UL  << 7   )    // Clear bit 3 for AF0 = 0000 for system function
#define   MCO1_0                   (   1UL  << 21  )   // Clear bit 0 and
#define   MCO1_1                   (   1UL  << 22  )   // Clear bit 1 for selecting HSI clock into RCC_CFGR register
#define   MCO2_0                   (   1UL  << 30  )   // Clear bit 0 and
#define   MCO2_1                   (   1UL  << 31  )   // Clear bit 1 for selecting HSI clock into RCC_CFGR register
#define   MCO1_PRE_0               (   1UL  << 24  )   // Clear bit 0 for selecting HSI clock into RCC_CFGR register
#define   MCO1_PRE_1               (   1UL  << 25  )   // Clear bit 1 for selecting HSI clock into RCC_CFGR register
#define   MCO1_PRE_2               (   1UL  << 26  )   // Clear bit 2 for selecting HSI clock into RCC_CFGR register
#define   MCO2_PRE_0               (   1UL  << 27  )   // Clear bit 0 for selecting System clock into RCC_CFGR register
#define   MCO2_PRE_1               (   1UL  << 28  )   // Clear bit 0 for selecting System clock into RCC_CFGR register
#define   MCO2_PRE_2               (   1UL  << 29  )   // Clear bit 0 for selecting System clock into RCC_CFGR register
#define   MCO2_DIV                 (  0b100 << 27  )   // 100: division by 2, 101: division by 3, 110: division by 4, 111: division by 5

// Setting CPU frequency to maximum frequency of 100 MHz.
#define  HSEON                     (   1UL  << 16  ) // Bit   0    of RCC_CR register to set
#define  HSERDY                    (   1UL  << 17  ) // Bit   1    of RCC_CR register to check

#define  PWREN                     (   1UL  << 28  ) // Bit  28    of RCC_APB1RSTR register to set to enable POWER RESET clock
#define  VOS_0                     (   1UL  << 14  ) // Bit  14    of PWR_CR register for set value or scale mode 1
#define  VOS_1                     (   1UL  << 15  ) // Bit  14    of PWR_CR register for set value or scale mode 1, VOS[1:0] = 0x11 for HCLK = 100 MHz

#define  LATENCY_0                 (   1UL  << 0   ) // Bit   0    of FLASH_ACR register to set for 3 WS (=4 CPU cycles) at 100 MHz, Table 10 RM0090 Page 80)
#define  LATENCY_1                 (   1UL  << 1   ) // Bit   1    of FLASH_ACR register to set for 3 WS (=4 CPU cycles) at 100 MHz, Table 10 RM0090 Page 80)
#define  LATENCY_2                 (   1UL  << 2   ) // Bit   2    of FLASH_ACR register to clear for 3 WS (=4 CPU cycles) at 100 MHz, Table 10 RM0090 Page 80)
#define  LATENCY_3                 (   1UL  << 3   ) // Bit   3    of FLASH_ACR register to clear for 3 WS (=4 CPU cycles) at 100 MHz, Table 5 RM0383 Rev3 Page 45)
#define  PRFTEN                    (   1UL  << 8   ) // Bit   8    of FLASH_ACR register to set
#define  ICEN                      (   1UL  << 9   ) // Bit   9    of FLASH_ACR register to set
#define  DCEN                      (   1UL  << 10  ) // Bit  10    of FLASH_ACR register to set
#define  RESET_CFGR                (       0x00    ) // Bit  10    of FLASH_ACR register to set

#define  PLLSRC                    (   1UL  << 22  ) // Bit  22    of RCC_PLLCFGR register to clear to select HSI as PLL source
#define  HPRE_0                    (   1UL  << 4   ) // Bit   4    of RCC_CFGR register (0XXX), to clear bit 7, others do not matter
#define  HPRE_1                    (   1UL  << 5   ) // Bits 7:4   of RCC_CFGR register (0XXX), to clear bit 7, others do not matter
#define  HPRE_2                    (   1UL  << 6   ) // Bits 7:4   of RCC_CFGR register (0XXX), to clear bit 7, others do not matter
#define  HPRE_3                    (   1UL  << 7   ) // Bits 7:4   of RCC_CFGR register (0XXX), to clear bit 7, others do not matter
#define  PPRE1_0                   (   1UL  << 10  ) // BitS 12:10 of RCC_CFGR register to set for Div = 4
#define  PPRE1_1                   (   1UL  << 11  ) // BitS 12:10 of RCC_CFGR register to set for Div = 4
#define  PPRE1_2                   (   1UL  << 12  ) // BitS 12:10 of RCC_CFGR register to set for Div = 4
#define  PPRE2_0                   (   1UL  << 13  ) // BitS 15:13 of RCC_CFGR register (100) to set for Div = 2
#define  PPRE2_1                   (   1UL  << 14  ) // BitS 15:13 of RCC_CFGR register (100) to set for Div = 2
#define  PPRE2_2                   (   1UL  << 15  ) // BitS 15:13 of RCC_CFGR register (100) to set for Div = 2
#define  PLLM_0                    (   1UL  << 0   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM_1                    (   1UL  << 1   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM_2                    (   1UL  << 2   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM_3                    (   1UL  << 3   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM_4                    (   1UL  << 4   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM_5                    (   1UL  << 5   ) // BitS 5:0   of RCC_PLLCFGR register to set to divide HSI 16MHz frequency by 8 (VCO input freq = 2 MHz)
#define  PLLM                      (       4       )
#define  PLLN_0                    (   1UL  << 6   ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_1                    (   1UL  << 7   ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_2                    (   1UL  << 8   ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_3                    (   1UL  << 9   ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_4                    (   1UL  << 10  ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100#define  PLL_N
#define  PLLN_5                    (   1UL  << 11  ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_6                    (   1UL  << 12  ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_7                    (   1UL  << 13  ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN_8                    (   1UL  << 14  ) // Bits 14:6  of RCC_PLLCFGR register to set for PLL_N = 100
#define  PLLN                      (     168       )
#define  PLLP_0                    (   1UL  << 16  ) // BitS 14:6  of RCC_PLLCFGR register to clear for Div =2 (PLL_P = 2)
#define  PLLP_1                    (   1UL  << 17  ) // BitS 14:6  of RCC_PLLCFGR register to clear for Div =2 (PLL_P = 2)
#define  PLL_P                     (        4      )
#define  PLLON                     (   1UL  << 24  ) // Bit  24    of RCC_CR register to enable PLL source
#define  PLLRDY                    (   1UL  << 25  ) // Bit  25    of RCC_CR register to check that PLL source is ready
#define  PLLSW0                    (   1UL  << 0   ) // Bit   0    of RCC_CFGR register  to clear  (SW0 = 0)
#define  PLLSW1                    (   1UL  << 1   ) // Bit   1    of RCC_CFGR register  to set    (SW1 =1 ) so that SW1 SW0 =10 select PLL as system clock
#define  PLLSWS1                   (   1UL  << 3   ) // Bit   3    of RCC_CFGR register  to check if changed from 0 to 1



/******************************************************* FUNCTION PROTOTYPES ******************************************************/
void InitMCOGPIOPins(void);
void SetSystemClock(void);




/******************************************************** GLOBAL VARIABLES *******************************************************/



int main()
{

	    /************************************************* LOCAL VARIABLES *****************************************************/




	  /************************************************ INITIALIZATION SECTION ************************************************/

	   InitMCOGPIOPins();
	   SetSystemClock();

		while(1)
		{


		}
}


/********************************************************* FUNCTION DEFINITIONS ***************************************************/

 void InitMCOGPIOPins(void)
 {
	 // Step 1: Enable clock access to GPIOA and GPIOC using RCC_AHB1ENR register.
	 RCC->AHB1ENR |= GPIOAEN | GPIOCEN;  // Enable clock access to GPIOA and GPIOC

	 // Step 2: Set GPIOA PA8 and GPIOC PC9 pins to alternate function mode using GPIOx_MODER register.
	 GPIOA->MODER &= ~MODER8_0;
	 GPIOA->MODER |=  MODER8_1;               // Setting 10 for PA8 in alternate function mode
	 GPIOC->MODER &= ~MODER9_0;
	 GPIOC->MODER |=  MODER9_1;              // Setting 10 for PC9 in alternate function mode
	 GPIOC->OTYPER = 0x00000000;             /*  push-pull */
	 GPIOC->OSPEEDR |= 1  <<  19 ;   ;       /* Fast  speed */
	 GPIOC->OSPEEDR &= ~  (1   <<  18 );

	 // Step 3: Set GPIOA PA8 and GPIOC PC9 pins to alternate function of type system  using GPIOx_AFR register (MCO  = system function), AF = 0000.
	 GPIOA->AFR[1] &= ~( AFRH8_0 | AFRH8_1 | AFRH8_2 | AFRH8_3 );   // Setting GPIOA PA8 in alternate function of type MCO1 which is a system function
	 GPIOC->AFR[1] &= ~( AFRH9_0 | AFRH9_1 | AFRH9_2 | AFRH9_3 );   // Setting GPIOC PC8 in alternate function of type MCO2 which is a system function
 }


 void SetSystemClock(void)
 {

	 // Step 1- Enable HSE clock and wait for HSE clock to be ready using RCC_CR register.
	     RCC->CR |=  HSEON;
		 while (! (RCC->CR & HSERDY ) ) {}



	 // Step 2- Set the Power Enable Clock using RCC_APB1ENR register and set voltage regulator using PWR_CR register.
		 RCC->APB1ENR |= PWREN;
		 PWR->CR |= ( VOS_1 | VOS_0 );  // Set 11 for HCKL = 100 MHz

	 // Step 3- Set the Flash Latency, Prefetch, Instruction Cache Enable and Data Cache Enable parameters using FLASH_ACR register.

		 FLASH->ACR |= ( LATENCY_2 | LATENCY_0 );   // Set bits 0 and 1 to have 11 (3 Wait states or 5 CPU cycles)
		 FLASH->ACR &= ~ ( LATENCY_3 | LATENCY_1 ); // Clear bits 2 and 3 to have 11 (3 Wait states or 4 CPU cycles)
		 FLASH->ACR |= ( DCEN |  ICEN | PRFTEN );

		 //FLASH -> ACR = FLASH_ACR_LATENCY_5WS| FLASH_ACR_PRFTEN  | FLASH_ACR_PRFTEN  | FLASH_ACR_ICEN  |  FLASH_ACR_DCEN ;
		 /* 6 CPU cycle wait */
		 /* enable prefetch */
		 /* instruction cache enable */
		 /* data cache enable */


	   // Step 4- Set the pre-scalers for AHB1, APB1 and APB2.
		 /* Configure clocks
		 * Max SYSCLK: 168MHz
		 * Max AHB: SYSCLK
		 * Max APB1: SYSCLK/4 = 48MHz
		 * * Max APB2: SYSCLK/2 = 86MHz
         *  enable sys clock output 2 with clock divider = 4 */

		 RCC-> CFGR = RESET_CFGR ; //Resetting RCC_CFGR allows to set MCO2 output to System Clock and MCO1 to HSI clock
		 RCC-> CFGR |= ( MCO2_DIV   | RCC_CFGR_PPRE2_DIV2  | RCC_CFGR_PPRE1_DIV4  ) ;

		 /* Clock output 2 MCO2 is SYSCLK (RCC_CFGR_MCO2). It is divided by
		  * 100: division by 2
            101: division by 3
            110: division by 4
            111: division by 5
		  *
		  * 4 */

		 /* Clock output divider */

		 /* APB2 prescaler */

		 /* APB2 prescaler */

    // Step 5- Configure the PLL parameters (Source, PLL_M, PLL_N, PLL_P).

		 RCC -> PLLCFGR = PLLSRC | ( PLLM << 0 ) | ( PLLN << 6 );
		 RCC -> PLLCFGR &= ~( PLLP_1 | PLLP_0 );

		 /* PLL source */

		 /* PLL input division */

		 /* PLL multiplication */

		 /* PLL sys clock division */

		 /* PLL usb clock division =48MHz */
		 /* crystal: 8MHz
		 * PLL in: 2MHz (div 4)
		 * PLL loop: 336MHz (mul 168)
		 * PLL out: 168MHz (div 2)
		 * PLL usb: 48MHz (div 7)
		 */


	// Step 6- Enable the PLL and wait for PLL clock to be ready using RCC_CR register.
		 RCC->CR |= PLLON;
		 while ( ! (RCC->CR & PLLRDY ) ) {}

     //Step 7-Select the PLL as the system clock source and wait for PLL clock to be ready using RCC_CFGR register

		 /* select system clock */
		 RCC->CFGR |= PLLSW1;
		 RCC->CFGR &= ~ PLLSW0;
		 while ( ! (RCC->CR & PLLSWS1) ) {}


	 /*   My code is not working

	 // Step 1- Enable HSI clock and wait for HSI clock to be ready using RCC_CR register.
	 RCC->CR |=  HSEON;
	 while (! (RCC->CR & HSERDY ) ) {}

	 // Step 2- Set the Power Enable Clock using RCC_APB1ENR register and set voltage regulator using PWR_CR register.
	 RCC->APB1ENR |= PWREN;
	 PWR->CR |= ( VOS_1 | VOS_0 );  // Set 11 for HCKL = 100 MHz

	 // Step 3- Set the Flash Latency, Prefetch, Instruction Cache Enable and Data Cache Enable parameters using FLASH_ACR register.

	 FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
	 //FLASH->ACR |= ( LATENCY_2 | LATENCY_0 );   // Set bits 0 and 1 to have 11 (3 Wait states or 4 CPU cycles)
	 //FLASH->ACR &= ~ ( LATENCY_3 | LATENCY_1 ); // Clear bits 2 and 3 to have 11 (3 Wait states or 4 CPU cycles)
	 //FLASH->ACR |= ( DCEN |  ICEN | PRFTEN );

	 // Step 4- Set the pre-scalers for AHB1, APB1 and APB2.
     // AHB1 Clock, No division, HPRE, 0xxx or 0000
	 //RCC->CFGR &= ~ ( HPRE_3  | HPRE_2  | HPRE_1  | HPRE_0 );
	 RCC->CFGR = 0x00;

	 // APB1 Clock, PPRE1, 101 for division by 4 (Reference Manual RM0383 Rev3, Page 107)
	 //RCC->CFGR &= ~ PPRE1_1;
	 RCC->CFGR |= ( PPRE1_2  | PPRE1_0);

	 // APB2 Clock, PPRE2, 100: AHB clock divided by 2  (RM0383 Rev3, Page 107)
	 RCC->CFGR &= ~ ( PPRE2_1  | PPRE2_0);;
     RCC->CFGR |=  PPRE2_2;

     // Step 5- Configure the PLL parameters (Source, PLL_M, PLL_N, PLL_P).

     // Disable PLL and PLLI2S before writing bit 22 of RCC_PLLCFGR (Reference Manual RM0383, Rev3, Page 104)
     //RCC->CR &= ~ (1 << 24);  // PLL disabled
     //RCC->CR &= ~ (1 << 26);  // PLLI2S disabled

     // PLL Source
     RCC->PLLCFGR |=  PLLSRC;  // Select HSE clock as PLL source clock

     // PLLM =8 , division is 8 to get max frequency of 2 MHz on VCO
     //RCC->PLLCFGR &= ~ ( PLLM_5 | PLLM_4  | PLLM_2   | PLLM_1  | PLLM_0);
     //RCC->PLLCFGR |=  PLLM_3;
     RCC->PLLCFGR |= 4 << 0;

     // PLLN = 100, 001100100: in Reference Manual RM0383 Rev3, Page 105
     RCC->PLLCFGR |= ( 168 << 6 );

     // PLLP = 00 for division by 2 to get 100 MHz at output of main PLL clock source
      //RCC->PLLCFGR |=  PLLP_1;
      //RCC->PLLCFGR &= ~ PLLP_0;
     RCC->PLLCFGR |= 0 << 16;


     // Step 6- Enable the PLL and wait for PLL clock to be ready using RCC_CR register.
     RCC->CR |= PLLON;
     while ( ! (RCC->CR & PLLRDY ) ) {}

     //Step 7-Select the PLL as the system clock source and wait for PLL clock to be ready using RCC_CFGR register
     RCC->CFGR |= PLLSW1;
     RCC->CFGR &= ~ PLLSW0;
     while ( ! (RCC->CR & PLLSWS1) ) {}

     // Configure MCO1 and MCO2 pins
     // Step 8- Select HSI as clock source for MCO1 (PA8) and set the prescaler using RCC_CFGR register.
     RCC->CFGR &= ~ ( MCO1_0 | MCO1_1 ); // Clearing bits 0 and 1 to select HSI clock to output on MCO1 PA8 pin
     RCC->CFGR &= ~ ( MCO1_PRE_2 |  MCO1_PRE_1 |  MCO1_PRE_0); // clearing bits 0, 1 and 2 for no division

     // Step 9- Select System Clock as clock source for MCO2 (PC9) and set the prescaler using RCC_CFGR register.
     RCC->CFGR &= ~ MCO2_1 ; // Clearing bits 0 and 1 to select System clock to output on MCO2 PC9 pin
     RCC->CFGR &= ~ MCO2_0 ;
     RCC->CFGR |=   6   <<  27;   // Setting 100 for division of system clock by 2
 }

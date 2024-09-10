/*
 * main.c
 *
 *  Created on: 23 mars 2023
 *      Author: cotti
 */

#include "stm32f0xx.h"
#include "bsp.h"
#include "main.h"
#include "delay.h"

// Static functions

static void SystemClock_Config (void);
static uint8_t wait_for_flags();

// Global variables

uint16_t		TIM6_counter;
uint16_t		TIM6_UIF;
//uint16_t		rx_data;

// Main program

/*
 * Project Entry Point
 */

int main(void)
{
    uint8_t 	lps25h_id;
    uint8_t	spi_buffer[5];

    uint32_t	pressure;
    int16_t	temperature;

    // Configure System Clock for 48MHz from 8MHz HSE
    SystemClock_Config();

    // Initialize LED and USER Button
    BSP_LED_Init();
    BSP_PB_Init();

    // Initialize Debug Console
    BSP_Console_Init();
    my_printf("\r\nConsole Ready!\r\n");
    my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

    // Initialize SPI1
    BSP_SPI1_Init();

    // Read LPS25H device ID
    BSP_LPS25H_Read(0x0F, &lps25h_id, 1);
    my_printf("LPS25H Device ID = 0x%02x\r\n", lps25h_id);

    // Turn LPS25H device ON
    BSP_LPS25H_Write(0x20, 0x80);

    // Read back CTRL_REG1 register
    BSP_LPS25H_Read(0x20, spi_buffer, 1);
    my_printf("LPS25H CTRL_REG1 = 0x%02x\r\n", spi_buffer[0]);

    // Set averaging to minimum
    BSP_LPS25H_Write(0x10, 0x00);

    while(1)
    {
	// Trigger a one-shot pressure measure
	BSP_LPS25H_Write(0x21, 0x01);

	// Allow some time for sensor to perform
	BSP_DELAY_ms(10);

	// Read sensor data
	// Pressure level in registers 0x2A | 0x29 | 0x28
	// Temperature    in registers 0x2C | 0x2B
	BSP_LPS25H_Read(0x28, spi_buffer, 5);

	// Compute pressure
	pressure = (uint32_t)( (spi_buffer[2] <<16U) +
                               (spi_buffer[1] << 8U) +
                               (spi_buffer[0] << 0U));
	my_printf("Pressure = %d hPa\r\n", pressure / 4096);

	// Compute temperature
	temperature = (int16_t)((spi_buffer[4] <<8U) + (spi_buffer[3] <<0U));
	my_printf("Temperature = %d C\r\n", (uint8_t)(42.5f + ((float)temperature/480.0f)));

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

/*
 * Wait for & Report I2C status flags
 */

/*
 * Wait for & Report I2C status flags
 */

static uint8_t wait_for_flags()
{
	uint8_t	exit = 0;

	// Clear STOPF and NACKF flags
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->ICR |= I2C_ICR_NACKCF;

	while(exit == 0)
	{
		// TXIS -> PA0
		// Exit when set
		if ((I2C1->ISR & I2C_ISR_TXIS) != 0)
		{
			GPIOA->BSRR = GPIO_BSRR_BS_0;
			exit = 1;
		}
		else GPIOA->BSRR = GPIO_BSRR_BR_0;

		// TC -> PA4
		// Exit when set
		if ((I2C1->ISR & I2C_ISR_TC) != 0)
		{
			GPIOA->BSRR = GPIO_BSRR_BS_4;
			exit = 3;
		}
		else GPIOA->BSRR = GPIO_BSRR_BR_4;

		// STOPF -> PC1
		// Exit when set
		if ((I2C1->ISR & I2C_ISR_STOPF) != 0)
		{
			GPIOC->BSRR = GPIO_BSRR_BS_1;
			exit = 5;
		}
		else GPIOC->BSRR = GPIO_BSRR_BR_1;
	}

	return exit;
}


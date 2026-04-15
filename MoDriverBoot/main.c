/****************************************************************************************
 * Include files
 ****************************************************************************************/
#include "boot.h"                /* bootloader generic header          */
#include "stm32g4xx.h"           /* STM32 CPU and HAL header           */
#include "stm32g4xx_ll_pwr.h"    /* STM32 LL PWR header                */
#include "stm32g4xx_ll_rcc.h"    /* STM32 LL RCC header                */
#include "stm32g4xx_ll_bus.h"    /* STM32 LL BUS header                */
#include "stm32g4xx_ll_system.h" /* STM32 LL SYSTEM header             */
#include "stm32g4xx_ll_utils.h"  /* STM32 LL UTILS header              */
#include "stm32g4xx_ll_gpio.h"   /* STM32 LL GPIO header               */

/****************************************************************************************
 * Function prototypes
 ****************************************************************************************/
static void Init(void);
static void SystemClock_Config(void);
int main(void)
{
	/* initialize the microcontroller */
	Init();
	/* initialize the bootloader */
	BootInit();

	/* start the infinite program loop */
	while (1) {
		/* run the bootloader task */
		BootTask();
	}

	/* program should never get here */
	return 0;
} /*** end of main ***/
static void Init(void)
{
	/* HAL library initialization */
	HAL_Init();
	/* configure system clock */
	SystemClock_Config();
} /*** end of Init ***/
static void SystemClock_Config(void)
{
	/* Set flash latency. */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
		;
	}

	/* Enable boost mode and the HSI clock. */
	LL_PWR_EnableRange1BoostMode();
	LL_RCC_HSI_Enable();
	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {
		;
	}

	/* Configure and enable the PLL: HSI(16MHz) / 4 * 80 / 2 = 160MHz */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 80, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_EnableDomain_SYS();
	LL_RCC_PLL_Enable();
	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {
		;
	}

	/* Set the source for the system clock and the AHB prescaler. */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
		;
	}

	/* Insure 1µs transition state at intermediate medium speed clock based on DWT */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;
	while (DWT->CYCCNT < 100)
		;

	/* Configure peripheral bus prescalers. */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	/* Update the system clock speed setting. */
	LL_SetSystemCoreClock(BOOT_CPU_SYSTEM_SPEED_KHZ * 1000u);
	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		ASSERT_RT(BLT_FALSE);
	}

	/* Set peripheral clock sources. */
	/* FDCAN 直接走 PCLK1 = 160MHz，这样就不需要额外配 PLLQ 了 */
	LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

#if (BOOT_COM_RS232_ENABLE > 0)
	LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
#endif
} /*** end of SystemClock_Config ***/
void HAL_MspInit(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* PWR and SYSCFG clock enable. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	/* Disable the internal pull-up in Dead Battery pins of UCPD peripheral. */
	LL_PWR_DisableUCPDDeadBattery();

	/* Disable LSE to free PC14/PC15 for GPIO usage. */
	LL_RCC_LSE_Disable();

	/* GPIO ports clock enable. */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

#if (BOOT_COM_RS232_ENABLE > 0)
	/* UART clock enable. */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
#endif
#if (BOOT_COM_CAN_ENABLE > 0)
	/* CAN clock enable. */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
#endif
	/* Configure GPIO pin for the LED. */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15);

	/* Configure GPIO pin for (optional) backdoor entry input. */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#if (BOOT_COM_RS232_ENABLE > 0)
	/* UART TX and RX GPIO pin configuration. */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
#if (BOOT_COM_CAN_ENABLE > 0)
	/* CAN TX and RX GPIO pin configuration. */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
} /*** end of HAL_MspInit ***/
void HAL_MspDeInit(void)
{
	/* Reset the RCC clock configuration to the default reset state. */
	LL_RCC_DeInit();

	/* Reset GPIO pin for the LED to turn it off. */
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15);

	/* Deinit used GPIOs. */
	LL_GPIO_DeInit(GPIOC);
	LL_GPIO_DeInit(GPIOB);
	LL_GPIO_DeInit(GPIOA);

#if (BOOT_COM_CAN_ENABLE > 0)
	/* CAN clock disable. */
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_FDCAN);
#endif
#if (BOOT_COM_RS232_ENABLE > 0)
	/* UART clock disable. */
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
#endif

	/* GPIO ports clock disable. */
	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

	/* Enable the internal pull-up in Dead Battery pins of UCPD peripheral. */
	LL_PWR_EnableUCPDDeadBattery();
	/* SYSCFG and PWR clock disable. */
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
} /*** end of HAL_MspDeInit ***/
void SysTick_Handler(void)
{
	/* Nothing to do here. */
} /*** end of SysTick_Handler ***/

/*********************************** end of main.c *************************************/

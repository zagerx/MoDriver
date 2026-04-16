#include "spi.h"
#include "gpio.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"

#define AS5047_CMD_READ_ANGLEUNC 0x7FFE

static __attribute__((always_inline)) inline uint16_t spi1_ll_trx_16bit(uint16_t txdata)
{
	/* 等待发送缓冲区空 */
	while (!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData16(SPI1, txdata);
	/* 等待接收缓冲区非空 */
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
	return LL_SPI_ReceiveData16(SPI1);
}

void encoder_init_pipeline(void)
{
	LL_SPI_Enable(SPI1);
	LL_GPIO_ResetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
	(void)spi1_ll_trx_16bit(AS5047_CMD_READ_ANGLEUNC);
	LL_GPIO_SetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
}

uint16_t encoder_getraw(void)
{
	LL_GPIO_ResetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
	uint16_t data = spi1_ll_trx_16bit(AS5047_CMD_READ_ANGLEUNC);
	LL_GPIO_SetOutputPin(AS5047_CS_GPIO_Port, AS5047_CS_Pin);
	return (data & 0x3FFF);
}

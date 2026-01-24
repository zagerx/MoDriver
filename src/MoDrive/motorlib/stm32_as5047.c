#include "stm32_as5047.h"
#include "main.h"
#include "spi.h"
#include "stdint.h"
/*
 *  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
 *  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
 *  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
 */

// AS5047p 地址
#define NOP      0x0000
#define ERRFL    0x0001
#define PROG     0x0003
#define DIAAGC   0x3FFC
#define MAG      0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM     0x0016
#define ZPOSL     0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

// 计算奇偶函数
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value = 0;
	while (data_2_cal != 0) {
		parity_bit_value ^= data_2_cal;
		data_2_cal >>= 1;
	}
	return (parity_bit_value & 0x1);
}
// SPI发送读取函数
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{
	HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_RESET);
	uint16_t rxdata;
	if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_txdata, (uint8_t *)&rxdata, 1, 1000) !=
	    HAL_OK) {
		rxdata = 0;
	}
	HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_SET);
	return rxdata;
}
uint16_t AS5047_read(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;
	// 读指令 bit14 置1
	if (Parity_bit_Calculate(add) == 1) {
		add = add | 0x8000; // 如果前15位 1的个数位偶数，则Bit15 置1
	}
	SPI_ReadWrite_OneByte(add);                   // 发送一条指令，不管读回的数据
	data = SPI_ReadWrite_OneByte((NOP | 0x4000)); // 发送一条空指令，读取上一次指令返回的数据。
	data &= 0x3fff;
	return data;
}

uint32_t as5047_read_raw(void)
{

	return AS5047_read(ANGLEUNC);
}

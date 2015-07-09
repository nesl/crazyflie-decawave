#ifndef DECA_SPI_H_
#define DECA_SPI_H_

// Includes
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_spi.h"

// Spi Hardware Setup
#define DW_SCK_PIN (GPIO_Pin_3)
#define DW_SCK_PSOURCE (GPIO_PinSource3)
#define DW_MISO_PIN (GPIO_Pin_4)
#define DW_MISO_PSOURCE (GPIO_PinSource4)
#define DW_MOSI_PIN (GPIO_Pin_5)
#define DW_MOSI_PSOURCE (GPIO_PinSource5)
#define DW_CS_PIN (GPIO_Pin_6)
#define DW_SPI_PORT (GPIOB)
#define DW_SPI 		(SPI1)
#define DW_SPI_AF	(GPIO_AF_SPI1)
#define PORT_SPI_CLEAR_CS_FAST	{DW_SPI_PORT->BRR = DW_CS_PIN;}
#define PORT_SPI_SET_CS_FAST	{DW_SPI_PORT->BSRR = DW_CS_PIN;}

// Spi DMA Setup
#define DMA_TX_CH	DMA2_Channel3
#define DMA_RX_CH	DMA2_Channel3
#define PORT_DMA_START_TX_FAST 	{DMA_TX_CH->CCR |= DMA_CCR1_EN;}
#define PORT_DMA_STOP_TX_FAST 	{DMA_TX_CH->CCR &= ~DMA_CCR1_EN;}
#define PORT_DMA_START_RX_FAST 	{DMA_RX_CH->CCR |= DMA_CCR1_EN;}
#define PORT_DMA_STOP_RX_FAST 	{DMA_RX_CH->CCR &= ~DMA_CCR1_EN;}

// Busy waits
#define CLOCK_TICKS_PER_USEC (33)

// Function definitions
void dw_spi_init();
uint8_t dw_spi_transferbyte(uint8_t byte);
int dw_spi_sendpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, const uint8_t* body);
int dw_spi_readpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, uint8_t* body);
void dw_spi_busysleep(uint32_t usec);

// Macros
#define writetospi dw_spi_sendpacket
#define readfromspi dw_spi_readpacket
#define usleep(x) dw_spi_busysleep(x)
#define Sleep(x) usleep(1000*x)

#endif
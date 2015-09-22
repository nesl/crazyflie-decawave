#ifndef DECARANGING_H_
#define DECARANGING_H_

// includes
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "deca_ranging.h"
#include "FreeRTOS.h"
#include "timers.h"

// Spi Hardware Setup ( PA5, 6, 7 are SCK, MISO, MOSI)
#define DW_SPI_PORT (GPIOA)
#define DW_SPI      (SPI1)
#define DW_SPI_AF   (GPIO_AF_SPI1)

// SCK = PA5
#define DW_SCK_PIN      (GPIO_Pin_5)
#define DW_SCK_PSOURCE  (GPIO_PinSource5)

// MISO = PA6
#define DW_MISO_PIN (GPIO_Pin_6)
#define DW_MISO_PSOURCE (GPIO_PinSource6)

// MOSI = PA7
#define DW_MOSI_PIN (GPIO_Pin_7)
#define DW_MOSI_PSOURCE (GPIO_PinSource7)

// CSn = PC12
#define DW_SPI_CS_PORT  (GPIOC)
#define DW_CS_PIN  (GPIO_Pin_12)
#define DW_SPI_CLEAR_CS_FAST {DW_SPI_CS_PORT->BRR = DW_CS_PIN;}
#define DW_SPI_SET_CS_FAST   {DW_SPI_CS_PORT->BSRR = DW_CS_PIN;}

// SPI peripheral
#define DW_SPI_PERIPH (RCC_APB2Periph_SPI1)
#define DW_SPI_GPIOPERIPH (RCC_AHB1Periph_GPIOA)
#define DW_SPI_CSPERIPH   (RCC_AHB1Periph_GPIOC)

// DW Reset Pin (PB5)
#define DW_RESET_PORT (GPIOB)
#define DW_RESET_PIN  (GPIO_Pin_5) // D4
#define DW_RESET_PERIPH (RCC_AHB1Periph_GPIOB)

// DW IRQ Pin (PB4)
#define DW_IRQ_PORT    (GPIOB)
#define DW_IRQ_PIN     (GPIO_Pin_4)
#define DW_IRQ_PERIPH  (RCC_AHB1Periph_GPIOB)
#define DW_IRQ_LINE    (EXTI_Line4)
#define DW_IRQ_CHNL    (EXTI4_IRQn)
#define DW_IRQ_EXTI    (EXTI_PortSourceGPIOB)
#define DW_IRQ_PSOURCE (EXTI_PinSource4)

// Misc.
#define CLOCK_TICKS_PER_USEC (33)
#define MAX_32BIT_INT (4294967296)
#define RANGE_PERIOD_MS (100)

// Local node ID
#define DECARANGE_LOCAL_UID (101)
#define DECARANGE_PAN       (0xAE70)

// INIT FUNCTIONS
void dw_init();
void dw_hw_init();
void dw_worker(void * data);
void dw_range_complete(DWR_RangeEst_t *range);

// SPI FUNCTIONS
uint8_t dw_spi_transferbyte(uint8_t byte);
int dw_spi_sendpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, const uint8_t* body);
int dw_spi_readpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, uint8_t* body);
void dw_spi_busysleep(uint32_t usec);

// BUSY WAIT FUNCTIONS
void dw_sleep_usec(uint32_t usec);
void dw_sleep_msec(uint32_t msec);

// RANDOM NUMBER FUNCTIONS
void dw_rng_init( void );
float dw_rng_float( void );

// DEBUG FUNCTIONS
void dw_debug_handler(uint8_t event);

#endif // DECARANGING_H_
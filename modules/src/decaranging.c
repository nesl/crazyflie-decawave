#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_spi.h"
#include "decaranging.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"


// FreeRTOS Scheduling
static xTimerHandle timer;
static void dw_timer_callback(xTimerHandle timer);

void dw_init()
{
	// initialize hardware
	dw_hw_init();

	// initialize the decaranging library
  	decaranging_init(
			DECARANGE_PAN,       // PAN address (constant for all nodes)
			DECARANGE_LOCAL_UID, // SHORT address (unique for all nodes)
			NODE_TYPE_MOBILE, // Node Type (fixed, mobile, or mobile responsive)
			dw_range_complete,// Ranging complete handler
			dw_spi_sendpacket,// SPI send packet 
			dw_spi_readpacket,// SPI read packet 
			dw_debug_handler, // Handle ranging events
			dw_sleep_msec,    // sleep in milliseconds
			dw_rng_float      // random number generator (float)
        );

  	// increase SPI frequency to accomodate short turnaround times
  	dw_spi_configprescaler(16); // was 32

	// set up task timer
	timer = xTimerCreate( (const signed char *)"decaTimer", M2T(RANGE_PERIOD_MS),
				pdTRUE, NULL, dw_timer_callback );
	xTimerStart(timer, 100);
}

static void dw_timer_callback(xTimerHandle timer)
{
	workerSchedule(dw_worker, NULL);
}

void dw_worker(void * data)
{
	// periodic call to decaranging library. In the normal case, this
	// will initiate a ranging sequence
	decaranging_poll(RANGE_PERIOD_MS);

	// blink butterfly green LED
	dwt_ntbled_toggle_green();
}

void dw_hw_init()
{
	// Init structures to be used throughout
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	// ==================================
	//          INITIALIZE SPI
	// ==================================
	// de-initialize to begin
	SPI_I2S_DeInit(DW_SPI);
	// enable peripheral clock
	RCC_APB2PeriphClockCmd(DW_SPI_PERIPH, ENABLE);

	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;  
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;       
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 4-64
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(DW_SPI, &SPI_InitStruct); 

	// Turn on port A for peripheral pins (SCK, MOSI, MISO)
	RCC_AHB1PeriphClockCmd(DW_SPI_GPIOPERIPH, ENABLE);
	 
	// Configure SCK, MISO, & MOSI
	GPIO_InitStruct.GPIO_Pin = DW_SCK_PIN | DW_MISO_PIN | DW_MOSI_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DW_SPI_PORT, &GPIO_InitStruct);

	// Configure CSn
	GPIO_InitStruct.GPIO_Pin = DW_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW_SPI_CS_PORT, &GPIO_InitStruct);

	// SPI alternate functions
	GPIO_PinAFConfig(DW_SPI_PORT, DW_SCK_PSOURCE,  DW_SPI_AF);
	GPIO_PinAFConfig(DW_SPI_PORT, DW_MISO_PSOURCE, DW_SPI_AF);
	GPIO_PinAFConfig(DW_SPI_PORT, DW_MOSI_PSOURCE, DW_SPI_AF);

	// Disable SPI SS output
	SPI_SSOutputCmd(DW_SPI, DISABLE);

	// start with CS high
	GPIO_SetBits(DW_SPI_CS_PORT, DW_CS_PIN);

	// Enable SPI
	SPI_Cmd(DW_SPI, ENABLE);

	// ==================================
	//           HW RESET LINE
	// ==================================

	// Configure DW reset line	
	RCC_AHB1PeriphClockCmd(DW_RESET_PERIPH, ENABLE);

	GPIO_InitStruct.GPIO_Pin = DW_RESET_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(DW_RESET_PORT, &GPIO_InitStruct);

	// ensure reset is high to begin
	GPIO_SetBits(DW_RESET_PORT, DW_RESET_PIN);


	// ==================================
	//            HW IRQ LINE
	// ==================================
	// Configure the DW IRQ line
	GPIO_InitStruct.GPIO_Pin = DW_IRQ_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(DW_IRQ_PORT, &GPIO_InitStruct);
	SYSCFG_EXTILineConfig(DW_IRQ_EXTI, DW_IRQ_PSOURCE);

	// Configure the EXTI for DW IRQ
	EXTI_InitStruct.EXTI_Line = DW_IRQ_LINE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = DW_IRQ_CHNL;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15; // lowest
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	// ==================================
	//      RANDOM NUMBER GENERATOR
	// ==================================
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	RNG_Cmd(ENABLE);

}

uint8_t dw_spi_transferbyte(uint8_t byte)
{
	while(!SPI_I2S_GetFlagStatus(DW_SPI, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(DW_SPI, byte);
	while(!SPI_I2S_GetFlagStatus(DW_SPI, SPI_I2S_FLAG_RXNE));

	return SPI_I2S_ReceiveData(DW_SPI);
}

int dw_spi_sendpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, const uint8_t* body)
{
	int i;
	//uint8_t tmp;

	// CS Low
	GPIO_ResetBits(DW_SPI_CS_PORT, DW_CS_PIN);

	// send header
	for( i=0; i<headerLen; i++ )
	{
		dw_spi_transferbyte( header[i] );
	}
	// send body
	for( i=0; i<bodyLen; i++ )
	{
		dw_spi_transferbyte( body[i] );
	}

	// CS high
	GPIO_SetBits(DW_SPI_CS_PORT, DW_CS_PIN);

	return 0;
}

int dw_spi_readpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, uint8_t* body)
{
	int i;
	//uint8_t tmp;

	// CS Low
	GPIO_ResetBits(DW_SPI_CS_PORT, DW_CS_PIN);

	// send header
	for( i=0; i<headerLen; i++ )
	{
		dw_spi_transferbyte( header[i] );
	}
	// send body
	for( i=0; i<bodyLen; i++ )
	{
		body[i] = dw_spi_transferbyte( 0x00 );
	}

	// CS high
	GPIO_SetBits(DW_SPI_CS_PORT, DW_CS_PIN);

	return 0;
}

void dw_sleep_usec(uint32_t usec)
{
	int i;
	for( i=0; i<usec*CLOCK_TICKS_PER_USEC; i++ )
	{
		__asm("nop");
	}
}

void dw_sleep_msec(uint32_t msec)
{
	dw_sleep_usec(msec*1000);
}


void dw_spi_configprescaler(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_I2S_DeInit(DW_SPI);

	// SPI Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(DW_SPI, &SPI_InitStructure);

	// Enable SPI
	SPI_Cmd(DW_SPI, ENABLE);
}


float dw_rng_float()
{
	uint32_t rand32;
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET){}
	rand32 = RNG_GetRandomNumber();
	return (float)rand32/MAX_32BIT_INT;
}


void dw_debug_handler(uint8_t event)
{
	switch( event )
	{
		case DWRANGE_EVENT_ERROR:
			break;
		case DWRANGE_EVENT_RXGOOD:
			break;
		case DWRANGE_EVENT_TXGOOD:
			break;
		case DWRANGE_EVENT_IRQ:
			break;
		case DWRANGE_EVENT_RXINIT:
			break;
		case DWRANGE_EVENT_RXFIN:
			break;
		case DWRANGE_EVENT_UNKNOWN_IRQ:
			break;
		case DWRANGE_EVENT_BADFRAME:
			break;
		default:
			break;
	}
}

void dw_range_complete(DWR_RangeEst_t *range)
{
	// Not using this for mobile nodes for now	
}




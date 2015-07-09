#include "deca_spi.h"

void dw_spi_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	SPI_I2S_DeInit(DW_SPI);

	/* ------------ Fire up SPI --------------- */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

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

	// Turn on port B
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	 
	// Configure SCK and MOSI
	GPIO_InitStruct.GPIO_Pin = DW_SCK_PIN | DW_MOSI_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW_SPI_PORT, &GPIO_InitStruct);

	// Configure MISO
	GPIO_InitStruct.GPIO_Pin = DW_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(DW_SPI_PORT, &GPIO_InitStruct);

	// Configure CS
	GPIO_InitStruct.GPIO_Pin = DW_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW_SPI_PORT, &GPIO_InitStruct);

	// SPI alternate functions
	GPIO_PinAFConfig(DW_SPI_PORT, DW_SCK_PSOURCE, DW_SPI_AF);
	GPIO_PinAFConfig(DW_SPI_PORT, DW_MISO_PSOURCE, DW_SPI_AF);
	GPIO_PinAFConfig(DW_SPI_PORT, DW_MOSI_PSOURCE, DW_SPI_AF);

	// Disable SPI SS output
	SPI_SSOutputCmd(DW_SPI, DISABLE);

	// Enable SPI
	SPI_Cmd(DW_SPI, ENABLE);

	// start with CS high
	GPIO_SetBits(DW_SPI_PORT, DW_CS_PIN);

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
	uint8_t tmp;

	// CS Low and wait a bit
	GPIO_ResetBits(DW_SPI_PORT, DW_CS_PIN);
	dw_spi_busysleep(1);

	// send header
	for( i=0; i<headerLen; i++ )
	{
		tmp = dw_spi_transferbyte( header[i] );
	}
	// send body
	for( i=0; i<bodyLen; i++ )
	{
		tmp = dw_spi_transferbyte( body[i] );
	}

	// CS high and wait a bit
	GPIO_SetBits(DW_SPI_PORT, DW_CS_PIN);
	dw_spi_busysleep(1);

	return 0;
}

int dw_spi_readpacket(uint16_t headerLen, const uint8_t* header, uint32_t bodyLen, uint8_t* body)
{
	int i;
	uint8_t tmp;

	// CS Low and wait a bit
	GPIO_ResetBits(DW_SPI_PORT, DW_CS_PIN);
	dw_spi_busysleep(1);

	// send header
	for( i=0; i<headerLen; i++ )
	{
		tmp = dw_spi_transferbyte( header[i] );
	}
	// send body
	for( i=0; i<bodyLen; i++ )
	{
		body[i] = dw_spi_transferbyte( 0x00 );
	}

	// CS high and wait a bit
	GPIO_SetBits(DW_SPI_PORT, DW_CS_PIN);
	dw_spi_busysleep(1);

	return 0;
}

void dw_spi_busysleep(uint32_t usec)
{
	int i;
	for( i=0; i<usec*CLOCK_TICKS_PER_USEC; i++ )
	{
		__asm("nop");
	}
}


void dw_spi_configprescaler(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_I2S_DeInit(DW_SPI);

	// SPIx Mode setup
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

	// Enable SPIx
	SPI_Cmd(DW_SPI, ENABLE);
}

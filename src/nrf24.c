// nrf24.c
// 
// Low-level radio driver for the Nordic Semi nrf24l01, Beken2432,
// and Panchip XN297. These IC's are broadly compatible, with some
// special register setup required depending upon the target.
//
// This file is part of the CX10_fnrf project, released under the 
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright � 	2015 Samuel Powell
//							2015 Goebish
//							2015 Bart Slinger
//							2014 Felix Niessen

#include "config.h"

#include "stm32f0xx_conf.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_exti.h"

/* Private variables */
static bool isInit;
static void (*interruptCb)(void) = NULL;

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_SendData8(RADIO_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_ReceiveData8(RADIO_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

/****************************************************************
 * nRF SPI commands, Every commands return the status byte      *
 ****************************************************************/

/* Read len bytes from a nRF24L register. 5 Bytes max */
unsigned char nrfReadReg(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte( CMD_R_REG | (address&0x1F) );
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

/* Write len bytes a nRF24L register. 5 Bytes max */
unsigned char nrfWriteReg(unsigned char address, char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the write command with the address */
  status = spiSendByte( CMD_W_REG | (address&0x1F) );
  /* Write LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

/* Write only one byte (useful for most of the reg.) */
unsigned char nrfWrite1Reg(unsigned char address, char byte)
{
  return nrfWriteReg(address, &byte, 1);
}

/* Read only one byte (useful for most of the reg.) */
unsigned char nrfRead1Reg(unsigned char address) {
  char byte;

  nrfReadReg(address, &byte, 1);

  return byte;
}

/* Sent the NOP command. Used to get the status byte */
unsigned char nrfNop()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_NOP);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushRx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_RX);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfFlushTx()
{
  unsigned char status;

  RADIO_EN_CS();
  status = spiSendByte(CMD_FLUSH_TX);
  RADIO_DIS_CS();

  return status;
}

// Return the payload length
unsigned char nrfRxLength(unsigned int pipe)
{
  unsigned char length;

  RADIO_EN_CS();
  spiSendByte(CMD_RX_PL_WID);
  length = spiReceiveByte();
  RADIO_DIS_CS();

  return length;
}

unsigned char nrfActivate()
{
  unsigned char status;
  
  RADIO_EN_CS();
  status = spiSendByte(CMD_ACTIVATE);
  spiSendByte(ACTIVATE_DATA);
  RADIO_DIS_CS();

  return status;
}

unsigned char nrfActivateBK2423()
{
    unsigned char status;
    
    RADIO_EN_CS();
    status = spiSendByte(CMD_ACTIVATE);
    spiSendByte(ACTIVATE_BK2423_DATA);
    RADIO_DIS_CS();
    
    return status;
}

// Write the ack payload of the pipe 0
unsigned char nrfWriteAck(unsigned int pipe, char *buffer, int len)
{
  unsigned char status;
  int i;

  //ASSERT(pipe<6);

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_W_ACK_PAYLOAD(pipe));
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);

  RADIO_DIS_CS();

  return status;
}

// Read the RX payload
unsigned char nrfReadRX(char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_R_RX_PAYLOAD);
  /* Read LEN bytes */
  for(i=0; i<len; i++)
    buffer[i]=spiReceiveByte();

  RADIO_DIS_CS();

  return status;
}

unsigned char nrfSendTX(char *buffer, int len)
{
  unsigned char status;
  int i;

  RADIO_EN_CS();

  /* Send the read command with the address */
  status = spiSendByte(CMD_W_TX_PAYLOAD);
  /* Send LEN bytes */
  for(i=0; i<len; i++)
    spiSendByte(buffer[i]);
  RADIO_DIS_CS();

  return status;
}

/* Interrupt service routine, call the interrupt callback
 */
void nrfIsr()
{
  if (interruptCb)
    interruptCb();

  return;
}

void nrfSetInterruptCallback(void (*cb)(void))
{
  interruptCb = cb;
}

void nrfSetChannel(unsigned int channel)
{
  if (channel<126)
    nrfWrite1Reg(REG_RF_CH, channel);
}

void nrfSetDatarate(int datarate)
{
  switch(datarate)
  {
    case RADIO_RATE_250K:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_250K);
      break;
    case RADIO_RATE_1M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_1M);
      break;
    case RADIO_RATE_2M:
      nrfWrite1Reg(REG_RF_SETUP, VAL_RF_SETUP_2M);
      break;
  }  
}

void nrfSetAddress(unsigned int pipe, char* address)
{
  int len = 5;

//  ASSERT(pipe<6);

  if (pipe > 1)
    len = 1;

  nrfWriteReg(REG_RX_ADDR_P0 + pipe, address, len);
}

void nrfSetEnable(bool enable)
{
  if (enable)
  {
    RADIO_EN_CE();
  } 
  else
  {
    RADIO_DIS_CE();
  }
}

unsigned char nrfGetStatus()
{
  return nrfNop();
}

/* Initialisation */
void nrfInit(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

    // Clocks configured in main
	
	// Initialise GPIO structure
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

    // SPI SCK pin configuration
    GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_SCK;
    GPIO_Init(RADIO_SPI_PORT, &GPIO_InitStructure);

    // SPI  MOSI pin configuration
    GPIO_InitStructure.GPIO_Pin =  RADIO_GPIO_SPI_MOSI;
    GPIO_Init(RADIO_SPI_PORT, &GPIO_InitStructure);

    // SPI MISO pin configuration
    GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_MISO;
    GPIO_Init(RADIO_SPI_PORT, &GPIO_InitStructure);
	
    // SPI CS pin configuration
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_CS;
    GPIO_Init(RADIO_SPI_CS_PORT, &GPIO_InitStructure);

    // RADIO CE pin configuration
    // NOTE: This is only used on the blue CX10, which uses a two-way protocol
    #if defined(CX10_BLUE)
    GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
    GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);
    #endif

    // Set pin alternative functions for SPI data and clocks
    GPIO_PinAFConfig(RADIO_SPI_PORT, RADIO_GPIO_SPI_SCK, GPIO_AF_0);
    GPIO_PinAFConfig(RADIO_SPI_PORT, RADIO_GPIO_SPI_MOSI, GPIO_AF_0);
    GPIO_PinAFConfig(RADIO_SPI_PORT, RADIO_GPIO_SPI_MISO, GPIO_AF_0);

    // Desiable the IC prior to SPI configuration
    RADIO_DIS_CS();
    RADIO_DIS_CE();
  
    // SPI configuration
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(RADIO_SPI, &SPI_InitStructure);
	
	// Set interrupt on 8-bit return
	SPI_RxFIFOThresholdConfig(RADIO_SPI, SPI_RxFIFOThreshold_QF);

    // Enable SPI 
    SPI_Cmd(RADIO_SPI, ENABLE);
  
    isInit = true;
}


bool nrfTest(void)
{
	return isInit;
}

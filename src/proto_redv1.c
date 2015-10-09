/*  Cheerson CX-10 integrated RF rate mode firmware.
     - Red PCB protocol implementation.

    This source contains components from Felix Niessen's original
    firmware (GPLv3), see the following thread for details:

    http://www.rcgroups.com/forums/showpost.php?p=30045580&postcount=1.

    The YD717 (Skywalker) protocol, and Beken 2423 initialisation
    routines were derived from the DeviationTX project (GPLv3), in
    particular the source file yd717_nrf24l01.c from:

    https://bitbucket.org/PhracturedBlue/

    All original components are provided under the GPLv3:

    Copyright (C) 2015, Samuel Powell.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "config.h"

#ifdef RF_PROTO_REDV1

#define RF_CHANNEL      0x3C      // Stock TX fixed frequency
#define PAYLOADSIZE       9          // Protocol packet size

extern uint8_t failsave;
bool bind = false;
extern int16_t RXcommands[6];

char rxbuffer[9] = {0,0,0,0,0,0,0,0,0};
const char rf_addr_bind[5] = {0x65, 0x65, 0x65, 0x65, 0x65};
static char rf_addr_cmnd[5];

bool flashstate = false;
uint32_t flashtime;

// Configure the nrf24/Beken 2423 and bind
void init_RFRX() {
  
  // Initialise SPI, clocks, etc.
  nrfInit();
    
  // Power down
  nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC  | NRF24_PRIM_RX));  

  // Configuration
  nrfWrite1Reg(REG_EN_AA, NRF24_ENAA_PA);          // Enable auto-ack on all pipes
  nrfWrite1Reg(REG_EN_RXADDR, NRF24_ERX_PA);      // Enable all pipes
  nrfWrite1Reg(REG_SETUP_AW, NRF24_AW_5_BYTES);    // 5-byte TX/RX adddress

  nrfWrite1Reg(REG_SETUP_RETR, 0x1A);              // 500uS timeout, 10 retries
  nrfWrite1Reg(REG_RF_CH, RF_CHANNEL);            // Channel 0x3C
  nrfWrite1Reg(REG_RF_SETUP, NRF24_PWR_0dBm);       // 1Mbps, 0dBm
  nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);       // Clear status

  nrfWrite1Reg( REG_RX_PW_P0, PAYLOADSIZE);         // Set payload size on all RX pipes
  nrfWrite1Reg( REG_RX_PW_P1, PAYLOADSIZE);
  nrfWrite1Reg( REG_RX_PW_P2, PAYLOADSIZE);
  nrfWrite1Reg( REG_RX_PW_P3, PAYLOADSIZE);
  nrfWrite1Reg( REG_RX_PW_P4, PAYLOADSIZE);
  nrfWrite1Reg( REG_RX_PW_P5, PAYLOADSIZE);

  nrfWrite1Reg( REG_FIFO_STATUS, 0x00);           // Clear FIFO bits (unnesseary?)

  // We we need to activate feature before we do this, presubaly we can delete the next two lines
  nrfWrite1Reg( REG_DYNPD, 0x3F);                  // Enable dynamic payload (all pipes)
  nrfWrite1Reg( REG_FEATURE, 0x07);                // Payloads with ACK, noack command

   // Maybe required
  nrfRead1Reg(REG_FEATURE);
  nrfActivate();                                  // Activate feature register
  nrfRead1Reg(REG_FEATURE);
  nrfWrite1Reg(REG_DYNPD, 0x3F);                   // Enable dynamic payload length on all pipes
  nrfWrite1Reg(REG_FEATURE, 0x07);                 // Set feature bits on

  // Check for Beken BK2421/BK2423 chip
  // It is done by using Beken specific activate code, 0x53
  // and checking that status register changed appropriately
  // There is no harm to run it on nRF24L01 because following
  // closing activate command changes state back even if it
  // does something on nRF24L01
  nrfActivateBK2423();

  if (nrfRead1Reg(REG_STATUS) & 0x80) {
      
      // RF is Beken 2423
      nrfWriteReg(0x00, (char *) "\x40\x4B\x01\xE2", 4);
      nrfWriteReg(0x01, (char *) "\xC0\x4B\x00\x00", 4);
      nrfWriteReg(0x02, (char *) "\xD0\xFC\x8C\x02", 4);
      nrfWriteReg(0x03, (char *) "\x99\x00\x39\x21", 4);
      nrfWriteReg(0x04, (char *) "\xD9\x96\x82\x1B", 4);
      nrfWriteReg(0x05, (char *) "\x24\x06\x7F\xA6", 4);
      nrfWriteReg(0x0C, (char *) "\x00\x12\x73\x00", 4);
      nrfWriteReg(0x0D, (char *) "\x46\xB4\x80\x00", 4);
      nrfWriteReg(0x04, (char *) "\xDF\x96\x82\x1B", 4);
      nrfWriteReg(0x04, (char *) "\xD9\x96\x82\x1B", 4);
  }

  nrfActivateBK2423(); 

  // Flush the tranmit and receive buffer
  nrfFlushRx();
  nrfFlushTx();

  // Set device to bind address
  nrfWriteReg( REG_RX_ADDR_P0, (char *) rf_addr_bind, 5);
  nrfWriteReg( REG_TX_ADDR, (char *) rf_addr_bind, 5);
  
  // Power up
  nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX));
  
  while(!bind) {
    
    // Wait until we receive a data packet, flashing alternately
		flashtime = micros()/1000;
    while(!(nrfGetStatus() & 0x40))  bindflasher(500);
  
    
      // TX sends mutliple packets, so keep reading the FIFO
      // until there is no more data.
      while (!(nrfRead1Reg(REG_FIFO_STATUS) & 0x01)) { 
        
        // Bind packet is nine bytes on pipe zero
        if(nrfRxLength(0) == PAYLOADSIZE) {
          
          // Get the packet
          nrfReadRX(rxbuffer, PAYLOADSIZE);
        }
      }
      
      // Flush buffer and clear status
      nrfFlushRx();
      nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
      
        // Configure the command address
        rf_addr_cmnd[0] = rxbuffer[0];
        rf_addr_cmnd[1] = rxbuffer[1];
        rf_addr_cmnd[2] = rxbuffer[2];
        rf_addr_cmnd[3] = rxbuffer[3];
        rf_addr_cmnd[4] = 0xC1;     
        
        // Set to TX command address
        nrfWriteReg( REG_RX_ADDR_P0,  rf_addr_cmnd, 5);
        nrfWriteReg( REG_TX_ADDR, rf_addr_cmnd, 5);
        
        // Flush buffer and clear status
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        
        // Wait until we receive data on the command address
				flashtime = micros()/1000;
        while(!(nrfGetStatus() & 0x40)) bindflasher(250);
        
        bind = true;
				
				// Turn of LEDs
				GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);  
        GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);  
              
      }

}



// Place RF command data in RXcommand variable, process AUX commands
void get_RFRXDatas() {
    
  // If a new packet exists in the buffer
  if(nrfGetStatus() & 0x40)
  {
    // Read the latest command to the buffer
    nrfReadRX(rxbuffer, PAYLOADSIZE);
    
    // Flush the buffer and clear interrupt 
    nrfFlushRx();
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
    
    // FC order: T   A   E   R   A1   A2
    // RF order: T    R   ?   E   A     Et    At   F   ?
    
    // PPM firmware expects a range of 1000, TX range is 0xFF (max rate), mid-stick is 0x40,
    // BS twice leads to a range of 1020, which is close enough for now.
    RXcommands[0] = constrain((((int16_t)rxbuffer[0])<<2), 0, 1000);
    RXcommands[1] = constrain((((int16_t)rxbuffer[4])<<2) - 512, -500, 500);
    RXcommands[2] = constrain((((int16_t)rxbuffer[3])<<2) - 512, -500, 500);
    RXcommands[3] = constrain((((int16_t)rxbuffer[1])<<2) - 512, -500, 500);
    
    // Forward flip sets AUX1 high, backwards flip sets AUX1 low
    if( rxbuffer[7] & 0x0F ) {
      if((uint8_t) rxbuffer[3] > 0xF0) RXcommands[4] = 500;
      if((uint8_t) rxbuffer[3] < 0x0F) RXcommands[4] = -500;
    }
        
    // Since data has been received, reset failsafe counter
    failsave = 0;
  }

}


void bindflasher(uint32_t rate) {
	
	uint32_t millitime = micros()/1000;
	
	 if(millitime-flashtime > rate)
      {
        flashtime = millitime;
        switch(flashstate)
        {
        
          case true:
            GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);  
            GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);  
            flashstate = false;
            break;
        
          case false:
            GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);  
            GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
            flashstate = true;
            break;
        
        }
      }
			
}

#endif

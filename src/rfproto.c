// proto_blue.c
//
// This file is part of the CX10_fnrf project, released under the
// GNU General Public License, see LICENSE.md for further details.
//
// Copyright ? 	2015 Samuel Powell
//              2015 Goebish
//              2015 Bart Slinger

// Implementation of the RF protocols of the red (v1) and blue CX10
// devices. Provides functions to bind to the TX, and receive the 
// command data.

#include "config.h"

extern uint8_t failsafe;

 // XN297 settings
 static const char demod_cal[5] = {0x0B, 0xDF, 0xC4, 0xA7, 0x03};
 static const char rf_cal[7]    = {0xC9, 0x9A, 0xB0, 0x61, 0xBB, 0xAB, 0x9C};
 static const char bb_cal[5]    = {0x4C, 0x84, 0x67, 0x9C, 0x20};

#ifdef RF_PROTO_REDV1
    #define RF_BIND_CHANNEL 0x3C
    #define PAYLOADSIZE     9
    const char rf_addr_bind[5] = {0x65, 0x65, 0x65, 0x65, 0x65};
    static char rf_addr_cmnd[5];
#endif

#ifdef RF_PROTO_BLUE
    #define RF_BIND_CHANNEL      0x02        // Stock TX fixed frequency
    #define PAYLOADSIZE 19    
    #define CX10_NUM_RF_CHANNELS    4
    const char rf_addr_bind[5]  = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
    static uint8_t CX10_freq[4]; // frequency hopping table
    static uint8_t CX10_current_chan = 0;
#endif
    
 // RF buffer
 static char rxbuffer[PAYLOADSIZE];

    
 
#ifdef RF_PROTO_BLUE

// Configure the nrf24/Beken 2423 and bind
void init_rf(){
    
    // Initialise SPI, clocks, etc.
    nrfInit();
    
    // Strange settings specific for xn297
    nrfWriteReg(REG_DEMOD_CAL, (char*) demod_cal, 5 );
    nrfWriteReg(REG_RF_CAL, (char*) rf_cal, 7 );
    nrfWriteReg(REG_BB_CAL, (char*) bb_cal, 5);
    
    nrfFlushTx();
    nrfFlushRx();
    
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);     // Clear status
    nrfWrite1Reg(REG_EN_AA, 0x00);                  // No auto-acknowledge
    nrfWrite1Reg(REG_EN_RXADDR, NRF24_ERX_P0);      // enable data pipe 0
    nrfWrite1Reg(REG_SETUP_AW, NRF24_AW_5_BYTES);   // 5-byte TX/RX adddress
    nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL);       // Channel 0x02
    nrfWrite1Reg(REG_SETUP_RETR, 0x00);             // disable retries
    nrfWrite1Reg(REG_RX_PW_P0, 19);                 // set payload size 19 bytes

    // set power 0dBm (max), last bit is 'HIGH CURRENT' in XN297 (not avail in NRF24)
    nrfWrite1Reg(REG_RF_SETUP, 0x07);
    
    nrfActivate();
    nrfWrite1Reg(REG_DYNPD, 0x00);                  // Disable dynamic payload
    nrfWrite1Reg(REG_FEATURE, 0x00);                // Disable features (ack stuff)
    

}

void bind_rf() {
    
    bool bind = false;
    

    char txbuffer[19] = {0xAA,                      // Fixed
                         0x00, 0x00, 0x00, 0x00,    // TXID
                         0x57, 0x02, 0x03, 0x81,    // ACID
                         0x00,                      // BIND
                         0x00, 0xDC, 0x05, 0xE8, 0x03, 0xDC, 0x05, 0x00, 0x00};
    
    // Set RX/TX address
    nrfWriteReg(REG_RX_ADDR_P0, (char *) rf_addr_bind, 5);
    nrfWriteReg(REG_TX_ADDR, (char *) rf_addr_bind, 5);
    
    // Power up, set RF channel
    nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX));
    nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL); // Channel 0x02
    
    while(!bind) {

        
        // STEP 1: Await bind packet
        
        // Wait until we receive a data packet, flashing alternately
        RADIO_EN_CE();
        while(!(nrfGetStatus() & 0x40));
        
        // Read FIFO until there is no more data
        while (!(nrfRead1Reg(REG_FIFO_STATUS) & 0x01))
        {
            // Bind packet is nine bytes on pipe zero
            if(nrfRxLength(0) == PAYLOADSIZE) nrfReadRX(rxbuffer, PAYLOADSIZE);
        }
        RADIO_DIS_CE();
        
        // Copy the TX address into the transmit buffer
        for(int i=1; i<5; i++) txbuffer[i] = rxbuffer[i];
        
        // Flush buffer and clear status
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        delay_micros(3000);
        
        // STEP 2: Reply to bind packet with aircraft ID
        
        // Configure for transmit
        nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP));
        nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL);
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        nrfFlushTx();
        
        nrfSendTX(txbuffer, 19);
        delay_micros(150);
        RADIO_EN_CE();
        delay_micros(6000);
        RADIO_DIS_CE();
        
        // STEP 3: Await bind packet with correct aircraft ID
        
        // Configure for receive
        nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX));
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL);
        delay_micros(300);
        
        // Wait until we receive a data packet, flashing alternately
        RADIO_EN_CE();
        while(!(nrfGetStatus() & 0x40));
        
        // Read FIFO until there is no more data
        while (!(nrfRead1Reg(REG_FIFO_STATUS) & 0x01))
        {
            // Bind packet is nine bytes on pipe zero
            if(nrfRxLength(0) == PAYLOADSIZE) nrfReadRX(rxbuffer, PAYLOADSIZE);
        }
        RADIO_DIS_CE();

        // We will be bound if the packet contains the correct ID
        bind = true;
        for(int i=5; i<9; i++)
        {
            if(txbuffer[i] != rxbuffer[i]) bind = false;
        
        }
        
        // STEP 4: Reply to bind packet indicating bound
        
        // Flush buffer and clear status
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        delay_micros(3000);
        
        // Configure for transmit
        nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP));
        nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL);
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        nrfFlushTx();
        
        txbuffer[9] = bind;
        
        delay_micros(6000);
        nrfSendTX(txbuffer, 19);
        delay_micros(150);
        RADIO_EN_CE();
        delay_micros(3000);
        RADIO_DIS_CE();
        
        // STEP 5: Set up frequency hopping table and move to data mode
        
        // Confiure frequency hopping table:
        CX10_freq[0] = (rxbuffer[1] & 0x0F) + 0x03;
        CX10_freq[1] = (rxbuffer[1] >> 4) + 0x16;
        CX10_freq[2] = (rxbuffer[2] & 0x0F) + 0x2D;
        CX10_freq[3] = (rxbuffer[2] >> 4) + 0x40;
        
        nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX));
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        nrfWrite1Reg(REG_RF_CH, CX10_freq[CX10_current_chan++]);
        delay_micros(300);
        RADIO_EN_CE();
    
    }
    
}





#endif







#ifdef RF_PROTO_REDV1

// Configure the nrf24/Beken 2423 and bind
void init_rf() {
  
  // Initialise SPI, clocks, etc.
  nrfInit();
    
  // Power down
  nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC  | NRF24_PRIM_RX));  

  // Configuration
  nrfWrite1Reg(REG_EN_AA, NRF24_ENAA_PA);          // Enable auto-ack on all pipes
  nrfWrite1Reg(REG_EN_RXADDR, NRF24_ERX_PA);      // Enable all pipes
  nrfWrite1Reg(REG_SETUP_AW, NRF24_AW_5_BYTES);    // 5-byte TX/RX adddress

  nrfWrite1Reg(REG_SETUP_RETR, 0x1A);              // 500uS timeout, 10 retries
  nrfWrite1Reg(REG_RF_CH, RF_BIND_CHANNEL);            // Channel 0x3C
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
  
  
}


void bind_rf() {
    
    bool bind = false;

  // Set device to bind address
  nrfWriteReg( REG_RX_ADDR_P0, (char *) rf_addr_bind, 5);
  nrfWriteReg( REG_TX_ADDR, (char *) rf_addr_bind, 5);
  
  // Power up
  nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX));
  
  while(!bind) {
    
    // Wait until we receive a data packet
    while(!(nrfGetStatus() & 0x40));
  
    
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
        while(!(nrfGetStatus() & 0x40));
        
        bind = true;
				
              
      }

}




#endif


// Place RF command data in RXcommand variable, process AUX commands
void rx_rf(int16_t *RXcommands) {
    
    float ratemul;

    uint16_t throttle, ailerons, rudder, elevator, mode;
    
    // If a new packet exists in the buffer
    if(nrfGetStatus() & 0x40)
    {
        // Read the latest command to the buffer
        nrfReadRX(rxbuffer, PAYLOADSIZE);
        
        // Flush the buffer and clear interrupt
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
        
        #ifdef RF_PROTO_REDV1   
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
        #endif
    
    
        #ifdef RF_PROTO_BLUE
        // Frequency hop
        CX10_current_chan %= CX10_NUM_RF_CHANNELS;
        nrfWrite1Reg(REG_RF_CH, CX10_freq[CX10_current_chan++]);
        
        // FC order: T   A   E   R   A1   A2
        // RF order: T    R   ?   E   A     Et    At   F   ?
      
        // PPM firmware expects a range of 1000, TX range is 1000 (1000-3000), mid-stick is 1500.
        throttle = (uint16_t)rxbuffer[14] << 8 | (uint16_t)rxbuffer[13];
        ailerons = (uint16_t)rxbuffer[10] << 8 | (uint16_t)rxbuffer[9];
        elevator = (uint16_t)rxbuffer[12] << 8 | (uint16_t)rxbuffer[11];
        rudder   = (uint16_t)(rxbuffer[16] & 0x0F) << 8 | (uint16_t)rxbuffer[15];
        mode     = (uint8_t)rxbuffer[17];
        
        RXcommands[0] = constrain(throttle-1000, 0, 1000);
        RXcommands[1] = -constrain((int16_t)ailerons-1500, -500, 500);
        RXcommands[2] = -constrain((int16_t)elevator-1500, -500, 500);
        RXcommands[3] = constrain((int16_t)rudder-1500, -500, 500);
        
         // Forward flip sets AUX1 high, backwards flip sets AUX1 low
        if( rxbuffer[16] & 0x10 )
        {
         if(RXcommands[2] >  450) RXcommands[4] = 500;
         if(RXcommands[2] < -450) RXcommands[4] = -500;
        }
        
        // Implement rates to match CX10 RED
        switch(mode)
        {
            case 0x00:
                ratemul = 0.33f;
                break;
            case 0x01:
                ratemul = 0.66f;
                break;
            default:
                ratemul = 1.0f;
        }
        
  
        for(int i = 1; i < 4; i++)
        {
            RXcommands[i] *= ratemul;
        }
        #endif
       
        // Since data has been received, reset failsafe counter
        failsafe = 0;
    }
    
}




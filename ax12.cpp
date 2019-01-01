/*
  ax12.cpp - ArbotiX library for control of Dynamixel servos using Protocol 2.0
  (such as the XL-320).
  
  Code modified by Albert Tai from vanilla Arbotix ax12.cpp file by Michael E.
  Ferguson. 
*/

#include "ax12.h"

/******************************************************************************
 * Hardware Serial Level, this uses the same stuff as Serial1, therefore 
 *  you should not use the Arduino Serial1 library.
 */

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];

// making these volatile keeps the compiler from optimizing loops of available()
volatile int ax_rx_Pointer;
volatile int ax_tx_Pointer;
volatile int ax_rx_int_Pointer;
#if defined(AX_RX_SWITCHED)
unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif

/** helper functions to switch direction of comms */
void setTX(int id){
    bitClear(UCSR1B, RXEN1); 
  #if defined(AX_RX_SWITCHED)
    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_WR;
    else
        SET_AX_WR;   
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif   
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    ax_tx_Pointer = 0;
}
void setRX(int id){ 
  #if defined(AX_RX_SWITCHED)
    int i;
    // Need to wait for last byte to be sent before turning the bus around.
    // Check the Transmit complete flag
    while (bit_is_clear(UCSR1A, UDRE1));
    for(i=0; i<UBRR1L*15; i++)    
        asm("nop");
    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_RD;
    else
        SET_AX_RD;
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      int i;
      // Need to wait for last byte to be sent before turning the bus around.
      // Check the Transmit complete flag
      while (bit_is_clear(UCSR1A, UDRE1));
      for(i=0; i<25; i++)    
          asm("nop");
      PORTD &= 0xEF;
    #endif 
    bitClear(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXCIE1);
  #endif  
    bitSet(UCSR1B, RXEN1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
}
// for sync write
void setTXall(){
    bitClear(UCSR1B, RXEN1);    
  #if defined(AX_RX_SWITCHED)
    SET_RX_WR;
    SET_AX_WR;   
  #else
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    ax_tx_Pointer = 0;
}

/** Sends a character out the serial port. */
void ax12write(unsigned char data){
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
    ax_tx_buffer[(ax_tx_Pointer++)] = data; 
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
ISR(USART1_RX_vect){
    ax_rx_int_buffer[(ax_rx_int_Pointer++)] = UDR1;
}

/** read back the error code for our latest packet read */
int ax12Error;
int ax12GetLastError(){ return ax12Error; }
/** > 0 = success */
int ax12ReadPacket(int length){
    unsigned long ulCounter;
    unsigned char offset, blength, checksum, timeout;
    unsigned char volatile bcount; 

    offset = 0;
    timeout = 0;
    bcount = 0;
    while(bcount < length){
        ulCounter = 0;
        while((bcount + offset) == ax_rx_int_Pointer){
            if(ulCounter++ > 1000L){ // was 3000
                timeout = 1;
                break;
            }
        }
        if(timeout) break;
        ax_rx_buffer[bcount] = ax_rx_int_buffer[bcount + offset];
        if((bcount == 0) && (ax_rx_buffer[0] != 0xff))
            offset++;
        else if((bcount == 2) && (ax_rx_buffer[2] == 0xff))
            offset++;
        else
            bcount++;
    }

    blength = bcount;
    checksum = 0;
    for(offset=2;offset<bcount;offset++)
        checksum += ax_rx_buffer[offset];
    if((checksum%256) != 255){
        return 0;
    }else{
        return 1;
    }
}

/** initializes serial1 transmit at baud, 8-N-1 */
void ax12Init(long baud){
    UBRR1H = (F_CPU / (8 * baud) - 1 ) >> 8;
    UBRR1L = (F_CPU / (8 * baud) - 1 );
    bitSet(UCSR1A, U2X1);
    ax_rx_int_Pointer = 0;
    ax_rx_Pointer = 0;
    ax_tx_Pointer = 0;
#if defined(AX_RX_SWITCHED)
    INIT_AX_RX;
    bitSet(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXEN1);
    bitSet(UCSR1B, RXCIE1);
#else
  #ifdef ARBOTIX_WITH_RX
    DDRD |= 0x10;   // Servo B = output
    PORTD &= 0xEF;  // Servo B low
  #endif
    // set RX as pull up to hold bus to a known level
    PORTD |= (1<<2);
    // enable rx
    setRX(0);
#endif
}

/** generates crc values for packets */
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for (j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length){  
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(regstart);
    ax12writeB(length);
    ax12writeB(checksum);  
    setRX(id);    
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

/* Set the value of a single-byte register. */
void ax12SetRegister(int id, int regstart, int data){
    setTX(id);    
	
	unsigned char packetdata[14];
	
	packetdata[0] = (0xFF);
	packetdata[1] = (0xFF);
	packetdata[2] = (0xFD);
	packetdata[3] = (0x00);
	packetdata[4] = (id);
	packetdata[5] = (7);
	packetdata[6] = (7 >> 8);
	packetdata[7] = (AX_WRITE_DATA);
	packetdata[8] = (regstart);
	packetdata[9] = (regstart >> 8);
	packetdata[10] = (data & 0xff);
	packetdata[11] = ((data & 0xff00) >> 8);

	unsigned short crc = update_crc(0, packetdata, 12);

	packetdata[12] = crc & 0xff;
	packetdata[13] = (crc >> 8) & 0xff;

	ax12writeB(packetdata[0]);
	ax12writeB(packetdata[1]);
	ax12writeB(packetdata[2]);
	ax12writeB(packetdata[3]);
	ax12writeB(packetdata[4]);
	ax12writeB(packetdata[5]);
	ax12writeB(packetdata[6]);
	ax12writeB(packetdata[7]);
	ax12writeB(packetdata[8]);
	ax12writeB(packetdata[9]);
	ax12writeB(packetdata[10]);
	ax12writeB(packetdata[11]);
	ax12writeB(packetdata[12]);
	ax12writeB(packetdata[13]);

	setRX(id);
/*
    ax12writeB(0xFF);
    ax12writeB(0xFF);
	ax12writeB(0xFD);
	ax12writeB(0x00);
    ax12writeB(id);
    ax12writeB(7);    // length
	ax12writeB(7 >> 8);
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
	ax12writeB(regstart >> 8);
    ax12writeB(data&0xff);
	ax12writeB((data & 0xff00) >> 8);
    // checksum = 
	ax12writeB(checksum);
	ax12writeB(checksum >> 8);
    setRX(id);
    //ax12ReadPacket();
*/
}

/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id); 
	
	unsigned char packetdata[16];

	packetdata[0] = (0xFF);
	packetdata[1] = (0xFF);
	packetdata[2] = (0xFD);
	packetdata[3] = (0x00);
	packetdata[4] = (id);
	packetdata[5] = (9);
	packetdata[6] = (9 >> 8);
	packetdata[7] = (AX_WRITE_DATA);
	packetdata[8] = (regstart);
	packetdata[9] = (regstart >> 8);
	packetdata[10] = (data & 0xff);
	packetdata[11] = ((data & 0xff00) >> 8);
	packetdata[12] = (((data & 0xff0000) >> 8) >> 8);
	packetdata[13] = ((((data & 0xff000000) >> 8) >> 8) >> 8);
	
	unsigned short crc = update_crc(0, packetdata, 14);

	packetdata[14] = crc & 0xff;
	packetdata[15] = (crc >> 8) & 0xff;
    
	ax12writeB(packetdata[0]);
	ax12writeB(packetdata[1]);
	ax12writeB(packetdata[2]);
	ax12writeB(packetdata[3]);
	ax12writeB(packetdata[4]);
	ax12writeB(packetdata[5]);
	ax12writeB(packetdata[6]);
	ax12writeB(packetdata[7]);
	ax12writeB(packetdata[8]);
	ax12writeB(packetdata[9]);
	ax12writeB(packetdata[10]);
	ax12writeB(packetdata[11]);
	ax12writeB(packetdata[12]);
	ax12writeB(packetdata[13]);
	ax12writeB(packetdata[14]);
	ax12writeB(packetdata[15]);
	
	setRX(id);

	/*
    ax12writeB(0xFF);
    ax12writeB(0xFF);
	ax12writeB(0xFD);
	ax12writeB(0x00);
    ax12writeB(id);
    ax12writeB(9);    // length
	ax12writeB(9>>8);
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
	ax12writeB(regstart>>8);
    ax12writeB(data&0xff);
	ax12writeB((data&0xff00) >> 8);
	ax12writeB(((data&0xff00) >> 8) >> 8);
	ax12writeB((((data&0xff00) >> 8) >> 8) >> 8);
    // checksum = 
    ax12writeB(checksum);
	ax12writeB(checksum>>8);
    setRX(id);
    //ax12ReadPacket();
	*/
}

// general write?
// general sync write?


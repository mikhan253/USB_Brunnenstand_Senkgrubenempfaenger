/**********************************************************************************
 ** Initialisierung System und GPIO                                              **
 **********************************************************************************/
/**********************************************************************************
 ** Verwendete Header                                                            **
 **********************************************************************************/
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "DrvSYS.h"
#include "DrvSPI.h"
#include <util/delay.h>
#include "HlDrvRFM23.h"
#if DEBUG
    #include <stdio.h>
#endif
/**********************************************************************************
 ** Macros und Definitionen                                                      **
 **********************************************************************************/
const uint8_t HlDrvRFM23_InitData[] PROGMEM = {
 /* Size  Addr   Data */
    5,    0x05,  0x06,0x00,           //05h-06h Interrupt Enable --> Packet Sent, Valid Packet Received
                 0x01,0x00,           //Operating Mode --> Enable READY Mode
				 0x7f,                //30 MHz Crystal Oscillator Load Capacitance (ODER EVTL D7 laut APPNOTE)
    2,    0x0b,  0xf5,                //GPIO0 RX State
                 0xf2,                //GPIO1 TX State
    3,    0x1c,  0x04,                //IF Filter Bandwidth
                 0x40,                //AFC Loop Gearshift Override --> AFC Enable
                 0x08,                //AFC Timing Control
	6,    0x20,  0x41,                //Clock Recovery Oversampling Rate
	             0x60,0x27,0x52,      //Clock Recovery Offset
				 0x00,0x06,           //Clock Recovery Timing Loop Gain
    6,    0x30,  0x8c,                //Data Access Control --> Packet RX/TX Handling, CRC Enable
				 0x00,                //**ignore**
                 0xff,0x42,           //Header Control --> Broadcast and Received Header Check, Headerlen 4, Synclen 2
                 64,                  //Preamble Length --> 64
                 0x20,                //Preamble Detection Control 1
   13,    0x3a,  'm','i','k','h',     //Transmit Header
                 RFM23_PACKETLENGTH,  //Packet Length
				 'm','i','k','h',     //Check Header
				 0xff,0xff,0xff,0xff, //Header Enable
    1,    0x57,  0x01,                //Appnote fix for Si4331 A0
    2,    0x59,  0x00,0x01,           //Appnote fix for Si4331 A0
    1,    0x6a,  0x0b,                //AGC Override 2
   11,    0x6d,  0x03,                //TX Power 13dBm
                 0x27,0x52,           //TX Data Rate
				 0x20,0x22,           //Modulation Mode Control --> Datarate <30kbps, FSK, FIFO Mode
                 0x48,                //Frequency Deviation
                 0x00,0x00,           //Frequency Offset
				 0x53,                //Frequency Band Select
				 0x64,0x00,           //Nominal Carrier Frequency
    0}; //EOF

/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/
uint8_t HlDrvRFM23_DataBuffer[RFM23_PACKETLENGTH];
/* im idle-mode soll er im ready mode sein */

#include <avr/interrupt.h>
#include <avr/sleep.h>

uint8_t HlDrvRFM23_Enable(void)
{
    DrvPWR_ModuleEnable(PRR_PCI);
    PCICR = (1<<PCIE2);
    PCMSK2 = (1<<PCINT19); 
    sei();

         HlDrvGPIO_RFM23_Enable();
    do sleep_mode(); while((PIND & _BV(PD3)));
    //while((PIND & _BV(PD3))) //HIER DEN INTERRUPT EINFÜGEN UND SCHLAFEN
    //    asm volatile ("nop");
    if(HlDrvRFM23_Read(0x04) != 0x01) //Kein POR??
        return -1;
    uint8_t dataptr=0;
    uint8_t count=0;
    while((count = pgm_read_byte(&HlDrvRFM23_InitData[dataptr++]))) { //count>0
        DrvSPI_SSON();
        DrvSPI_transferByte(pgm_read_byte(&HlDrvRFM23_InitData[dataptr++]) | _BV(7)); //Addresse        
        while(count--)
            DrvSPI_transferByte(pgm_read_byte(&HlDrvRFM23_InitData[dataptr++])); //Daten
        DrvSPI_SSOFF();
    }
    asm volatile ("nop");
    _delay_ms(10); //AUS IRGEND EINEM GRUND MUSS DAS DRINNEN SEIN???
//    printf("Register DUMP after INIT:\n"); //DEBUG
//    rfm23_temp(); //DEBUG
    return 0;
}

uint8_t HlDrvRFM23_ReceiveData(void) {
    HlDrvRFM23_Write(0x08,0x03); //Clear FIFO
    HlDrvRFM23_Write(0x08,0x00);
    HlDrvRFM23_Write(0x07,0x05); //RX Mode
    while((PIND & _BV(PD3))) //HIER DEN INTERRUPT EINFÜGEN UND SCHLAFEN
        asm volatile ("nop");
    if(HlDrvRFM23_Read(0x03) != 0x02) //Valid Packet Received
        return -1;
    HlDrvRFM23_MultipleTransaction(0,RFM23_PACKETLENGTH,0x7F,HlDrvRFM23_DataBuffer);
    return 0;
}

uint8_t HlDrvRFM23_TransmitData(void) {
    HlDrvRFM23_Write(0x08,0x03); //Clear FIFO
    HlDrvRFM23_Write(0x08,0x00);
    HlDrvRFM23_Write(0x07,0x09); //TX Mode
    HlDrvRFM23_MultipleTransaction(1,RFM23_PACKETLENGTH,0x7F,HlDrvRFM23_DataBuffer);
    while((PIND & _BV(PD3))) //HIER DEN INTERRUPT EINFÜGEN UND SCHLAFEN
        asm volatile ("nop");
    if(HlDrvRFM23_Read(0x03) != 0x24) //Valid Sent Interrupt
        return -1;
    return 0;
}

#if DEBUG
uint8_t rfm23_temp()
{
    uint8_t i, max=0x7F,x;
	for (i=0; i<=max; i++)
    {
        x=HlDrvRFM23_Read(i);
        printf("addr=%02X, data=%02X\n",i,x);
    }
}
#endif

void HlDrvRFM23_Write(uint8_t addr, uint8_t val) {
	addr |= _BV(7);
	DrvSPI_SSON();
	DrvSPI_transferByte(addr);
	DrvSPI_transferByte(val);
	DrvSPI_SSOFF();
}

void HlDrvRFM23_MultipleTransaction(uint8_t write, uint8_t size, uint8_t addr, uint8_t* val) {
    if(write)
        addr |= _BV(7);
    else
        addr &= ~_BV(7);
    DrvSPI_SSON();
    DrvSPI_transferByte(addr);
    DrvSPI_transferBuffer(val,size);
    DrvSPI_SSOFF();
}

uint8_t HlDrvRFM23_Read(uint8_t addr) {
	addr &= ~_BV(7);
	DrvSPI_SSON();
	DrvSPI_transferByte(addr);
	uint8_t val = DrvSPI_transferByte(0x00);
	DrvSPI_SSOFF();
	return val;
}
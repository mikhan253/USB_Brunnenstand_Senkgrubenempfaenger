/*****************************************************************************
 ** @file DrvTWI.h                                                          **
 ** @brief Hardware TWI Treiber                                             **
 *****************************************************************************/

/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include "DrvTWI.h"
#include "DrvUSART.h"
/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

// Standard I2C bit rates are:
// 100KHz for slow speed
// 400KHz for high speed

#ifndef TWI_BUF_SIZE
#define TWI_BUF_SIZE 0x8
#endif

// I2C state and address variables
static volatile emTWIState twiState;
static uint8_t TWI_DeviceAddrRW;
// I2C transceiver data buffer
static uint8_t TWI_DataBuffer[TWI_BUF_SIZE];
static uint8_t TWI_BufferIndex;
static uint8_t TWI_BufferLength;

// functions
void DrvTWI_Init(void)
{
    DrvPWR_ModuleEnable(PRR_TWI);
    TWBR = INIT_TWBR;
    TWSR = INIT_TWPS;
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
    twiState = I2C_IDLE;
    sei();
}

void DrvTWI_Deinit(void)
{
    TWCR = 0;
    cli();
    DrvPWR_ModuleDisable(PRR_TWI);
}

void DrvTWI_MasterSend(uint8_t deviceAddr, uint8_t length, uint8_t *data)
{
    uint8_t i;
    // wait for interface to be ready
    while (twiState)
        ;
    // set state
    twiState = I2C_MASTER_TX;
    // save data
    TWI_DeviceAddrRW = (deviceAddr & 0xFE); // RW cleared: write operation
    for (i = 0; i < length; i++)
        TWI_DataBuffer[i] = *data++;
    TWI_BufferIndex = 0;
    TWI_BufferLength = length;
    // send start condition
    DrvTWI_SendStart();
}

void DrvTWI_MasterReceive(uint8_t deviceAddr, uint8_t length, uint8_t *data)
{
    uint8_t i;
    // wait for interface to be ready
    while (twiState)
        ;
    // set state
    twiState = I2C_MASTER_RX;
    // save data
    TWI_DeviceAddrRW = (deviceAddr | 0x01); // RW set: read operation
    TWI_BufferIndex = 0;
    TWI_BufferLength = length;
    // send start condition
    DrvTWI_SendStart();
    // wait for data
    while (twiState)
        ;
    // return data
    for (i = 0; i < length; i++)
        *data++ = TWI_DataBuffer[i];
}

//! I2C (TWI) interrupt service routine
ISR(TWI_vect)
{
    // read status bits
    uint8_t status = TWSR & TWSR_STATUS_MASK;
    switch (status)
    {
    // Master General
    case TW_START:     // 0x08: Sent start condition
    case TW_REP_START: // 0x10: Sent repeated start condition
        // send device address
        DrvTWI_SendByte(TWI_DeviceAddrRW);
        break;

    // Master Transmitter & Receiver status codes
    case TW_MT_SLA_ACK:  // 0x18: Slave address acknowledged
    case TW_MT_DATA_ACK: // 0x28: Data acknowledged
        if (TWI_BufferIndex < TWI_BufferLength)
        {
            // send data
            DrvTWI_SendByte(TWI_DataBuffer[TWI_BufferIndex++]);
        }
        else
        {
            // transmit stop condition, enable SLA ACK
            DrvTWI_SendStop();
            // set state
            twiState = I2C_IDLE;
        }
        break;
    case TW_MR_DATA_NACK: // 0x58: Data received, NACK reply issued
        // store final received data byte
        TWI_DataBuffer[TWI_BufferIndex++] = TWDR;
        // continue to transmit STOP condition
    case TW_MR_SLA_NACK:  // 0x48: Slave address not acknowledged
    case TW_MT_SLA_NACK:  // 0x20: Slave address not acknowledged
    case TW_MT_DATA_NACK: // 0x30: Data not acknowledged
        // transmit stop condition, enable SLA ACK
        DrvTWI_SendStop();
        // set state
        twiState = I2C_IDLE;
        break;
    case TW_MT_ARB_LOST: // 0x38: Bus arbitration lost
                         // case TW_MR_ARB_LOST:				// 0x38: Bus arbitration lost
        //  release bus
        TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT);
        // set state
        twiState = I2C_IDLE;
        // release bus and transmit start when bus is free
        // outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT)|_BV(TWSTA));
        break;
    case TW_MR_DATA_ACK: // 0x50: Data acknowledged
        // store received data byte
        TWI_DataBuffer[TWI_BufferIndex++] = TWDR;
        // fall-through to see if more bytes will be received
    case TW_MR_SLA_ACK: // 0x40: Slave address acknowledged
        if (TWI_BufferIndex < (TWI_BufferLength - 1))
            // data byte will be received, reply with ACK (more bytes in transfer)
            DrvTWI_ReceiveByte(1);
        else
            // data byte will be received, reply with NACK (final byte in transfer)
            DrvTWI_ReceiveByte(0);
        break;
    // Misc
    case TW_NO_INFO: // 0xF8: No relevant state information
        // do nothing
        break;
    case TW_BUS_ERROR: // 0x00: Bus error due to illegal start or stop condition
        // reset internal hardware and release bus
        TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT) | _BV(TWSTO) | _BV(TWEA);
        // set state
        twiState = I2C_IDLE;
        break;
    }
}

emTWIState DrvTWI_GetState(void)
{
    return twiState;
}

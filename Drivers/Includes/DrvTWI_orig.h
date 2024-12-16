/*****************************************************************************
 ** @file DrvTWI.h                                                          **
 ** @brief Hardware TWI Treiber                                             **
 *****************************************************************************/
#ifndef _DrvTWI_H_
#define _DrvTWI_H_

/*****************************************************************************
 *  Konfiguration                                                            *
 *****************************************************************************/

#define INIT_TWBR 7 /*400kHz*/
#define INIT_TWPS 1  /*400kHz*/
/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/
#include "lgt8f328p_spec.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "DrvSYS.h"

// TWSR values (not bits)
// Master
#define TW_START 0x08
#define TW_REP_START 0x10
// Master Transmitter
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
#define TW_MT_DATA_NACK 0x30
#define TW_MT_ARB_LOST 0x38
// Master Receiver
#define TW_MR_ARB_LOST 0x38
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58
// Slave Transmitter
#define TW_ST_SLA_ACK 0xA8
#define TW_ST_ARB_LOST_SLA_ACK 0xB0
#define TW_ST_DATA_ACK 0xB8
#define TW_ST_DATA_NACK 0xC0
#define TW_ST_LAST_DATA 0xC8
// Slave Receiver
#define TW_SR_SLA_ACK 0x60
#define TW_SR_ARB_LOST_SLA_ACK 0x68
#define TW_SR_GCALL_ACK 0x70
#define TW_SR_ARB_LOST_GCALL_ACK 0x78
#define TW_SR_DATA_ACK 0x80
#define TW_SR_DATA_NACK 0x88
#define TW_SR_GCALL_DATA_ACK 0x90
#define TW_SR_GCALL_DATA_NACK 0x98
#define TW_SR_STOP 0xA0
// Misc
#define TW_NO_INFO 0xF8
#define TW_BUS_ERROR 0x00

// defines and constants
#define TWCR_CMD_MASK 0x0F
#define TWSR_STATUS_MASK 0xF8

// return values
#define I2C_OK 0x00
#define I2C_ERROR_NODEV 0x01

// types
typedef enum
{
    I2C_IDLE = 0,
    I2C_BUSY = 1,
    I2C_MASTER_TX = 2,
    I2C_MASTER_RX = 3
} emTWIState;

// Low-level I2C transaction commands

#define DrvTWI_ENINT()      \
    do                      \
    {                       \
        TWCR &= ~_BV(TWIE); \
    } while (0)

#define DrvTWI_DISINT()    \
    do                     \
    {                      \
        TWCR |= _BV(TWIE); \
    } while (0)

//! Send an I2C start condition in Master mode
#define DrvTWI_SendStart()                                       \
    do                                                           \
    {                                                            \
        TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT) | _BV(TWSTA); \
    } while (0)

//! Send an I2C stop condition in Master mode
#define DrvTWI_SendStop()                                                    \
    do                                                                       \
    {                                                                        \
        TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO); \
    } while (0)

//! Wait for current I2C operation to complete
#define DrvTWI_WaitForComplete() while (!(TWCR & _BV(TWINT)))

//! Send an (address|R/W) combination or a data byte over I2C
#define DrvTWI_SendByte(data)                       \
    do                                              \
    {                                               \
        TWDR = data;                                \
        TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT); \
    } while (0)

//! Receive a data byte over I2C
// ackFlag = 1 if recevied data should be ACK'ed
// ackFlag = 0 if recevied data should be NACK'ed
#define DrvTWI_ReceiveByte(ackFlag)                                 \
    do                                                              \
    {                                                               \
        if (ackFlag)                                                \
        {                                                           \
            TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT) | _BV(TWEA); \
        }                                                           \
        else                                                        \
        {                                                           \
            TWCR = (TWCR & TWCR_CMD_MASK) | _BV(TWINT);             \
        }                                                           \
    } while (0)

//! Pick up the data that was received with i2cReceiveByte()
#define DrvTWI_GetReceivedByte() (TWDR)

//! Get current I2c bus status from TWSR
#define DrvTWI_GetStatus() (TWSR)

//! Initialize I2C (TWI) interface
void DrvTWI_Init();
void DrvTWI_Deinit();

//! send I2C data to a device on the bus
void DrvTWI_MasterSend(uint8_t deviceAddr, uint8_t length, uint8_t *data);
//! receive I2C data from a device on the bus
void DrvTWI_MasterReceive(uint8_t deviceAddr, uint8_t length, uint8_t *data);

//! Get the current high-level state of the I2C interface
emTWIState DrvTWI_GetState(void);

#endif
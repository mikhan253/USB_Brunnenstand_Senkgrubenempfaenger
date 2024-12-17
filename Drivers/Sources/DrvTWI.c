/*************************************************************************
 * Title:    I2C master library using hardware TWI interface
 * Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 * File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
 * Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
 * Target:   any AVR device with hardware TWI
 * Usage:    API compatible with I2C Software Library i2cmaster.h
 **************************************************************************/
#include <inttypes.h>
#include <compat/twi.h>
// #include "debugPrint.h"
#include "DrvTWI.h"
#include "DrvSYS.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void DrvTWI_Init(uint8_t twbrset)
{
    DrvPWR_ModuleEnable(PRR_TWI);

    /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
    TWSR = 0; /* no prescaler */
    TWBR = twbrset;
    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();
    // TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
} /* i2c_init */

void DrvTWI_Deinit()
{
    cli();
    TWCR = 0;
    DrvPWR_ModuleDisable(PRR_TWI);
} /* i2c_init */

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
uint8_t DrvTWI_Start(uint8_t address)
{
    uint8_t twst;

    // send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);

    // wait until  START condition has been transmitted
    do
        sleep_mode();
    while (!(TWCR & (1 << TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ((twst != TW_START) && (twst != TW_REP_START))
        return 1;

    // send device address
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
    // wail until transmission completed and ACK/NACK has been received
    do
        sleep_mode();
    while (!(TWCR & (1 << TWINT)));
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK))
    {
        //		debug_str("\ni2c_start(): !!! com error !!!\n");
        return 1;
    }
    return 0;
} /* i2c_start */

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 Input:   address and transfer direction of I2C device
*************************************************************************/
void DrvTWI_StartWait(uint8_t address)
{
    uint8_t twst;
    while (1)
    {
        // send START condition
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);

        // wait until transmission completed
        do
            sleep_mode();
        while (!(TWCR & (1 << TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ((twst != TW_START) && (twst != TW_REP_START))
            continue;

        // send device address
        TWDR = address;
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);

        // wail until transmission completed
        do
            sleep_mode();
        while (!(TWCR & (1 << TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ((twst == TW_MT_SLA_NACK) || (twst == TW_MR_DATA_NACK))
        {
            /* device busy, send stop condition to terminate write operation */
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

            // wait until stop condition is executed and bus released
            while (TWCR & (1 << TWSTO))
                ;

            continue;
        }
        // if( twst != TW_MT_SLA_ACK) return 1;
        break;
    }

} /* i2c_start_wait */

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction

 Input:   address and transfer direction of I2C device

 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
uint8_t DrvTWI_RepeatedStart(uint8_t address)
{
    return DrvTWI_Start(address);

} /* i2c_rep_start */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void DrvTWI_Stop(void)
{
    /* send stop condition */
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    // wait until stop condition is executed and bus released

    while (1)
    {
        if (!(TWCR & (1 << TWSTO)))
        {
            return;
        }
    }
    // debug_str("i2c_stop(): timeout\n");
} /* i2c_stop */

/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
uint8_t DrvTWI_Write(uint8_t data)
{
    uint8_t twst;
    // send data to the previously addressed device
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);

    // wait until transmission completed
    // wait until  START condition has been transmitted
    do
        sleep_mode();
    while (!(TWCR & (1 << TWINT)));

    // check value of TWI Status Register. Mask prescaler bits
    twst = TW_STATUS & 0xF8;
    if (twst != TW_MT_DATA_ACK)
    {
        // debug_str("i2c_write(): com error\n");
        return 1;
    }
    return 0;

} /* i2c_write */

/*************************************************************************
 Read one byte from the I2C device, request more data from device

 Return:  byte read from I2C device
*************************************************************************/
uint8_t DrvTWI_ReadAck(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    do
        sleep_mode();
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
} /* i2c_readAck */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
uint8_t DrvTWI_ReadNAck(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
    do
        sleep_mode();
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
} /* i2c_readNak */

ISR(TWI_vect)
{
    return;
}
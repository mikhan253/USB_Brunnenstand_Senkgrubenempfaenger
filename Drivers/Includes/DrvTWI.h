#ifndef _I2CMASTER_H
#define _I2CMASTER_H 1
/*************************************************************************
 * Title:    C include file for the I2C master interface
 *           (i2cmaster.S or twimaster.c)
 * Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 *           Modified by M. Betz
 **************************************************************************/
#include <avr/io.h>

// I2C clock in Hz
#define SCL_CLOCK 400000L

/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ 1

/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE 0

/**
 @brief initialize the I2C master interace. Need to be called only once
 @param  void
 @return none
 */
extern void DrvTWI_Init(uint8_t twbrset);

extern void DrvTWI_Deinit();

/**
 @brief Terminates the data transfer and releases the I2C bus
 @param void
 @return none
 */
extern void DrvTWI_Stop(void);

/**
 @brief Issues a start condition and sends address and transfer direction

 @param    addr address and transfer direction of I2C device
 @retval   0   device accessible
 @retval   1   failed to access device
 */
extern uint8_t DrvTWI_Start(uint8_t addr);

/**
 @brief Issues a repeated start condition and sends address and transfer direction

 @param   addr address and transfer direction of I2C device
 @retval  0 device accessible
 @retval  1 failed to access device
 */
extern uint8_t DrvTWI_RepeatedStart(uint8_t addr);

/**
 @brief Issues a start condition and sends address and transfer direction

 If device is busy, use ack polling to wait until device ready
 @param    addr address and transfer direction of I2C device
 @return   none
 */
extern void DrvTWI_StartWait(uint8_t addr);

/**
 @brief Send one byte to I2C device
 @param    data  byte to be transfered
 @retval   0 write successful
 @retval   1 write failed
 */
extern uint8_t DrvTWI_Write(uint8_t data);

/**
 @brief    read one byte from the I2C device, request more data from device
 @return   byte read from I2C device
 */
extern uint8_t DrvTWI_ReadAck(void);

/**
 @brief    read one byte from the I2C device, read is followed by a stop condition
 @return   byte read from I2C device
 */
extern uint8_t DrvTWI_ReadNAck(void);

/**
 @brief    read one byte from the I2C device

 Implemented as a macro, which calls either i2c_readAck or i2c_readNak

 @param    ack 1 send ack, request more data from device<br>
               0 send nak, read is followed by a stop condition
 @return   byte read from I2C device
 */
extern uint8_t i2c_read(uint8_t ack);
#define i2c_read(ack) (ack) ? i2c_readAck() : i2c_readNak();

// Print a list of connected I2C devices (right shifted 7 bit addresses in hex)
void searchI2C();

/**@}*/
#endif
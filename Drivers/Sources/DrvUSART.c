/*****************************************************************************
 ** @file DrvGPIO.h                                                         **
 ** @brief Hardware System, GPIO Treiber                                    **
 *****************************************************************************/
/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
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

void DrvUSART_PutChar(uint8_t send)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = send;
}

/**
 * @fn u8 DrvUSART_RecvChar(void)
 */
uint8_t DrvUSART_GetChar(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;
    return UDR0;
}

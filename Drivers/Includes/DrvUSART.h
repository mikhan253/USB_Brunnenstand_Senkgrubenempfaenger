/*****************************************************************************
 ** @file DrvUSART.h                                                        **
 ** @brief Hardware USART Treiber                                           **
 *****************************************************************************/
#ifndef _USART_H_
#define _USART_H_
/*****************************************************************************
 *  Konfiguration                                                            *
 *****************************************************************************/
#define USART_UMSEL0 0x0 // Asynchronous USART
#define USART_RXEN 1
#define USART_RXDIO 0x0 // RXD on PD0
#define USART_TXEN 1
#define USART_TXDIO 0x0   // TXD on PD1
#define USART_BDR0 115200 // 115200bps
#define USART_UCSZ0 0x3   // 8bit data
#define USART_USBS0 0x0   // one stop bit
#define USART_UPM0 0x2    // Enable Even Parity
#define USART_UCPOL0 0x0  // Rise edge of XCK1
#define USART_U2X0 0
#define USART_MPCM0 0
#define USART_RXC 0
#define USART_TXC 0
#define USART_UDRE 0

/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include <avr/io.h>

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/
#define E_UMSEL0_UART 0 /**< Asynchronous USART */
#define E_UMSEL0_USRT 1 /**< Synchronous USART */
#define E_UMSEL0_SPIS 2 /**< Slave SPI */
#define E_UMSEL0_SPIM 3 /**< Master SPI */

#if (USART_UMSEL0 == E_UMSEL0_UART)
#if (USART_U2X0)
#define USART_UBRR (uint16_t)(((((F_CPU / USART_BDR0) >> 2) + 1) >> 1) - 1)
#else
#define USART_UBRR (uint16_t)(((((F_CPU / USART_BDR0) >> 3) + 1) >> 1) - 1)
#endif
#else
#define USART_UBRR (uint16_t)(((((F_CPU / USART_BDR)) + 1) >> 1) - 1)
#endif
#define USART_TXREN ((USART_RXEN << 4) | (USART_TXEN << 3))

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

/**
 * @fn static inline void DrvUSART_Init()
 * @brief Initialisiert USART
 **/
static inline void DrvUSART_Init()
{
    UCSR0A = (USART_MPCM0 << MPCM0) | (USART_U2X0 << U2X0);

    UCSR0C = (USART_UMSEL0 << UMSEL00) | (USART_UPM0 << UPM00) | (USART_USBS0 << USBS0) |
             ((USART_UCSZ0 & 3) << UCSZ00) | (USART_UCPOL0 << UCPOL0);
    UCSR0B = USART_TXREN | (USART_UCSZ0 & 4) | (USART_RXC << RXCIE0) | (USART_TXC << TXCIE0) | (USART_UDRE << UDRIE0);

    UBRR0H = (USART_UBRR >> 8) & 0xff;
    UBRR0L = USART_UBRR & 0xff;
}

/**
 * @fn void DrvUSART_PutChar(uint8_t send)
 * @brief Sendet ein Byte über USART
 **/
void DrvUSART_PutChar(uint8_t send);

/**
 * @fn uint8_t DrvUSART_GetChar(void)
 * @brief Empfängt ein Byte über USART
 **/
uint8_t DrvUSART_GetChar();

#endif
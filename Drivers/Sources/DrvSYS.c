/*****************************************************************************
 ** @file DrvSYS.c                                                          **
 ** @brief Hardware System, GPIO Treiber                                    **
 *****************************************************************************/

/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include "DrvSYS.h"
#include "lgt8f328p_spec.h"

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

void DrvSYS_Init()
{
    DrvSYS_SetMCLKDiv(INIT_MCK_CLKDIV);

    // Ports definieren
    DDRB = INIT_DDRB;
    PORTB = INIT_PORTB;
    DDRC = INIT_DDRC;
    PORTC = INIT_PORTC;
    DDRD = INIT_DDRD;
    PORTD = INIT_PORTD;
    DDRE = INIT_DDRE;
    PORTE = INIT_PORTE;
    DDRF = INIT_DDRF;
    PORTF = INIT_PORTF;

    // Power Reduce
    PRR10 = PRR_ADC | PRR_USART0 | PRR_SPI | PRR_TIM1 | PRR_TIM0 | PRR_TIM2 | PRR_TWI | PRR_PCI | PRR_EFL;

#ifdef DEBUG
    DrvPWR_ModuleEnable(PRR_USART0);
#endif
}

void DrvSYS_SetMCLKDiv(uint8_t div)
{
    CLKPR = _BV(WCE);
    CLKPR = _BV(WCE) | div;
}

void DrvSYS_Reset()
{
    VDTCR = _BV(WCE);
    VDTCR = _BV(WCE) | _BV(SWR);
}
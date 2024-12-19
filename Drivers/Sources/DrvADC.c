/*****************************************************************************
 ** @file DrvADC.c                                                          **
 ** @brief Hardware ADC Treiber                                             **
 *****************************************************************************/
/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "DrvADC.h"
#include "DrvSYS.h"
#include "lgt8f328p_spec.h"

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

void DrvADC_Init()
{
    // ADC Multiplexer Selection Register
    DrvPWR_ModuleEnable(PRR_ADC);
    VCAL = INIT_VCAL;
    ADMUX = INIT_ADC_CHMUX | INIT_ADMUX_REFS;
    ADCSRD = INIT_ADCSRD_REFS;
    ADCSRA = _BV(ADEN) | _BV(ADIE) | 5; // INT und 1MHz ADCCLK
    sei();
}

void DrvADC_Deinit()
{
    cli();
    ADCSRA = 0;
    DrvPWR_ModuleDisable(PRR_ADC);
}


uint16_t DrvADC_ReadData()
{
    uint16_t pVal, nVal;

    set_sleep_mode(SLEEP_MODE_ADC);

    // SPN1 conversation
    ADCSRC |= (1 << SPN);
    nVal = _adcRead() >> 1;

    // SPN0 conversation
    ADCSRC &= ~(1 << SPN);
    pVal = _adcRead() >> 1;

    // average
    pVal = pVal + nVal;

    // gain-error correction
    pVal -= (pVal >> 7);

    return pVal;
}

uint16_t _adcRead()
{
    uint16_t wtmp;

    // ADCSRA |= 0x40; Geht automatisch in den Startmodus
    // do
    //{
    sleep_mode();
    //} while ((ADCSRA & 0x40) != 0x00);

    wtmp = ADCL;

    return (ADCH << 8) | wtmp;
}

ISR(ADC_vect)
{
    return;
}
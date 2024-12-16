/*****************************************************************************
 ** @file DrvADC.c                                                          **
 ** @brief Hardware ADC Treiber                                             **
 *****************************************************************************/
/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include "DrvADC.h"
#ifdef LLDEBUG
#include "DrvUSART.h"
#endif

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

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

    //ADCSRA |= 0x40; Geht automatisch in den Startmodus
    //do
    //{
    sleep_mode();
    //} while ((ADCSRA & 0x40) != 0x00);

    wtmp = ADCL;

    return (ADCH << 8) | wtmp;
}

ISR(ADC_vect)
{
#ifdef LLDEBUG
    DrvUSART_PutChar('!');
#endif
    return;
}
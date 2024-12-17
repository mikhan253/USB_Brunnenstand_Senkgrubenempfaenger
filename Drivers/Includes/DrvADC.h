/*****************************************************************************
 ** @file DrvADC.h                                                          **
 ** @brief Hardware ADC Treiber                                             **
 *****************************************************************************/
#ifndef _DrvADC_H_
#define _DrvADC_H_

/*****************************************************************************
 *  Konfiguration                                                            *
 *****************************************************************************/
#define INIT_ADC_CHMUX 3
#define INIT_ADC_VREF_1V024 1

/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "DrvSYS.h"
#include "lgt8f328p_spec.h"

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/
#ifdef INIT_ADC_VREF_1V024
#define INIT_ADMUX_REFS (_BV(6) | _BV(7))
#define INIT_ADCSRD_REFS 0
#define INIT_VCAL VCAL1
#endif
/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

/**
 * @fn static inline void DrvADC_Init()
 * @brief Initialisiert ADC
 **/
static inline void DrvADC_Init()
{
    // ADC Multiplexer Selection Register
    DrvPWR_ModuleEnable(PRR_ADC);
    VCAL = INIT_VCAL;
    ADMUX = INIT_ADC_CHMUX | INIT_ADMUX_REFS;
    ADCSRD = INIT_ADCSRD_REFS;
    ADCSRA = _BV(ADEN) | _BV(ADIE) | 5; // INT und 1MHz ADCCLK
    sei();
}

/**
 * @fn static inline void DrvADC_Deinit()
 * @brief Deinitialisiert ADC
 **/
static inline void DrvADC_Deinit()
{
    cli();
    ADCSRA = 0;
    DrvPWR_ModuleDisable(PRR_ADC);
}

/**
    @brief    Lese Wert von ADC
    Lese Wert von ADC auf aktuellem Kanal
    @param
    @return   ADC Wert
 **/
uint16_t DrvADC_ReadData();

uint16_t _adcRead();

#endif

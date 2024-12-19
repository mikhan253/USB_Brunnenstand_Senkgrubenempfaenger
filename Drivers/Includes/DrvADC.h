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
void DrvADC_Init();

/**
 * @fn static inline void DrvADC_Deinit()
 * @brief Deinitialisiert ADC
 **/
void DrvADC_Deinit();


/**
    @brief    Lese Wert von ADC
    Lese Wert von ADC auf aktuellem Kanal
    @param
    @return   ADC Wert
 **/
uint16_t DrvADC_ReadData();

uint16_t _adcRead();

#endif

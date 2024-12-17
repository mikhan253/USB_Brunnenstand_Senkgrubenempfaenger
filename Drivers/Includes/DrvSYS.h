/*****************************************************************************
 ** @file DrvSYS.h                                                          **
 ** @brief Hardware System, GPIO Treiber                                    **
 *****************************************************************************/
#ifndef _DrvSYS_H_
#define _DrvSYS_H_

/*****************************************************************************
 *  Konfiguration                                                            *
 *****************************************************************************/
// PB0 VL53L0X SHUTDOWN (LOW AKTIV)
// PB1 RFM23 SHUTDOWN (HIGH AKTIV)
//*PB2 RFM23 SPI CS    PB3 SPI MOSI    *PB4 SPI MISO    *PB5 SPI SCLK
// PC0 JUMPER BOOTLOADER
// PC1 JUMPER BOOTLOADER
// PC2 BATTERIESPANNUNG ENABLE (HIGH AKTIV)
//*PC3 (ADC3) BATTERIESPANNUNG
//*PC4 VL53L0X SDA     *PC5 VL53L0X SCL
//*PC6 RST
//*PD0 RX              *PD1 TX
// PD2 LED
// PD3 RF23M IRQ (LOW AKTIV)
// PD4 VL53L0X INT (LOW AKTIV)
// PD5 DONE (BATTERY TIMER --> POWER OFF)
// PD6 - PD7 -    PE0 DEBUG SWC    PE1 -    PE2 DEBUG SWD    PE3 - PF1 - PF2 -

// CLOCK
#define INIT_MCK_CLKDIV 0 /* 32MHz */

// GPIO PORTS
// Data Direction 0=Eingang 1=Ausgang
#define INIT_DDRB (_BV(0)) | (_BV(1)) | (_BV(2)) | (_BV(3)) | (0 & _BV(4)) | (_BV(5))
#define INIT_DDRC (_BV(0)) | (_BV(1)) | (_BV(2)) | (0 & _BV(3)) | (0 & _BV(4)) | (0 & _BV(5)) | (0 & _BV(6))
#define INIT_DDRD (0 & _BV(0)) | (0 & _BV(1)) | (_BV(2)) | (0 & _BV(3)) | (0 & _BV(4)) | (_BV(5)) | (_BV(6)) | (_BV(7))
#define INIT_DDRE (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2)) | (0 & _BV(3))
#define INIT_DDRF (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2))
// Port Register Startwerte 0=Low 1=High (Pull-up bei Eingang)
#define INIT_PORTB (0 & _BV(0)) | (_BV(1)) | (_BV(2)) | (0 & _BV(3)) | (0 & _BV(4)) | (0 & _BV(5))
#define INIT_PORTC (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2)) | (0 & _BV(3)) | (0 & _BV(4)) | (0 & _BV(5)) | (0 & _BV(6))
#define INIT_PORTD (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2)) | (_BV(3)) | (_BV(4)) | (0 & _BV(5)) | (0 & _BV(6)) | (0 & _BV(7))
#define INIT_PORTE (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2)) | (0 & _BV(3))
#define INIT_PORTF (0 & _BV(0)) | (0 & _BV(1)) | (0 & _BV(2))

/*****************************************************************************
 *  Headers                                                                  *
 *****************************************************************************/
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include "lgt8f328p_spec.h"

/*****************************************************************************
 *  Definitionen                                                             *
 *****************************************************************************/
#define PRR_ADC 0x0001
#define PRR_USART0 0x0002
#define PRR_SPI 0x0004
#define PRR_TIM1 0x0008
#define PRR_TIM0 0x0020
#define PRR_TIM2 0x0040
#define PRR_TWI 0x0080

#define PRR_PCI 0x0200
#define PRR_EFL 0x0400
#define PRR_WDT 0x2000

#ifndef PRR10
#define PRR10 (*(volatile uint16_t *)0x0064)
#endif

/*****************************************************************************
 *  Makros                                                                   *
 *****************************************************************************/
#define DrvPWR_SetMCLKDiv(div) \
    CLKPR = 0x80;              \
    CLKPR = 0x80 | (div & 0xf);
#define DrvPWR_ModuleEnable(ID) PRR10 &= ~ID
#define DrvPWR_ModuleDisable(ID) PRR10 |= ID
#define DrvGPIO_SetPin(BANK, NR) PORT##BANK |= _BV(NR)
#define DrvGPIO_ClearPin(BANK, NR) PORT##BANK &= ~_BV(NR)
#define DrvGPIO_TogglePin(BANK, NR) PORT##BANK ^= _BV(NR)
#define HlDrvGPIO_VL53L0X_Disable() PORTB &= ~_BV(0)
#define HlDrvGPIO_VL53L0X_Enable() PORTB |= _BV(0)
#define HlDrvGPIO_RFM23_Disable() PORTB |= _BV(1)
#define HlDrvGPIO_RFM23_Enable() PORTB &= ~_BV(1)
#define HlDrvGPIO_ADCVBATT_Disable() PORTC &= ~_BV(2)
#define HlDrvGPIO_ADCVBATT_Enable() PORTC |= _BV(2)
#define HlDrvGPIO_LED_Disable() PORTD &= ~_BV(2)
#define HlDrvGPIO_LED_Enable() PORTD |= _BV(2)
#define HlDrvGPIO_PowerOff() PORTD |= _BV(5)

/*****************************************************************************
 *  Funktionen                                                               *
 *****************************************************************************/

/**
 * @fn static inline void DrvSYS_Init()
 * @brief Initialisiert CLOCK und GPIO
 **/
static inline void DrvSYS_Init()
{
    DrvPWR_SetMCLKDiv(INIT_MCK_CLKDIV);

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
    PRR10 = PRR_ADC | PRR_USART0 | PRR_SPI | PRR_TIM1 | PRR_TIM0 | PRR_TIM2 | PRR_TWI | PRR_PCI | PRR_EFL | PRR_WDT;
#ifdef DEBUG
    DrvPWR_ModuleEnable(PRR_USART0);
#endif

}

#endif

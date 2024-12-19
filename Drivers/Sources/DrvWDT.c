#include "DrvWDT.h"
#include <avr/sfr_defs.h>
//#include "DrvSYS.h"
#include "lgt8f328p_spec.h"

uint8_t DrvWDT_Timeout;

void DrvWDT_Init(uint8_t val, uint8_t reset)
{
    DrvWDT_Timeout=0; 
    DrvWDT_Reset();
    if(reset)
        WDTCSR = _BV(WDIE) | val;
    else
        WDTCSR = _BV(WDE) | val;
}

void DrvWDT_Deinit()
{
    DrvWDT_Reset();
    WDTCSR = _BV(WDE) | _BV(WDCE);
    WDTCSR = 0;
}

ISR(WDT_vect)
{
    DrvWDT_Timeout=1; 

    return;
}
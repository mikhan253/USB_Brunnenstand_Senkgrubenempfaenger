/**********************************************************************************
 ** SPI Treiber                                                                  **
 **********************************************************************************/

/**********************************************************************************
 ** Verwendete Header                                                            **
 **********************************************************************************/
#include <avr/io.h>
#include "lgt8f328p_spec.h"
#include "DrvSPI.h"
#include "DrvSYS.h"
#include "DrvWDT.h"
/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/

void DrvSPI_Init(void)
{
    DrvPWR_ModuleEnable(PRR_SPI);
    // SS = 1
    PORTB = (PORTB & 0xc3) | 0x4;
    // SCK,MISO,MOSI,SS = O,I,O,O
    DDRB = (DDRB & 0xc3) | 0x2c;

    SPCR = (0 << SPIE) | (0 << DORD) | (0 << CPHA) |
           (1 << MSTR) | ((0 & 0x3) << SPR0);

    SPSR = (0 >> 2) & 1;

    // enable spi
    SPCR |= (1 << SPE);
}

void DrvSPI_Deinit(void)
{
    SPCR = 0;
    SPCR |= 0;
    DrvPWR_ModuleDisable(PRR_SPI);
    // SS = 1
    PORTB = (PORTB & 0xc3) | 0x4;
    // SCK,MISO,MOSI,SS = O,I,O,O
    DDRB = (DDRB & 0xc3) | 0x2c;
}

uint8_t DrvSPI_transferByte(uint8_t data)
{
    uint8_t rxdd;

    SPDR = data;

    DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
    while ((SPFR & (1 << RDEMPT)));
    DrvWDT_Deinit();

    rxdd = SPDR;
    SPFR = _BV(RDEMPT) | _BV(WREMPT);

    return rxdd;
}

uint16_t DrvSPI_transferWord(uint16_t data)
{
    union
    {
        uint16_t val;
        struct
        {
            uint8_t lsb;
            uint8_t msb;
        };
    } din, dout;

    din.val = data;
    if (!(SPCR & (1 << DORD)))
    {
        SPDR = din.msb;

        DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
        while ((SPFR & (1 << RDEMPT)));
        DrvWDT_Deinit();

        dout.msb = SPDR;
        SPFR = _BV(RDEMPT) | _BV(WREMPT);

        SPDR = din.lsb;

        DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
        while ((SPFR & (1 << RDEMPT)));
        DrvWDT_Deinit();

        dout.lsb = SPDR;
        SPFR = _BV(RDEMPT) | _BV(WREMPT);
    }
    else
    {
        SPDR = din.lsb;
        DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
        while ((SPFR & (1 << RDEMPT)));
        DrvWDT_Deinit();
            ;
        dout.lsb = SPDR;
        SPFR = _BV(RDEMPT) | _BV(WREMPT);

        SPDR = din.msb;

        DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
        while ((SPFR & (1 << RDEMPT)));
        DrvWDT_Deinit();

        dout.msb = SPDR;
        SPFR = _BV(RDEMPT) | _BV(WREMPT);
    }

    return dout.val;
}

/**
 * @fn void DrvSPI_Exchange(uint8_t *src, uint8_t length)
 */
void DrvSPI_transferBuffer(uint8_t *src, uint8_t length)
{
    if (length == 0)
        return;

    uint8_t *p = (uint8_t *)src;
    SPDR = *p;
    while (--length > 0)
    {
        uint8_t out = *(p + 1);

        DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
        while ((SPFR & (1 << RDEMPT)));
        DrvWDT_Deinit();

        uint8_t in = SPDR;
        SPFR = _BV(RDEMPT) | _BV(WREMPT);
        SPDR = out;
        *p++ = in;
    }

    DrvWDT_Init(WDT_1MS,1); //Reset bei Fehler
    while ((SPFR & (1 << RDEMPT)));
    DrvWDT_Deinit();

    *p = SPDR;
    SPFR = _BV(RDEMPT) | _BV(WREMPT);
}

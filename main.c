// #include "HlDrvRFM23.h"

#include <stdint.h>
#include <util/delay.h> //nur zum debuggen
#include "DrvSYS.h"
#include "DrvUSART.h"
#include "DrvADC.h"
#include "DrvTWI.h"
#include "HlDrvVL53L0X.h"

#ifdef DEBUG
#include <stdio.h> //nur zum debuggen
#endif
static int uart_putchar(char c, FILE *stream)
{
    DrvUSART_PutChar(c);
    return 0;
}
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void printports()
{
    uint8_t a, b, c;
    a = PORTB;
    b = DDRB;
    c = PINB;
    printf("PORTB=%02X DDR=%02X PIN=%02X\r\n", a, b, c);
    a = PORTC;
    b = DDRC;
    c = PINC;
    printf("PORTC=%02X DDR=%02X PIN=%02X\r\n", a, b, c);
    a = PORTD;
    b = DDRD;
    c = PIND;
    printf("PORTD=%02X DDR=%02X PIN=%02X\r\n", a, b, c);
}
int main(void)
{
    stdout = &mystdout;
    uint8_t i = 10;
    uint16_t adcval;

    DrvSYS_Init();
#ifdef DEBUG
    DrvUSART_Init();
#endif

    printf("RFTransmitterSenkrgube - Wähle was:\r\n");
    while (1)
    {
        HlDrvGPIO_LED_Enable();
        _delay_ms(100);
        HlDrvGPIO_LED_Disable();
        _delay_ms(100);
        //i = DrvUSART_GetChar();
        //printf("Dein Zeichen war: %i\r\n", i);
        i='b';
        switch (i)
        {
        case 'a':
            printf("Batteriespannung ");
            /* Batteriespannung erfassen*/
            HlDrvGPIO_ADCVBATT_Enable();
            DrvADC_Init();
            _delay_us(2);
            adcval = DrvADC_ReadData();
            HlDrvGPIO_ADCVBATT_Disable();
            DrvADC_Deinit();
            _delay_ms(10);
            printf("ADC-Wert: %u\r\n", adcval);
            break;
        case 'b':
            printf("Abstandssensor\r\n");
uint16_t result;
while(1){
            HlDrvGPIO_VL53L0X_Enable();
            i2c_init();
            printf("INIT OK\r\n");
            _delay_ms(2);
            printf("HlDrvInit\r\n");
            initVL53L0X(0);
            //HlDrvVL53L0X_Init();
            setSignalRateLimit(0.1);
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(500 * 1000UL);
            printf("HlDrvSingleMeasure\r\n");
            statInfo_t xTraStats;

                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i ",result);
                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i ",result);
                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i ",result);
                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i ",result);
                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i ",result);
                 result = readRangeSingleMillimeters(&xTraStats);
            printf("val:%i\r\n",result);
            HlDrvGPIO_VL53L0X_Disable();
_delay_ms(100);
}

            //HlDrvVL53L0X_SingleMeasurement();
            /*i = 0xC0;
            DrvTWI_MasterSend(0x52, 1, &i);
            DrvTWI_MasterReceive(0x52, 1, &i);
            _delay_ms(2);
            i = 0xC0;
            DrvTWI_MasterSend(0x52, 1, &i);
            DrvTWI_MasterReceive(0x52, 1, &i);
            printf("Wert: %02X\r\n", i);*/
            //DrvTWI_Deinit();
            HlDrvGPIO_VL53L0X_Disable();
            break;
        case 'c':
            break;
        case 'x':
            printf("POWEROFF\r\n");
            while (1)
                HlDrvGPIO_PowerOff();
            break;
        default:
        }
    }

    // Device initialization
    // init();
    // DrvUSART_SendStr("main: Init successful\n");
    // if(HlDrvRFM23_Enable())
    //	DrvUSART_SendStr("RFM23: Init Failed\n");
    // else
    //	DrvUSART_SendStr("RFM23: Init OK\n");
    // HlDrvVL53L0X_Enable();

    // DDRB |=  _BV(PB5);
    // DrvSYS_SetPin(B,PB5);
    // DrvSYS_ClearPin(B,PB5);

    // Add your code from here
    /*while(1)
    {
        DrvMISC_Delayms(1000);
        i++;
        sprintf((char*)HlDrvRFM23_DataBuffer,"Ha %i",i);
        if (HlDrvRFM23_TransmitData())
            DrvUSART_SendStr("RFM23 - Transmit Failed\n");
        else
            DrvUSART_SendStr("RFM23 - Transmit OK\n");
    }*/
}

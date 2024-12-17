// #include "HlDrvRFM23.h"

#include <stdint.h>
#include <util/delay.h> //nur zum debuggen
#include "DrvSYS.h"
#include "DrvUSART.h"
#include "DrvADC.h"
#include "DrvTWI.h"
#include "DrvSPI.h"
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
            statInfo_t xTraStats;
    stdout = &mystdout;
    uint8_t i = 10;
    uint16_t adcval;
uint16_t result[3];
    uint8_t btmp;
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

        printf("Batteriespannung ");
        /* Batteriespannung erfassen*/
        HlDrvGPIO_ADCVBATT_Enable();
        DrvADC_Init();
        _delay_us(2);
        adcval = DrvADC_ReadData();
        HlDrvGPIO_ADCVBATT_Disable();
        DrvADC_Deinit();
        _delay_ms(10);
        printf("%u\r\n", adcval);

        printf("Abstandssensor ");
        for(i=0;i<3;i++)
            result[i] = 0;
        HlDrvGPIO_VL53L0X_Enable();
        DrvTWI_Init(2);
        _delay_ms(2);
        DrvPWR_SetMCLKDiv(2); //8Mhz
        HlDrvVL53L0X_Init();
        HlDrvVL53L0X_SetSignalRateLimit(0.1);
        HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 18);
        HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
        HlDrvVL53L0X_SetMeasurementTimingBudget(500 * 1000UL);
        for(i=0;i<3;i++)
            result[i] = HlDrvVL53L0X_ReadRangeSingleMillimeters(&xTraStats);
        HlDrvVL53L0X_Deinit();
        DrvTWI_Deinit();
        HlDrvGPIO_VL53L0X_Disable();
        DrvPWR_SetMCLKDiv(0); //32Mhz
        for(i=0;i<3;i++)
            printf("val:%i ",result[i]);
        printf("\r\n");
        _delay_ms(100);
     
     printf("Funkmodul ");
     DrvSPI_Init();
     HlDrvGPIO_RFM23_Enable();
      if(HlDrvRFM23_Enable())
    	printf("RFM23: Init Failed\n");
     else
    	printf("RFM23: Init OK\n");       

    DrvSPI_Deinit();
     HlDrvGPIO_RFM23_Disable();

        _delay_ms(100);
        _delay_ms(100);
        _delay_ms(100);
        printf("RESTART\r\n");

        /*
            printf("POWEROFF\r\n");
            while (1)
                HlDrvGPIO_PowerOff();
            break;
        default:*/
        
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

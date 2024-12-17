// #include "HlDrvRFM23.h"

#include <stdint.h>
#include <util/delay.h> //nur zum debuggen
#include "DrvSYS.h"
#include "DrvUSART.h"
#include "DrvADC.h"
#include "DrvTWI.h"
#include "DrvSPI.h"
#include "HlDrvVL53L0X.h"
#include "HlDrvRFM23.h"

#ifdef DEBUG
#include <stdio.h> //nur zum debuggen
#endif
static int uart_putchar(char c, FILE *stream)
{
    DrvUSART_PutChar(c);
    return 0;
}
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
    statInfo_t xTraStats;
    stdout = &mystdout;
    uint8_t i = 10;
    uint16_t batt_adcval;
    uint16_t tof_range[3];
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
        batt_adcval = DrvADC_ReadData();
        HlDrvGPIO_ADCVBATT_Disable();
        DrvADC_Deinit();
        _delay_ms(10);
        printf("%u\r\n", batt_adcval);

        printf("Abstandssensor ");
        for (i = 0; i < 3; i++)
            tof_range[i] = 0;
        HlDrvGPIO_VL53L0X_Enable();
        DrvTWI_Init(2);
        _delay_ms(2);
        DrvPWR_SetMCLKDiv(2); // 8Mhz
        HlDrvVL53L0X_Init();
        HlDrvVL53L0X_SetSignalRateLimit(0.1);
        HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 18);
        HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
        HlDrvVL53L0X_SetMeasurementTimingBudget(500 * 1000UL);
        for (i = 0; i < 3; i++)
            tof_range[i] = HlDrvVL53L0X_ReadRangeSingleMillimeters(&xTraStats);
        HlDrvVL53L0X_Deinit();
        DrvTWI_Deinit();
        HlDrvGPIO_VL53L0X_Disable();
        DrvPWR_SetMCLKDiv(0); // 32Mhz
        for (i = 0; i < 3; i++)
            printf("val:%i ", tof_range[i]);
        printf("\r\n");
        _delay_ms(100);

        printf("Funkmodul ");
        DrvSPI_Init();
        if (HlDrvRFM23_Enable())
            printf("FAIL ");
        else
            printf("OK ");

        HlDrvRFM23_DataBuffer[0]=batt_adcval >> 8;
        HlDrvRFM23_DataBuffer[1]=batt_adcval;
        HlDrvRFM23_DataBuffer[2]=tof_range[0] >> 8;
        HlDrvRFM23_DataBuffer[3]=tof_range[0];
        HlDrvRFM23_DataBuffer[4]=tof_range[1] >> 8;
        HlDrvRFM23_DataBuffer[5]=tof_range[1];
        HlDrvRFM23_DataBuffer[6]=tof_range[2] >> 8;
        HlDrvRFM23_DataBuffer[7]=tof_range[2];
        for (i = 0; i < 3; i++)
        {
            printf("t");
            sprintf((char*)HlDrvRFM23_DataBuffer,"Ha %i",i);
            HlDrvRFM23_TransmitData();
        }
        printf("\r\n");
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

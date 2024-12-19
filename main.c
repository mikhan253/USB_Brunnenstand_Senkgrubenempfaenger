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
static int uart_putchar(char c, FILE *stream)
{
    DrvUSART_PutChar(c);
    return 0;
}
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif

int main(void)
{
    statInfo_t xTraStats;
    uint8_t i;
    uint16_t batt_adcval;
    uint16_t tof_range[3];
    DrvSYS_Init();

#ifdef DEBUG
    DrvUSART_Init();
    stdout = &mystdout;

    HlDrvGPIO_LED_Enable();
    _delay_ms(100);
    printf("TESTPROGRAMM\r\n");
    HlDrvGPIO_LED_Disable();
    _delay_ms(100);
#endif

    /* Batteriespannung erfassen*/
    HlDrvGPIO_ADCVBATT_Enable(); //2us bis Signal stabil
    DrvADC_Init();
    batt_adcval = DrvADC_ReadData();
    HlDrvGPIO_ADCVBATT_Disable();
    DrvADC_Deinit();

    /* VL53L0X starten */
    HlDrvGPIO_VL53L0X_Enable();
    
    //WATCHDOG 2ms
    //DrvTWI_Init(32);
    //DrvPWR_SetMCLKDiv(0); // 32Mhz
    DrvTWI_Init(2);
    DrvSYS_SetMCLKDiv(2); // 8Mhz
    HlDrvVL53L0X_Init();
    HlDrvVL53L0X_SetSignalRateLimit(0.1);
    HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 18);
    HlDrvVL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
    HlDrvVL53L0X_SetMeasurementTimingBudget(200 * 1000UL);
    for (i = 0; i < 3; i++)
        tof_range[i] = HlDrvVL53L0X_ReadRangeSingleMillimeters(&xTraStats);
    HlDrvVL53L0X_Deinit();
    DrvTWI_Deinit();
    HlDrvGPIO_VL53L0X_Disable();
    DrvSYS_SetMCLKDiv(0); // 32Mhz
    /* RFM23 starten */
    DrvSPI_Init();
    HlDrvRFM23_Enable();
    HlDrvRFM23_PrepareTransmit();
    _delay_us(500);
    for (i = 0; i < 3; i++)
    {
        HlDrvRFM23_DataBuffer[0]=batt_adcval >> 8;
        HlDrvRFM23_DataBuffer[1]=batt_adcval;
        HlDrvRFM23_DataBuffer[2]=tof_range[0] >> 8;
        HlDrvRFM23_DataBuffer[3]=tof_range[0];
        HlDrvRFM23_DataBuffer[4]=tof_range[1] >> 8;
        HlDrvRFM23_DataBuffer[5]=tof_range[1];
        HlDrvRFM23_DataBuffer[6]=tof_range[2] >> 8;
        HlDrvRFM23_DataBuffer[7]=tof_range[2];
        HlDrvRFM23_TransmitData();

    }
    DrvSPI_Deinit();
    HlDrvGPIO_RFM23_Disable();

    printf("Batteriespannung %u\r\nAbstand ", batt_adcval);
    for (i = 0; i < 3; i++)
        printf("%u ", tof_range[i]);
    printf("\r\n");

#ifdef DEBUG
    main();
#endif
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
    HlDrvGPIO_PowerOff();
}

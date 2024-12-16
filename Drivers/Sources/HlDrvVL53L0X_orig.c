/**********************************************************************************
 ** Highlevel Treiber für VL53L0X                                                **
 **********************************************************************************/
#define DEBUG 1
/**********************************************************************************
 ** Verwendete Header                                                            **
 **********************************************************************************/
#include "DrvTWI.h"
#include <avr/pgmspace.h>
#include "HlDrvVL53L0X.h"

#if DEBUG
#include <stdio.h>
#endif
/**********************************************************************************
 ** Macros und Definitionen                                                      **
 **********************************************************************************/
#define HLDRVVL53L0X_RANGE_DATA_INIT 0,7
#define HLDRVVL53L0X_RANGE_DATA_INIT2 8,13
#define HLDRVVL53L0X_RANGE_DEFAULT_TUNING_SETTINGS 14,173
#define HLDRVVL53L0X_RANGE_CONFIGURE_INTERRUPT 174,181
#define HLDRVVL53L0X_RANGE_CONFIGURE_CALIB_VHV 182,185
#define HLDRVVL53L0X_RANGE_CONFIGURE_CALIB_PHASE 186,189
#define HLDRVVL53L0X_RANGE_CONFIGURE_CLEAR_INT 190,193
#define HLDRVVL53L0X_RANGE_MEASURE1 194,199
#define HLDRVVL53L0X_RANGE_MEASURE2 200,207

const uint8_t HlDrvVL53L0X_InitData[] PROGMEM = {
  /*Addr  Data */
    0x88, 0x00, 0x80, 0x01, 0xFF, 0x01, 0x00, 0x00, // data_init
    0x00, 0x01, 0xFF, 0x00, 0x80, 0x00, // data_init2
    0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00, // default_tuning_settings
    0x10, 0x00, 0x11, 0x00, 0x24, 0x01, 0x25, 0xFF, // *
    0x75, 0x00, 0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00, // *
    0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00, // *
    0x31, 0x04, 0x32, 0x03, 0x40, 0x83, 0x46, 0x25, // *
    0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00, // *
    0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00, // *
    0x62, 0x00, 0x64, 0x00, 0x65, 0x00, 0x66, 0xA0, // *
    0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF, // *
    0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00, // *
    0x78, 0x21, 0xFF, 0x01, 0x23, 0x34, 0x42, 0x00, // *
    0x44, 0xFF, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40, // *
    0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00, // *
    0x34, 0x03, 0x35, 0x44, 0xFF, 0x01, 0x31, 0x04, // *
    0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00, // *
    0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28, // *
    0x67, 0x00, 0x70, 0x04, 0x71, 0x01, 0x72, 0xFE, // *
    0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01, // *
    0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01, // *
    0x8E, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00, // *
    0x0A, 0x04, 0x84, 0x01, 0x0B, 0x01, 0x01, 0xE8, // configure interrupt + set_sequence_steps(RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_PRE_RANGE + RANGE_SEQUENCE_STEP_FINAL_RANGE)
    0x01, 0x01, 0x00, 0x41, //single_ref_calibration vhv
    0x01, 0x02, 0x00, 0x01, //single_ref_calibration phase
    0x0B, 0x01, 0x00, 0x00, //single_ref_calib after INT
    0x80, 0x01, 0xFF, 0x01, 0x00, 0x00, //single_measurement1
    0x00, 0x01, 0xFF, 0x00, 0x80, 0x00, 0x00, 0x01, //singel_measurement2
    };

/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/

/* im idle-mode soll er im ready mode sein */

uint8_t buffer[3];
uint8_t stop_variable;

#include <stdio.h>
void HlDrvVL53L0X_Init()
{
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_DATA_INIT);
    buffer[0]=0x91;
    _HlDrvVL53L0X_readbyte();
    stop_variable=buffer[1];
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_DATA_INIT2);
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_DEFAULT_TUNING_SETTINGS);
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_INTERRUPT);
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_CALIB_VHV);
    _HlDrvVL53L0X_waitforinterrupt();
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_CLEAR_INT);
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_CALIB_PHASE);
    _HlDrvVL53L0X_waitforinterrupt();
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_CLEAR_INT);
}

void _HlDrvVL53L0X_waitforinterrupt() {
    //wait for int
    loop_until_bit_is_clear(PIND,4);
    buffer[0]=0x13;
    _HlDrvVL53L0X_readbyte();
    if(!(buffer[1] & 0x07))
        printf("ERROR!!\r\n");
}


uint16_t HlDrvVL53L0X_SingleMeasurement() {
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_MEASURE1);
    printf("stop_variable=%i\r\n",stop_variable);
    buffer[0]=0x91; buffer[1]=stop_variable;
    _HlDrvVL53L0X_writebyte();
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_MEASURE2);
    buffer[0]=0x00;
    do {
        _HlDrvVL53L0X_readbyte();
    } while (buffer[1] & 0x01);
    _HlDrvVL53L0X_waitforinterrupt();

    buffer[0]=0x1E;
    DrvTWI_MasterSend(0x52, 1, &buffer[0]);
    DrvTWI_MasterReceive(0x52, 2, &buffer[1]);
    _delay_ms(2);    
    _HlDrvVL53L0X_bulkwrite(HLDRVVL53L0X_RANGE_CONFIGURE_CLEAR_INT);
    printf("Answer = %02X %02X\r\n",buffer[1],buffer[2]);


}


void _HlDrvVL53L0X_readbyte() {
    DrvTWI_MasterSend(0x52, 1, &buffer[0]);
    DrvTWI_MasterReceive(0x52, 1, &buffer[1]);
    _delay_ms(2);
}

void _HlDrvVL53L0X_writebyte() {
    DrvTWI_MasterSend(0x52, 2, buffer);
    _delay_ms(2);
}
void _HlDrvVL53L0X_bulkwrite(uint8_t start, uint8_t stop) {
    while(start < stop) {
        buffer[0] = pgm_read_byte(&HlDrvVL53L0X_InitData[start++]);
        buffer[1] = pgm_read_byte(&HlDrvVL53L0X_InitData[start++]);
        _HlDrvVL53L0X_writebyte();
    }
}
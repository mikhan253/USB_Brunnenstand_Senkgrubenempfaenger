/**********************************************************************************
 ** Initialisierung System und GPIO                                              **
 **********************************************************************************/
#ifndef _HlDrvVL53L0X_H_
#define _HlDrvVL53L0X_H_

/**********************************************************************************
 ** Verwendete Header                                                            **
 **********************************************************************************/
#include <util/delay.h>

/**********************************************************************************
 ** Macros und Definitionen                                                      **
 **********************************************************************************/

/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/

/**
 * @fn void HlDrvVL53L0X_Init()
 * @brief Initialisierung von Abstandsmesser VL53L0X
 */
void HlDrvVL53L0X_Init();
uint16_t HlDrvVL53L0X_SingleMeasurement();

void _HlDrvVL53L0X_waitforinterrupt();
void _HlDrvVL53L0X_readbyte();
void _HlDrvVL53L0X_writebyte();
void _HlDrvVL53L0X_bulkwrite(uint8_t start, uint8_t stop);
#endif

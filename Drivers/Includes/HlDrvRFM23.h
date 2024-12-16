/**********************************************************************************
 ** Initialisierung System und GPIO                                              **
 **********************************************************************************/
#ifndef _HlDrvRFM23_H_
#define _HlDrvRFM23_H_

/**********************************************************************************
 ** Verwendete Header                                                            **
 **********************************************************************************/

/**********************************************************************************
 ** Macros und Definitionen                                                      **
 **********************************************************************************/
#define RFM23_PACKETLENGTH 8
extern uint8_t HlDrvRFM23_DataBuffer[RFM23_PACKETLENGTH];
/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/

/**
 * @fn void DrvSYS_Init(void)
 * @brief Initialisierung von Funkmodul RFM23
 */
uint8_t HlDrvRFM23_Enable(void);
uint8_t HlDrvRFM23_ReceiveData(void);
uint8_t HlDrvRFM23_TransmitData(void);

void HlDrvRFM23_Write(uint8_t addr, uint8_t val);
void HlDrvRFM23_MultipleTransaction(uint8_t write, uint8_t size, uint8_t addr, uint8_t *val);
uint8_t HlDrvRFM23_Read(uint8_t addr);

uint8_t rfm23_temp();
#endif

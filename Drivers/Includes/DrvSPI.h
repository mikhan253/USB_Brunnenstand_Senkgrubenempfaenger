/**********************************************************************************
 ** SPI Treiber                                                                  **
 **********************************************************************************/
#ifndef _SPI_H_
#define _SPI_H_

/**********************************************************************************
 ** Macros und Definitionen                                                      **
 **********************************************************************************/
#define	DrvSPI_SSON()	PORTB &= 0xfb
#define	DrvSPI_SSOFF()	PORTB |= 0x4

/**********************************************************************************
 ** Funktionen                                                                   **
 **********************************************************************************/

/**
 * @fn void DrvSPI_Init(void)
 * @brief Initialisierung von SPI Interface
 */
void DrvSPI_Init(void);
void DrvSPI_Deinit(void);

void DrvSPI_transferBuffer(uint8_t *, uint8_t);
uint8_t DrvSPI_transferByte(uint8_t);
uint16_t DrvSPI_transferWord(uint16_t);

#ifndef DrvSPI_Transceive
#define DrvSPI_Transceive(len, str) DrvSPI_transferBuffer(str, len)
#endif

#endif

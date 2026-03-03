/*
 * nfc.h
 *
 *  Created on: Mar 3, 2026
 *      Author: HP
 */

#ifndef SRC_NFC_H_
#define SRC_NFC_H_

#include "..\..\Drivers\BSP\B-L4S5I-IOT01\stm32l4s5i_iot01_nfctag.h"
#include <stdint.h>

void NFC_Init(void);
void NFC_WriteURL(const char *url);
void NFC_WriteFallURL(uint32_t fall_number);

#endif /* SRC_NFC_H_ */

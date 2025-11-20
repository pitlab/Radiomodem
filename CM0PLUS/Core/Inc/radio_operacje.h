//////////////////////////////////////////////////////////////////////////////////
//  Moduł operacji podstawowych na radiu
//
//  Created on: 19 lis 2025
//  Author: Piotr
//////////////////////////////////////////////////////////////////////////////////


#ifndef RADIO_OPERACJE_H_
#define RADIO_OPERACJE_H_
#include "radio_funkcje.h"


#define SYNC_WORD_LEN		4	//w bajtach
#define PAYLOAD_GFSK_LEN	18

uint8_t SkanujPasmo(void);
uint8_t WlaczObiorGFSK(uint32_t nTimeout);
uint8_t WyslijRamkeGFSK(void);
uint8_t WyslijRamkeLoRa(void);
uint8_t WlaczObiorLoRa(uint32_t nTimeout);
uint8_t NadawajNosna(uint32_t nCzestotliwosc, uint32_t czas_ms);
uint8_t NadawajPrembule(uint32_t nCzestotliwosc, uint32_t czas_ms);


#endif /* RADIO_OPERACJE_H_ */

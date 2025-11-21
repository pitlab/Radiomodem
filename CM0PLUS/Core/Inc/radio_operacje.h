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
#define PREAMBULA_LEN_GFSK	64
#define PREAMBULA_LEN_LORA	64
#define PAYLOAD_LEN_GFSK	250
#define PAYLOAD_LEN_LORA	25
#define CRC_EN_LORA			1

#define TIMEOUT_NADAJNIKA	500000	//[us]
#define TIMEOUT_ODBIORNIKA	500000	//[us]
//#define TIMEOUT_ODB_LORA	0x03D1	//czas liczony w symbolach a nie mikrosekundach [ok. 1s]
#define TIMEOUT_ODB_LORA	0xFFF1	//czas liczony w symbolach a nie mikrosekundach


uint8_t SkanujPasmo(void);
uint8_t WlaczObiorGFSK(uint32_t nTimeout);
uint8_t WyslijRamkeGFSK(void);
uint8_t WyslijRamkeLoRa(void);
uint8_t WlaczObiorLoRa(uint32_t nTimeout);
uint8_t NadawajNosna(uint32_t nCzestotliwosc, uint32_t czas_ms);
uint8_t NadawajPrembule(uint32_t nCzestotliwosc, uint32_t czas_ms);


#endif /* RADIO_OPERACJE_H_ */

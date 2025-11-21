/*
 * czas.h
 *
 *  Created on: Feb 14, 2025
 *      Author: PitLab
 */

#ifndef INC_CZAS_H_
#define INC_CZAS_H_


#include "stm32wlxx_hal.h"

uint32_t PobierzCzas(void);
uint32_t MinalCzas(uint32_t nPoczatek);
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec);
uint8_t CzekajNaZero(uint8_t chZajety, uint32_t nCzasOczekiwania);

void StartPomiaruCykli(void);
uint32_t WynikPomiaruCykli(uint8_t *CPI, uint8_t *EXC, uint8_t *SLEEP, uint8_t *LSU);
void WyswietlCykle(void);
#endif /* INC_CZAS_H_ */

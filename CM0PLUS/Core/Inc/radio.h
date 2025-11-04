/*
 * radio.h
 *
 *  Created on //Oct 29, 2025
 *      Author //Piotr
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_subghz.h"


#define ROZMIAR_POLECENIA	4
#define ROZMIAR_BUF_RSSI	81

//typy pakietów
#define PAKIET_FSK		0
#define PAKIET_LORA		1
#define PAKIET_BPSK		2
#define PAKIET_MSK		3

//tryby fallback
#define FALLBACK_STDBY		0x20
#define FALLBACK_STDBY_HSE	0x30
#define FALLBACK_FS			0x40

//szerokość pasma
#define BW_FSK4 	0x1F	//4.8 kHz DSB
#define BW_FSK5 	0x17	//5.8 kHz DSB
#define BW_FSK7 	0x0F	//7.3 kHz DSB
#define	BW_FSK9 	0x1E	//9.7 kHz DSB
#define BW_FSK11 	0x16	//11.7 kHz DSB
#define BW_FSK14 	0x0E	//14.6 kHz DSB
#define BW_FSK19 	0x1D	//19.5 kHz DSB
#define BW_FSK23 	0x15	//23.4 kHz DSB
#define BW_FSK29 	0x0D	//29.3 kHz DSB
#define BW_FSK39 	0x1C	//39.0 kHz DSB
#define BW_FSK46 	0x14	//46.9 kHz DSB
#define BW_FSK58 	0x0C	//58.6 kHz DSB
#define BW_FSK78 	0x1B	//78.2 kHz DSB
#define BW_FSK93 	0x13	//93.8 kHz DSB
#define BW_FSK117 	0x0B	//117.3 kHz DSB
#define BW_FSK156 	0x1A	//156.2 kHz DSB
#define BW_FSK187 	0x12	//187.2 kHz DSB
#define BW_FSK234 	0x0A	//234.3 kHz DSB
#define BW_FSK312 	0x19	//312.0 kHz DSB
#define BW_FSK373	0x11 	//373.6 kHz DSB
#define BW_FSK467 	0x09	//467.0 kHz DSB


#define BW_LORA7	0x00	//bandwidth 7 (7.81 kHz)
#define BW_LORA10	0x08 	//bandwidth 10 (10.42 kHz)
#define BW_LORA15	0x01 	//bandwidth 15 (15.63 kHz)
#define BW_LORA20	0x09 	//bandwidth 20 (20.83 kHz)
#define BW_LORA31	0x02 	//bandwidth 31 (31.25 kHz)
#define BW_LORA41	0x0A 	//bandwidth 41 (41.67 kHz)
#define BW_LORA52	0x03 	//bandwidth 62 (62.50 kHz)
#define BW_LORA125	0x04 	//bandwidth 125 (125 kHz)
#define BW_LORA250	0x05 	//bandwidth 250 (250 kHz)
#define BW_LORA500	0x06 	//bandwidth 500 (500 kHz)


//definicje trybów pracy
#define TP_SLEEP		0x1
#define TP_STANDBY_RC	0x2	//Standby mode with RC 13 MHz
#define TP_STANDBY_HSE	0x3	//Standby mode with HSE32
#define TP_FS			0x4	//FS mode
#define TP_RX			0x5	//RX mode
#define TP_TX			0x6 //TX mode
#define MASKA_TRYBU		0x70

union u32_8_t
{
	uint32_t nDane32;
	uint16_t sDane16[2];
	uint8_t chDane8[4];
};


uint8_t UstawTrybOdbioru(uint32_t nTimeout);
uint8_t UstawTrybNadawania(uint32_t nTimeout);
uint8_t UstawTrybSyntezy(void);
uint8_t UstawCzestotliwoscPLL(uint32_t nCzestotliwosc);
uint8_t KalibrujZakresCzestotliwosci(uint16_t sCzestotliwoscDolna, uint16_t sCzestotliwoscGorna);
uint8_t ZmierzRSSI(uint8_t* chStatus, int8_t* chRSSI);
uint8_t UstawTypPakietu(uint8_t chTypPakietu);
uint8_t PobierzTypPakietu(uint8_t *chStatus, uint8_t *chTypPakietu);
uint8_t UstawMocNadajnika(uint8_t chMoc, uint8_t chCzasNarastania);
uint8_t UstawParametryNadajnika(uint8_t chWypelnienieCyklu, uint8_t HpMax, uint8_t chZakresMocy);
uint8_t UstawTrybFallbaclk(uint8_t chTryb);
uint8_t UstawAdresyBuforow(uint8_t chBufNad, uint8_t chBudOdb);
uint8_t UstawParametryModulacjiFSK(uint8_t chSpeadingFactor, uint8_t chKsztaltImpulsu, uint8_t chSzerokoscPasma, uint32_t nOdchylCzestotliwosci);
uint8_t PobierzStatusPakietu(uint8_t *chStatus, uint8_t *chStatusOdbioru, int8_t *chRSSISync, int8_t *chSrednRSSI);
uint8_t PobierzBlad(uint8_t *chStatus, uint8_t *chBlad);
uint8_t UstawSleep(uint8_t chKonfig);
uint8_t UstawStandby(uint8_t chKonfig);
uint8_t PobierzStatus(uint8_t *chStatus);
uint8_t UstawParametryPakietow(uint8_t chTryb);
uint8_t UstawPrzerwnia(void);

uint8_t SkanujPasmo(void);
uint8_t WlaczObior(void);


#endif /* INC_RADIO_H_ */

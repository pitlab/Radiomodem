//////////////////////////////////////////////////////////////////////////////////
//  Moduł niskopoziomowej obsługi radia
//
//  Created on: Oct 29, 2025
//  Author: Piotr
//////////////////////////////////////////////////////////////////////////////////
#include <radio_funkcje.h>
#include "errors.h"
#include "stdio.h"
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_nucleo_radio.h"

extern SUBGHZ_HandleTypeDef hsubghz;
extern UART_HandleTypeDef huart1;
volatile uint8_t chStanProtokolu;
uint8_t chIloscOdebranychDanych;

//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego nadawania
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_TxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_WYSLANO;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_TX_DONE);
}

//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego odbioru
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_RxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	/*uint16_t sStatusPrzerwania = 0;
	uint8_t chStatus, chErr = 0;
	uint8_t chWskaznikDanych;

	chErr |= PobierzStatusPrzerwania(&chStatus, &sStatusPrzerwania);
	chErr |= PobierzStatusBufora(&chIloscOdebranychDanych, &chWskaznikDanych);
	//chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, ADR_BUF_ODB, chBuforOdbiorczy, chIloscOdebranychDanych);	//odczytaj bufor*/
	chStanProtokolu = RP_ODEBR_DANE;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_RX_DONE);
}

void HAL_SUBGHZ_PreambleDetectedCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_PREAMB;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_PREAMB_DET);
}

void HAL_SUBGHZ_SyncWordValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_SYNC;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_SYNC_DET);
}

void HAL_SUBGHZ_HeaderValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_NAGL;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_HEAD_VALID);
}

void HAL_SUBGHZ_HeaderErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_BLAD_NAGL;
	BSP_LED_Toggle(LED_RED);
	//KasujPrzerwnie(IRQ_HEAD_ERROR);
}

void HAL_SUBGHZ_CRCErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_BLAD_CRC;
	BSP_LED_Toggle(LED_RED);
	//KasujPrzerwnie(IRQ_CRC_ERROR);
}

void HAL_SUBGHZ_CADStatusCallback(SUBGHZ_HandleTypeDef *hsubghz, HAL_SUBGHZ_CadStatusTypeDef cadstatus)
{
	chStanProtokolu = RP_CAD;
	BSP_LED_Toggle(LED_GREEN);
	//KasujPrzerwnie(IRQ_CAD_DETECT);
}

void HAL_SUBGHZ_RxTxTimeoutCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_TIMEOUT;
	BSP_LED_Toggle(LED_RED);
	//KasujPrzerwnie(IRQ_TIMEOUT);
}

void HAL_SUBGHZ_LrFhssHopCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_HOP_LR_FHSS;
	BSP_LED_Toggle(LED_GREEN);
}

/*(+) Set and execute a command in blocking mode using @ref HAL_SUBGHZ_ExecSetCmd()
  (+) Get a status blocking mode using @ref HAL_SUBGHZ_ExecGetCmd()
  (+) Write a Data Buffer in blocking mode using @ref HAL_SUBGHZ_WriteBuffer()
  (+) Read a Data Buffer  in blocking mode using @ref HAL_SUBGHZ_ReadBuffer()
  (+) Write Registers (more than 1 byte) in blocking mode using @ref HAL_SUBGHZ_WriteRegisters()
  (+) Read Registers (more than 1 byte) in blocking mode using @ref HAL_SUBGHZ_ReadRegisters()
  (+) Write Register (1 byte) in blocking mode using @ref HAL_SUBGHZ_WriteRegister()
  (+) Read Register (1 byte) in blocking mode using @ref HAL_SUBGHZ_ReadRegister()*/



//////////////////////////////////////////////////////////////////////////////////
// Ustawia rado w tryb odbioru z timeoutem podanym w mirosekundach
// Parametry: sTimeout w mikrosekundach. 0 = wyłączony
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybOdbioru(uint32_t nTimeout)
{
	HAL_StatusTypeDef chErr;
	uint8_t chStatus;
	uint8_t chLicznikTimeoutu = 100;
	uint8_t chBuforDanych[3];

	//nTimeout *= 64; 	//1/15,625 = 0,064 * 1000us/ms = 64
	nTimeout = (uint32_t)(nTimeout * 64) / 1000; 	//1/15,625 = 0,064
	chBuforDanych[0] = (nTimeout >> 16) & 0xFF;
	chBuforDanych[1] = (nTimeout >> 8) & 0xFF;
	chBuforDanych[2] = (nTimeout) & 0xFF;
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, chBuforDanych, 3);

	//sprawdź czy wszedł w tryb odbioru
	do
	{
		chErr = PobierzStatus(&chStatus);
		chLicznikTimeoutu--;
	}
	while ((((chStatus & MASKA_TRYBU) >> 4) != TP_RX) && (chLicznikTimeoutu));

	if (!chLicznikTimeoutu)
		chErr = ERR_TIMEOUT;
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia rado w tryb nadawania z timeoutem podanym w mikrosekundach
// Parametry: sTimeout w mikrosekundach. 0 = wyłączony
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybNadawania(uint32_t nTimeout)
{
	HAL_StatusTypeDef chErr;
	uint8_t chStatus;
	uint8_t chLicznikTimeoutu = 100;
	uint8_t chBuforDanych[3];

	nTimeout = (uint32_t)(nTimeout * 64) / 1000; 	//1/15,625 = 0,064

	chBuforDanych[0] = (nTimeout >> 16) & 0xFF;
	chBuforDanych[1] = (nTimeout >> 8) & 0xFF;
	chBuforDanych[2] = (nTimeout) & 0xFF;
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, chBuforDanych, 3);

	//sprawdź czy wszedł w tryb nadawania
	do
	{
		chErr = PobierzStatus(&chStatus);
		chLicznikTimeoutu--;
		HAL_Delay(1);
	}
	while ((((chStatus & MASKA_TRYBU) >> 4) != TP_TX) && (chLicznikTimeoutu));

	if (!chLicznikTimeoutu)
		chErr = ERR_TIMEOUT;
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia rado w tryb ustawiania częstotliwości w PLL
// Parametry: brak
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybSyntezy(void)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforPolecen;
	uint8_t chStatus;
	uint8_t chLicznikTimeoutu = 100;

	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_FS, &chBuforPolecen, 0);

	//sprawdź czy wszedł w tryb syntezy
	do
	{
		chErr = PobierzStatus(&chStatus);
		chLicznikTimeoutu--;
	}
	while ((((chStatus & MASKA_TRYBU) >> 4) != TP_FS) && (chLicznikTimeoutu));

	if (chLicznikTimeoutu)
		chErr = ERR_TIMEOUT;
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Funkcja ustawia częstotliwość PLL według wzoru: PLL frequency = 32e6 x RFfreq / 2^25
// Stąd RF Freq = PLL frequency * 2^25 / 32e6
// Parametry: fCzestotliwosc [Hz]
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawCzestotliwoscPLL(uint32_t nCzestotliwosc)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4];
	uint32_t nRejCzest;

	nRejCzest = (uint32_t)(((double)nCzestotliwosc * (1UL << 25)) / 32000000.0);
	chBuforDanych[0] = (nRejCzest >> 24) & 0xFF;
	chBuforDanych[1] = (nRejCzest >> 16) & 0xFF;
	chBuforDanych[2] = (nRejCzest >> 8) & 0xFF;
	chBuforDanych[3] = (nRejCzest) & 0xFF;

	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, chBuforDanych, 4);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Wykonuje kalibrację dla podanego zakresu częstotliwości
// Parametry: sCzestotliwoscDolna, sCzestotliwoscGorna - zakres częstotliwości do kalibracji [MHz]
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujZakresCzestotliwosci(uint16_t sCzestotliwoscDolna, uint16_t sCzestotliwoscGorna)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2];

	if (sCzestotliwoscDolna > sCzestotliwoscGorna)
		return ERR_PARAMETRY;

	chBuforDanych[0] = (sCzestotliwoscDolna >> 2) & 0xFF;
	chBuforDanych[1] = (sCzestotliwoscGorna >> 2) & 0xFF;
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATEIMAGE, chBuforDanych, 2);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Mierzy wartość SSI gdy jest w trybie odbioru
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chRSSI - wskaźnik na wartość ze znakiem zmierzonego RSSI [dB]
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzRSSI(uint8_t* chStatus, int8_t* chRSSI)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2] = {0};

	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RSSIINST, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		*chStatus = chBuforDanych[1];
		*chRSSI = chBuforDanych[0] / 2 * (-1);
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Funkcja ustawia radio do obsługu danego typu pakietów danych
// Parametry: chTypPakietu - patrz definicje stałych: PAKIET_FSK..PAKIET_MSK
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTypPakietu(uint8_t chTypPakietu)
{
	uint8_t chBuforDanych = chTypPakietu & 0x03;

	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, &chBuforDanych, 1);
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera typ pakietów danych
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chTypPakietu - wskaźnik na typ pakietu. Patrz definicje stałych: PAKIET_FSK..PAKIET_MSK
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzTypPakietu(uint8_t *chStatus, uint8_t *chTypPakietu)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2] = {0};

	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETTYPE, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		*chStatus = chBuforDanych[1];
		*chTypPakietu = chBuforDanych[0];
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Funkcja ustawia moc i czas narastania sygnału.
// Parametry:
//	chMoc - Moc jest w zakresie -17..+14dBm dla LP lub -9..+22dBm dla HP
//	chCzasNarastania - 0=10us..7=3400us
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawMocNadajnika(uint8_t chMoc, uint8_t chCzasNarastania)
{
	uint8_t chBuforDanych[2];

	chBuforDanych[0] = chMoc;
	chBuforDanych[1] = chCzasNarastania;
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, chBuforDanych, 2);
}



//////////////////////////////////////////////////////////////////////////////////
// Funkcja ustawia parametry nadajnika
// Parametry:
//	chWypelnienieCyklu - PA duty cycle (conduit angle) control
//	HpMax - HP PA output power (see Table 35 for settings)
//	chZakresMocy - jeden z dwu zakresów 1=LP lub 0=HP
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryNadajnika(uint8_t chWypelnienieCyklu, uint8_t HpMax, uint8_t chZakresMocy)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4];

	chBuforDanych[0] = chWypelnienieCyklu;
	chBuforDanych[1] = HpMax;
	chBuforDanych[2] = chZakresMocy;
	chBuforDanych[3] = 0x01;	//stała zarezerwowana
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, chBuforDanych, 4);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Definiuje tryb w jaki przechodzi radio po zakończonej operacji nadawania lub odbioru
// Parametry:
//	chTryb - 00 Standby; 0x30 Standby z właczonym HSE32; 0x40 tryb FS
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybFallbaclk(uint8_t chTryb)
{
	uint8_t chBuforDanych = chTryb;

	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXFALLBACKMODE, &chBuforDanych, 1);
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia adresy buforów nadawczego i odbiorczego
// Parametry:
//	chBufNad - offset adresu bufora nadajnika względem adresu bazowego sub-GHz RAM
//	chBudOdb - offset adresu bufora odbiornika względem adresu bazowego sub-GHz RAM
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawAdresyBuforow(uint8_t chBufNad, uint8_t chBudOdb)
{
	uint8_t chBuforDanych[2];

	chBuforDanych[0] = chBufNad;
	chBuforDanych[1] = chBudOdb;
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_BUFFERBASEADDRESS, chBuforDanych, 2);
}



//////////////////////////////////////////////////////////////////////////////////
// pobiera status bufora odbiorczego - funkcja w rzeczywistości działa inaczej niż opisana w dokumentacji. Zwraca 2 bajty zamiast 3
// Parametry:
//	*chIloscOdebrana - wskaźnik na liczbę odebranych danych
//	*chWskDane - wskaźnik na dane w buforze odbiorczym
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStatusBufora(uint8_t *chIloscOdebrana, uint8_t *chWskDane)
{
	uint8_t chErr, chBuforDanych[2] = {0, 0};

	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RXBUFFERSTATUS, chBuforDanych, 2);	//w przykładzie radio_driver.c są pobierane 2 bajty
	if (chErr == ERR_OK)
	{
		*chIloscOdebrana = chBuforDanych[0];
		*chWskDane = chBuforDanych[1];
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// pobiera status przerwania
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*sStatusIRQ - wskaźnik na status przerwań
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStatusPrzerwania(uint8_t *chStatus, uint16_t *sStatusIRQ)
{
	uint8_t chErr, chBuforDanych[4] = {0, 0, 0, 0};

	//chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_IRQSTATUS, chBuforDanych, 3);
	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_IRQSTATUS, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		//*chStatus = chBuforDanych[0];
		//*sStatusIRQ = chBuforDanych[2] + (uint16_t)chBuforDanych[1] * 0x100;
		*sStatusIRQ = chBuforDanych[1] + (uint16_t)chBuforDanych[2] * 0x100;
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// ustawia parametry modulacji (G)FSK. Musi być poprzedzone funkcją: UstawTypPakietu()
// Parametry:
//	nPredkoscBit - określa prędkość transmisji
//	chKsztaltImpulsu - wartość filtra kstałtujacego impulsy: 0=bez filtra, 8=Gaussian BT 0.3; 9=Gaussian BT 0.5; 10=Gaussian BT 0.7; 11=Gaussian BT 1.0;
//	chSzerokoscPasma - patrz definicje stałych: BW4..BW467
//	nOdchylCzestotliwosci - Frequency deviation x 225 / 32 MHz
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryModulacjiFSK(uint32_t nPredkoscBit, uint8_t chKsztaltImpulsu, uint8_t chSzerokoscPasma, uint32_t nOdchylCzestotliwosci)
{
	uint8_t chBuforDanych[8];
	uint32_t nBitrate = (uint32_t)(((uint64_t)32 * 32000000) / nPredkoscBit);
	uint32_t nFdewiac = (uint32_t)(((uint64_t)nOdchylCzestotliwosci * (1UL << 25)) / 32000000.0);

	chBuforDanych[0] = (nBitrate >> 16) & 0xFF;
	chBuforDanych[1] = (nBitrate >> 8) & 0xFF;
	chBuforDanych[2] = (nBitrate) & 0xFF;
	chBuforDanych[3] = chKsztaltImpulsu & 0x0F;
	chBuforDanych[4] = chSzerokoscPasma;
	chBuforDanych[5] = (nFdewiac >> 16) & 0xFF;
	chBuforDanych[6] = (nFdewiac >> 8) & 0xFF;
	chBuforDanych[7] = (nFdewiac) & 0xFF;
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 8);
}



//////////////////////////////////////////////////////////////////////////////////
// ustawia parametry modulacji LoRa. Musi być poprzedzone funkcją: UstawTypPakietu()
// Parametry:
//	chSpredingFactor = 0x05..0x0C - rozproszenie widma
//	chSzerokoscPasma - patrz definicje stałych: BW_LORA7..BW_LORA500
//	chKorekcjaBledow -
//	chOptymalizacja
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryModulacjiLoRa(uint8_t chSpredingFactor, uint8_t chSzerokoscPasma, uint8_t chKorekcjaBledow, uint8_t chOptymalizacja)
{
	uint8_t chBuforDanych[4];

	chBuforDanych[0] = chSpredingFactor & 0x0F;
	chBuforDanych[1] = chSzerokoscPasma & 0x0F;
	chBuforDanych[2] = chKorekcjaBledow & 0x07;
	chBuforDanych[3] = chOptymalizacja & 0x01;
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 4);
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera informacje o błędzie
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chBlad - wskaźnik na błąd
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzBlad(uint8_t *chStatus, uint8_t *chBlad)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2] = {0};

	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		*chStatus = chBuforDanych[1];
		*chBlad = chBuforDanych[0];
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera informacje o statusie pakietów danych
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chStatusOdbioru
//	*chRSSISync - wskaźnik na RSSI podczas synchronizacji
//	*chSrednRSSI - wskaxnik na srednie RSSI poczas odbioru pakietu
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStatusPakietu(uint8_t *chStatus, uint8_t *chStatusOdbioru, int8_t *chRSSISync, int8_t *chSrednRSSI)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4] = {0};

	chErr = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, chBuforDanych, 4);
	if (chErr == ERR_OK)
	{
		*chStatus = (int8_t)chBuforDanych[0];
		*chStatusOdbioru = (int8_t)chBuforDanych[1];
		*chRSSISync = (int8_t)(chBuforDanych[2] / 2) * (-1);
		*chSrednRSSI = (chBuforDanych[3] / 2) * (-1);
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera informacje o statusie pakietów danych
// Parametry:
//	*chStatus - wskaźnik na status radia
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStatus(uint8_t *chStatus)
{
	*chStatus = 0;
	return HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, chStatus, 1);
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera informacje o statusie pakietów danych
// Parametry:
//	chKonfig - b0=0 Sub-GHz radio RTC wake-up disabled, b2=0 cold startup when exiting Sleep mode, configuration registers reset
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawSleep(uint8_t chKonfig)
{
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &chKonfig, 1);
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia tryb pracy
// Parametry:
//	chKonfig - 0: RC 13 MHz used in Standby mode, 1: HSE32 used in Standby mode (Standby with HSE32)
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawStandby(uint8_t chKonfig)
{
	HAL_StatusTypeDef chErr;
	uint8_t chStatus;
	uint8_t chLicznikTimeoutu = 100;

	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &chKonfig, 1);

	//sprawdź czy radio weszło w standby
	do
	{
		chErr = PobierzStatus(&chStatus);
		chLicznikTimeoutu--;
	}
	while ((((chStatus & MASKA_TRYBU) >> 4) != TP_STANDBY_RC) && (chLicznikTimeoutu));	//czy jest w trybie: 0x2: Standby mode with RC 13 MHz

	if (chLicznikTimeoutu)
		chErr = ERR_TIMEOUT;
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia Cannel Activity Detection
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawCAD(void)
{
	uint8_t chKonfig = 0;
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_CAD, &chKonfig, 0);
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia parametry pakietów dla GFSK
// Parametry:
//	sDlugPreamb - liczba bitów preambuły
//	chSyncWordlLen - liczba bajtów słowa synchronizująceg [0..8]
//	chPayloadLen - liczba bajtów wysyłanych danych [0.255]
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryPakietowGFSK(uint16_t sDlugPreamb, uint8_t chSyncWordlLen, uint8_t chPayloadLen)
{
	HAL_StatusTypeDef chErr;
	uint8_t chKonfig[8] = {0};

	//ustaw ziarno CRC
	chKonfig[0] = ZIARNO_CRC_CCITT >> 8;
	chKonfig[1] = ZIARNO_CRC_CCITT & 0xFF;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GCRCINIRH, chKonfig, 2);

	//ustaw wielomian CRC
	chKonfig[0] = WIELOMIAN_CRC_CCITT >> 8;
	chKonfig[1] = WIELOMIAN_CRC_CCITT & 0xFF;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GCRCPOLRH, chKonfig, 2);

	chKonfig[0] = (sDlugPreamb >> 8) & 0xFF;	//bytes 2:1 bits 15:0 PbLength[15:0]: Preamble length in number of symbols
	chKonfig[1] = (sDlugPreamb) & 0xFF;
	chKonfig[2] = 5;	//bits 2:0 PbDetLength[2:0]: Preamble detection length in number of bit symbols: 0x0: preamble detection disabled, 0x4: 8-bit preamble detection, 0x5: 16-bit preamble detection, 0x6: 24-bit preamble detection, 0x7: 32-bit preamble detection
	chKonfig[3] = chSyncWordlLen * 8;	//bits: 6:0 SyncWordLength[6:0]: Synchronization word length in number of bit symbols: 0x00 - 0x40: 0 to 64-bit synchronization word (synchronization word data defined in SUBGHZ_GSYNCR[0:7])
	chKonfig[4] = 0;	//bits: 1:0 AddrComp[1:0]: Address comparison/filtering, 0x0: address comparison/filtering disabled, 0x1: address comparison/filtering on node address, 0x2: address comparison/filtering on node and broadcast addresses
	chKonfig[5] = 1;	//bit 0 PktType: Packet type definition: 0: Fixed payload length and header field not added to packet, 1: Variable payload length and header field added to packet
	chKonfig[6] = chPayloadLen;	//bits 7:0 PayloadLength[7:0]: Payload length in number of bytes 0x00- 0xFF: 0 to 255 bytes
	chKonfig[7] = 2;	//bits 2:0 CrcType[2:0]: CRC type definition The CRC initialization value is provided in SUBGHZ_GCRCINIRL and SUBGHZ_GCRCINIRH. The polynomial is defined in SUBGHZ_GCRCPOLRL and SUBGHZ_GCRCPOLRH.
						//0x0: 1-byte CRC, 0x1: no CRC, 0x2: 2-byte CRC, 0x4: 1-byte inverted CRC, 0x6: 2-byte inverted CRC
	chKonfig[8] = 0;	//bit 0 Whitening: Whitening enable. The whitening initial value is provided in WHITEINI[8:0]: 0: Whitening disabled, 1: Whitening enabled
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, chKonfig, 8);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia parametry pakietów dla LoRa
// Parametry:
//	sDlugPreamb - liczba bitów preambuły
//	chStalyNagl - stała (1) lub mienna (0) długość nagłówka
//	chWlaczCRC - włączenie obecnosci CRC
//	chSyncWordlLength - liczba bitów słowa synchronizująceg
//	chPayloadLenght - liczba bajtów wysyłanych danych
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryPakietowLoRa(uint16_t sDlugPreamb, uint8_t chStalyNagl, uint8_t chPayloadLenght, uint8_t chWlaczCRC, uint8_t chInvertIQ)
{
	HAL_StatusTypeDef chErr;
	uint8_t chKonfig[6] = {0};

	//ustaw ziarno CRC
	chKonfig[0] = ZIARNO_CRC_CCITT >> 8;
	chKonfig[1] = ZIARNO_CRC_CCITT & 0xFF;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GCRCINIRH, chKonfig, 2);

	//ustaw wielomian CRC
	chKonfig[0] = WIELOMIAN_CRC_CCITT >> 8;
	chKonfig[1] = WIELOMIAN_CRC_CCITT & 0xFF;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GCRCPOLRH, chKonfig, 2);

	chKonfig[0] = (sDlugPreamb >> 8) & 0xFF;	//bytes 2:1 bits 15:0 PbLength[15:0]: Preamble length in number of symbols
	chKonfig[1] = (sDlugPreamb) & 0xFF;
	chKonfig[2] = chStalyNagl;	//bit 1:0 PHeaderType: Header type: 0: explicit header for variable length payload, 1: implicit header for fixed length payload
	chKonfig[3] = chPayloadLenght;	//bits 7:0 PayloadLength[7:0]: Payload length in number of bytes 0x00- 0xFF: 0 to 255 bytes
	chKonfig[4] = chWlaczCRC;	//CrcType CRC enable: 0: CRC disabled, 1: CRC enabled
	chKonfig[5] = chInvertIQ;	//bit 0 InvertIQ: IQ setup: 0: standard IQ setup, 1: inverted IQ setup
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, chKonfig, 6);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Ustawia parametry pakietów
// Parametry: na razie nic
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawPrzerwnie(uint16_t sGlobalEnable, uint16_t sIRQ1En, uint16_t sIRQ2En, uint16_t sIRQ3En)
{
	HAL_StatusTypeDef chErr;
	uint8_t chKonfig[8] = {0};

	//bytes 2:1 bits 15:0 IrqMask[15:0]
	//Global interrupt enable See Table 37 for interrupt bit map definition. For each bit: 0: IRQ disabled, 1: IRQ enabled
	chKonfig[0] = sGlobalEnable;
	chKonfig[1] = (sGlobalEnable >> 8);
	chKonfig[2] = sIRQ1En;
	chKonfig[3] = (sIRQ1En >> 8);
	chKonfig[4] = sIRQ2En;
	chKonfig[5] = (sIRQ2En >> 8);
	chKonfig[6] = sIRQ3En;
	chKonfig[7] = (sIRQ3En >> 8);
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CFG_DIOIRQ, chKonfig, 8);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// kasuje flagę aktywnego przerwania
// Parametry: sPrzerwanie - bit przerwania
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t KasujPrzerwnie(uint16_t sPrzerwanie)
{
	uint8_t chKonfig[8] = {0};

	chKonfig[0] = sPrzerwanie;
	chKonfig[1] = (sPrzerwanie >> 8);
	return HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CFG_DIOIRQ, chKonfig, 2);
}




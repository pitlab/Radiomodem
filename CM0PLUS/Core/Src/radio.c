//////////////////////////////////////////////////////////////////////////////////
//  Moduł niskopoziomowej obsługi radia
//
//  Created on: Oct 29, 2025
//  Author: Piotr
//////////////////////////////////////////////////////////////////////////////////
#include "radio.h"
#include "errors.h"
#include "stdio.h"
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_nucleo_radio.h"

extern SUBGHZ_HandleTypeDef hsubghz;
extern UART_HandleTypeDef huart1;
volatile uint8_t chStanProtokolu;
uint8_t chLicznikRamek;
uint8_t chBuforUart[90];
uint8_t chBuforUart2[40];
uint8_t chBuforNadawczy[ROZMIAR_BUFORA_NADAWCZEGO];
uint8_t chBuforOdbiorczy[ROZMIAR_BUFORA_ODBIORCZEGO];

//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego nadawania
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_TxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_WYSLANO;
}



//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego odbioru
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_RxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_DANE;
}

void HAL_SUBGHZ_PreambleDetectedCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_PREAMB;
}

void HAL_SUBGHZ_SyncWordValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_SYNC;
}

void HAL_SUBGHZ_HeaderValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_ODEBR_NAGL;
}

void HAL_SUBGHZ_HeaderErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_BLAD_NAGL;
}

void HAL_SUBGHZ_CRCErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_BLAD_CRC;
}

void HAL_SUBGHZ_CADStatusCallback(SUBGHZ_HandleTypeDef *hsubghz, HAL_SUBGHZ_CadStatusTypeDef cadstatus)
{
	chStanProtokolu = RP_CAD;
}

void HAL_SUBGHZ_RxTxTimeoutCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_TIMEOUT;
}

void HAL_SUBGHZ_LrFhssHopCallback(SUBGHZ_HandleTypeDef *hsubghz)
{
	chStanProtokolu = RP_HOP_LR_FHSS;
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
// Ustawia rado w tryb odbioru z timeoutem podanym w mikrosekundach
// Parametry: sTimeout w mikrosekundach. 0 = wyłączony
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybOdbioru(uint32_t nTimeout)
{
	HAL_StatusTypeDef chErr;
	uint8_t chStatus;
	uint8_t chLicznikTimeoutu = 100;
	uint8_t chBuforDanych[3];

	nTimeout = (uint32_t)(nTimeout * 64) / 1000; 	//1/15,625 = 0,064
	chBuforDanych[0] = (nTimeout >> 16) & 0xFF;
	chBuforDanych[1] = (nTimeout >> 8) & 0xFF;
	chBuforDanych[2] = (nTimeout) & 0xFF;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, chBuforDanych, 3);

	//sprawdź czy wszedł w tryb odbioru
	do
	{
		chErr = PobierzStatus(&chStatus);
		chLicznikTimeoutu--;
	}
	while ((((chStatus & MASKA_TRYBU) >> 4) != TP_RX) && (chLicznikTimeoutu));

	if (chLicznikTimeoutu)
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
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, chBuforDanych, 3);

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

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_FS, &chBuforPolecen, 0);

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

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, chBuforDanych, 4);
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
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATEIMAGE, chBuforDanych, 2);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Mierzy wartość SSI gdy jest w trybie odbioru
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chRSSI - wskaźnik na wartość ze zanakiem zmierzonego RSSI [dB]
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzRSSI(uint8_t* chStatus, int8_t* chRSSI)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2] = {0};


	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RSSIINST, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		*chStatus = chBuforDanych[1];
		*chRSSI = chBuforDanych[0] / 2 * (-1);
		/*chStatus = chBuforDanych[0];
		*chRSSI = (int8_t)chBuforDanych[1] / (-2);*/
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
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych = chTypPakietu;

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, &chBuforDanych, 1);
	return chErr;
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

	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETTYPE, chBuforDanych, 2);
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
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2];

	chBuforDanych[0] = chMoc;
	chBuforDanych[1] = chCzasNarastania;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, chBuforDanych, 2);
	return chErr;
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
	chBuforDanych[2]= chZakresMocy;
	chBuforDanych[3] = 0x01;	//stała zarezerwowana
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, chBuforDanych, 4);
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
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych = chTryb;

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXFALLBACKMODE, &chBuforDanych, 1);
	return chErr;
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
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2];

	chBuforDanych[0] = chBufNad;
	chBuforDanych[1] = chBudOdb;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_BUFFERBASEADDRESS, chBuforDanych, 2);
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
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[7];

	chBuforDanych[0] = (nPredkoscBit >> 16) & 0xFF;
	chBuforDanych[1] = (nPredkoscBit >> 8) & 0xFF;
	chBuforDanych[2] = (nPredkoscBit) & 0xFF;
	chBuforDanych[3] = chKsztaltImpulsu & 0x0F;
	chBuforDanych[4] = chSzerokoscPasma;
	chBuforDanych[5] = (nOdchylCzestotliwosci >> 16) & 0xFF;
	chBuforDanych[6] = (nOdchylCzestotliwosci >> 8) & 0xFF;
	chBuforDanych[7] = (nOdchylCzestotliwosci) & 0xFF;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 8);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// ustawia parametry modulacji LoRa. Musi być poprzedzone funkcją: UstawTypPakietu()
// Parametry:
//	chSpredingFactor = 0x05..0x0C - rozproszenie widma
//	chSzerokoscPasma - patrz definicje stałych: BW_LORA7..BW_LORA500
//	nOdchylCzestotliwosci - Frequency deviation x 225 / 32 MHz
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryModulacjiLoRa(uint8_t chSpredingFactor, uint8_t chSzerokoscPasma, uint8_t chKorekcjaBledow, uint8_t chOptymalizacja)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4];

	chBuforDanych[0] = chSpredingFactor & 0x0F;
	chBuforDanych[1] = chSzerokoscPasma & 0x0F;
	chBuforDanych[2] = chKorekcjaBledow & 0x07;
	chBuforDanych[3] = chOptymalizacja & 0x01;

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 4);
	return chErr;
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

	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, chBuforDanych, 2);
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

	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, chBuforDanych, 4);
	if (chErr == ERR_OK)
	{
		/*chSrednRSSI = (int8_t)chBuforDanych[0] / -2;
		*chRSSISync = (int8_t)chBuforDanych[1] / -2;
		*chStatusOdbioru = chBuforDanych[2];
		*chStatus = chBuforDanych[3];*/

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
	HAL_StatusTypeDef chErr;

	*chStatus = 0;
	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, chStatus, 1);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Pobiera informacje o statusie pakietów danych
// Parametry:
//	chKonfig - b0=0 Sub-GHz radio RTC wake-up disabled, b2=0 cold startup when exiting Sleep mode, configuration registers reset
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawSleep(uint8_t chKonfig)
{
	HAL_StatusTypeDef chErr;

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &chKonfig, 1);
	return chErr;
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

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &chKonfig, 1);

	//sprawdź czy raio weszło w standby
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
// Ustawia parametry pakietów
// Parametry: na razie nic
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryPakietow(uint8_t chTryb)
{
	HAL_StatusTypeDef chErr;
	uint8_t chKonfig[8] = {0};

	chKonfig[0] = 8;	//bytes 2:1 bits 15:0 PbLength[15:0]: Preamble length in number of symbols
	chKonfig[1] = 0;
	chKonfig[2] = 5;	//bits 2:0 PbDetLength[2:0]: Preamble detection length in number of bit symbols: 0x0: preamble detection disabled, 0x4: 8-bit preamble detection, 0x5: 16-bit preamble detection, 0x6: 24-bit preamble detection, 0x7: 32-bit preamble detection
	chKonfig[3] = 16;	//bits: 6:0 SyncWordLength[6:0]: Synchronization word length in number of bit symbols: 0x00 - 0x40: 0 to 64-bit synchronization word (synchronization word data defined in SUBGHZ_GSYNCR[0:7])
	chKonfig[4] = 0;	//bits: 1:0 AddrComp[1:0]: Address comparison/filtering, 0x0: address comparison/filtering disabled, 0x1: address comparison/filtering on node address, 0x2: address comparison/filtering on node and broadcast addresses
	chKonfig[5] = 1;	//bit 0 PktType: Packet type definition: 0: Fixed payload length and header field not added to packet, 1: Variable payload length and header field added to packet
	chKonfig[6] = 20;	//bits 7:0 PayloadLength[7:0]: Payload length in number of bytes 0x00- 0xFF: 0 to 255 bytes
	chKonfig[7] = 1;	//bits 2:0 CrcType[2:0]: CRC type definition The CRC initialization value is provided in SUBGHZ_GCRCINIRL and SUBGHZ_GCRCINIRH. The polynomial is defined in SUBGHZ_GCRCPOLRL and SUBGHZ_GCRCPOLRH.
						//0x0: 1-byte CRC, 0x1: no CRC, 0x2: 2-byte CRC, 0x4: 1-byte inverted CRC, 0x6: 2-byte inverted CRC
	chKonfig[8] = 0;	//bit 0 Whitening: Whitening enable. The whitening initial value is provided in WHITEINI[8:0]: 0: Whitening disabled, 1: Whitening enabled
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, chKonfig, 8);
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
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CFG_DIOIRQ, chKonfig, 8);
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



//////////////////////////////////////////////////////////////////////////////////
// funkcja skanuje pasmo i zwraca UARTem wartość zmierzonego RSSI dla badanych częstotliwosci
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t SkanujPasmo(void)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	//uint8_t chRssi[ROZMIAR_BUF_RSSI];
	int8_t chRssi;
	int8_t chRssiPakietu;
	int8_t chRssiSync;
	uint8_t chBuforUart[120];
	uint8_t chStatus1, chStatus2, chStatus3;
	uint16_t sRozmiar;
	uint8_t chStatusOdbioru;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RX);
	chErr = UstawSleep(0);	//b0=0 Sub-GHz radio RTC wake-up disabled, b2=0 cold startup when exiting Sleep mode, configuration registers reset
	chErr = UstawStandby(0);
	chErr = UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT, IRQ_RX_DONE, IRQ_TIMEOUT, IRQ_CAD_DETECT);
	chErr = KalibrujZakresCzestotliwosci(400, 800);

	//chErr = UstawTrybFallbaclk(FALLBACK_STDBY_HSE);	//tryb standby z właczonym HSE

	chErr = UstawTypPakietu(PAKIET_LORA);


	//chErr = PobierzTypPakietu(&chRssi[0], &chRssi[1]);	//sprawdź czy się zapisało
	//chErr |= UstawCzestotliwoscPLL(868000000);

	sRozmiar = sprintf((char*)chBuforUart, "Skaner radiowy 850 - 900 MHz\r\n");
	//sRozmiar = sprintf((char*)chBuforUart, "Skaner radiowy 400 - 450 MHz\r\n");
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);

	//dziele pasmo 850-900 MHz na 50 kawałków po 1 MHz
	for (uint16_t n=0; n<50; n++)
	{
		chErr = UstawStandby(0);

		chErr |= UstawCzestotliwoscPLL(850000000 + n * 1000000);
		//chErr |= UstawCzestotliwoscPLL(400000000 + n * 1000000);
		chErr |= UstawTrybSyntezy();

		BSP_LED_Toggle(LED_BLUE);
		chErr |= UstawTrybOdbioru(1000);	//timeout [us]

		HAL_Delay(100);

		chErr = PobierzStatus(&chStatus1);
		chErr |= ZmierzRSSI(&chStatus2, &chRssi);
		chErr = PobierzStatusPakietu(&chStatus3, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);

		sRozmiar = sprintf((char*)chBuforUart, "RSSI @ %d MHz: %d dBm, RSync:%d, RSSI Pakietu: %d, Status1: 0x%.2X, Status2: 0x%.2X, Status3: 0x%.2X\r\n", 850 + n * 10, chRssi, chRssiSync, chRssiPakietu, chStatus1, chStatus2, chStatus3);
		chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	}
	return chErr;
}




//////////////////////////////////////////////////////////////////////////////////
// funkcja skanuje pasmo i zwraca UARTem wartość zmierzonego RSSI dla badanych częstotliwosci
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WlaczObiorGFSK(void)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	int8_t chRssi;
	int8_t chRssiPakietu;
	int8_t chRssiSync;
	uint8_t chTimeout;
	uint8_t chStatus;
	uint16_t sRozmiar, sRozmiar2;
	uint8_t chStatusOdbioru;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RX);

	//Lista operacji aby przejsć w tryb RX str 205
	//The sub-GHz radio can be set in LoRa or (G)FSK receive operation mode with the following steps:
	//1. Define the location where the received payload data must be stored in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(0x80, 0x00);	//(Tx, Rx)

	//2. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr = UstawTypPakietu(PAKIET_FSK);

	//3. Define the frame format with Set_PacketParams().
	chErr = UstawParametryPakietow(0);

	//4. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	for (int8_t n=0; n<8; n++)
		chBuforUart[n] = 0x55;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x6C0, chBuforUart, 8);	//SUBGHZ_GSYNCR0

	//5. Define the RF frequency with Set_RfFrequency().
	chErr = UstawCzestotliwoscPLL(FREQ_GFSK);

	//6. Define the modulation parameters with Set_ModulationParams().
	chErr = UstawParametryModulacjiFSK(6, 9, BW_FSK467, 1000000);

	//7. Enable RxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr = UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET + IRQ_CRC_ERROR, IRQ_RX_DONE, IRQ_TIMEOUT + IRQ_CRC_ERROR, IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET);
	UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);

	//8. Start the receiver by setting the sub-GHz radio in RX mode with Set_Rx():
	//– When in continuous receiver mode, the sub-GHz radio remains in RX mode to look for packets until stopped with Set_Standby().
	//– In single mode (with or without timeout), when the reception is finished, the sub-GHz radio enters automatically the Standby mode.
	//– In listening mode, the sub-GHz radio repeatedly switches between RX single with timeout mode and Sleep mode.
	chErr |= UstawTrybOdbioru(1000);	//timeout [us]

	//9. Wait for sub-GHz radio IRQ interrupt and read the interrupt status with Get_IrqStatus():
	//a) On a RxDone interrupt, a packet is received:
	//– Check received packet error status (header error, crc error) with Get_IrqStatus().
	//– When a valid packet is received, read the receive start buffer pointer and received	payload length with Get_RxBufferStatus().
	//– Read the received payload data from the receive data buffer with Read_Buffer().
	chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);
	//b) On a timeout interrupt, the reception is timed out.
	//10. Clear interrupts with Clr_IrqStatus().
	chErr = KasujPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT);
	//11. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.

	chTimeout = TIMEOUT_ODB;
	do
	{
	  chTimeout--;
	  sRozmiar = 0;
	  switch (chStanProtokolu)
	  {
	  case RP_ODEBR_DANE:  	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano dane:", TIMEOUT_ODB - chTimeout);
	  	  for (uint8_t n=0; n<10; n++)
	  	  {
	  		sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, " %d,", chBuforOdbiorczy[n]);
	  		sRozmiar += sRozmiar2;
	  	  }
	  	  sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, "\n\r");
	  	  sRozmiar += sRozmiar2;
	  	  break;

	  case RP_ODEBR_SYNC:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano sync\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_ODEBR_NAGL:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano naglowek\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_ODEBR_PREAMB:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano preamule\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_BLAD_NAGL:	sRozmiar = sprintf((char*)chBuforUart, "%d: Blad naglowka\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_BLAD_CRC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad CRC\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_TIMEOUT:		sRozmiar = sprintf((char*)chBuforUart, "%d: Timeout\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_CAD:			sRozmiar = sprintf((char*)chBuforUart, "%d: CAD\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_HOP_LR_FHSS:	sRozmiar = sprintf((char*)chBuforUart, "%d: HOP_LR_FHSS\r\n", TIMEOUT_ODB - chTimeout);	  break;
		  break;
	  }
	  if (sRozmiar)
		  chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	  chStanProtokolu = 0;
	  HAL_Delay(1);
	}
	while (chTimeout);

	chErr |= ZmierzRSSI(&chStatus, &chRssi);
	chErr = PobierzStatusPakietu(&chStatus, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);

	sRozmiar = sprintf((char*)chBuforUart, "RSSI2B @ %dHz: %d dBm, RSync:%d, Status: 0x%.2X, RSSI Pakietu: %d\r\n", FREQ_GFSK, chRssi, chRssiSync, chStatus, chRssiPakietu);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	return ERR_OK;
}



//////////////////////////////////////////////////////////////////////////////////
// okresowo wysyła ramkę GFSK danych aby sprawdzić czy działa nadajnik
// Parametry: nic
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijRamkeGFSK(void)
{
	uint16_t sRozmiar;
	uint8_t chErr;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RFO_HP);

	//str 204
	//The sub-GHz radio can be set in LoRa, (G)MSK or (G)FSK transmit operation mode with the following steps:
	//1. Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(ROZMIAR_BUFORA_ODBIORCZEGO, 0x00);	//(Tx, Rx)

	//2. Write the payload data to the transmit data buffer with Write_Buffer().
	for (uint8_t n=0; n<ROZMIAR_BUFORA_NADAWCZEGO; n++)
		chBuforNadawczy[n] = n;
	chErr |= HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, chBuforNadawczy, ROZMIAR_BUFORA_NADAWCZEGO);	//zapełnij danymi
	chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);	//testuj czy się zapisało

	//3. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr |= UstawTypPakietu(PAKIET_FSK);

	//4. Define the frame format with Set_PacketParams().
	chErr |= UstawParametryPakietow(0);

	//5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	for (int8_t n=0; n<8; n++)
		chBuforUart[n] = 0x55;
	chErr |= HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x6C0, chBuforUart, 8);	//SUBGHZ_GSYNCR0

	//6. Define the RF frequency with Set_RfFrequency().
	chErr |= UstawCzestotliwoscPLL(FREQ_GFSK + chLicznikRamek);

	//7. Define the PA configuration with Set_PaConfig().
	//chErr = UstawParametryNadajnika(1, 0, 1);	//tabela str184 moc: +10dBm
	chErr |= UstawParametryNadajnika(7, 0, 1);	//tabela str184 moc: +15dBm

	//8. Define the PA output power and ramping with Set_TxParams().
	chErr |= UstawMocNadajnika(0x0E, 4);

	//9. Define the modulation parameters with Set_ModulationParams().
	chErr |= UstawParametryModulacjiFSK(100000, 9, 0x09, 0xFF);	//100kbps, środkowy shaping, max pasmo, dewiacja od czapy

	//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	chErr |= UstawPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_TX_DONE, IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_SYNC_DET);

	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	chErr |= UstawTrybNadawania(500);	//timeout

	//12. Wait for sub-GHz radio IRQ interrupt and read interrupt status with Get_IrqStatus():
	//a) On a TxDone interrupt, the packet is successfully sent
	//b) On a timeout interrupt, the transmission is timeout.
	sRozmiar = sprintf((char*)chBuforUart, "Wyslano ramke %d\r\n", chLicznikRamek++);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	HAL_Delay(50);
	BSP_LED_Toggle(LED_BLUE);
	//13. Clear interrupt with Clr_IrqStatus().
	chErr |= KasujPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET);
	//14. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.
	chErr |= UstawSleep(0);
	return chErr;
}




//////////////////////////////////////////////////////////////////////////////////
// okresowo wysyła ramkę LoRa danych aby sprawdzić czy działa nadajnik
// Parametry: nic
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijRamkeLoRa(void)
{
	uint16_t sRozmiar;
	uint8_t chErr;
	uint8_t chBuf[4];

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RFO_LP);

	//str 204
	//The sub-GHz radio can be set in LoRa, (G)MSK or (G)FSK transmit operation mode with the following steps:
	//1. Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(ROZMIAR_BUFORA_ODBIORCZEGO, 0x00);	//(Tx, Rx)

	//2. Write the payload data to the transmit data buffer with Write_Buffer().
	for (uint8_t n=0; n<ROZMIAR_BUFORA_NADAWCZEGO; n++)
		chBuforNadawczy[n] = n;
	chErr |= HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, chBuforNadawczy, ROZMIAR_BUFORA_NADAWCZEGO);	//zapełnij danymi

	//3. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr |= UstawTypPakietu(PAKIET_LORA);

	//4. Define the frame format with Set_PacketParams().
	chErr |= UstawParametryPakietow(0);

	//5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	for (int8_t n=0; n<8; n++)
		chBuforUart[n] = 0x55;
	chErr |= HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x6C0, chBuforUart, 8);	//SUBGHZ_GSYNCR0

	//6. Define the RF frequency with Set_RfFrequency().
	//chErr |= UstawCzestotliwoscPLL(FREQ_LORA + chLicznikRamek);
	chErr |= UstawCzestotliwoscPLL(FREQ_LORA);

	//7. Define the PA configuration with Set_PaConfig().
	//chErr = UstawParametryNadajnika(1, 0, 1);	//tabela str184 moc: +10dBm
	chErr |= UstawParametryNadajnika(7, 0, 1);	//tabela str184 moc: +15dBm

	//8. Define the PA output power and ramping with Set_TxParams().
	chErr |= UstawMocNadajnika(0x0E, 4);

	//9. Define the modulation parameters with Set_ModulationParams().
	chErr |= UstawParametryModulacjiLoRa(ROZPROSZ5, BW_LORA125, 0,  0);

	//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	chErr |= UstawPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_TX_DONE, IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_SYNC_DET);

	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	chErr |= UstawTrybNadawania(500);	//timeout

	chErr |= HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_IRQSTATUS, chBuf, 3);


	//12. Wait for sub-GHz radio IRQ interrupt and read interrupt status with Get_IrqStatus():
	//a) On a TxDone interrupt, the packet is successfully sent
	//b) On a timeout interrupt, the transmission is timeout.
	sRozmiar = sprintf((char*)chBuforUart, "Wyslano ramke %d\r\n", chLicznikRamek++);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	HAL_Delay(50);
	BSP_LED_Toggle(LED_BLUE);
	//13. Clear interrupt with Clr_IrqStatus().
	chErr |= KasujPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET);
	//14. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.

	chErr |= UstawSleep(0);
	return chErr;
}


//////////////////////////////////////////////////////////////////////////////////
// funkcja odbiera dane modulacją LoRa
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WlaczObiorLoRa(void)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	int8_t chRssi;
	int8_t chRssiPakietu;
	int8_t chRssiSync;
	uint8_t chTimeout;
	uint8_t chStatus;
	uint16_t sRozmiar, sRozmiar2;
	uint8_t chStatusOdbioru;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RX);
	chErr = UstawCAD();

	//Lista operacji aby przejsć w tryb RX str 205
	//The sub-GHz radio can be set in LoRa or (G)FSK receive operation mode with the following steps:
	//1. Define the location where the received payload data must be stored in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(0x80, 0x00);	//(Tx, Rx)

	//2. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr = UstawTypPakietu(PAKIET_LORA);

	//3. Define the frame format with Set_PacketParams().
	chErr = UstawParametryPakietow(0);

	//4. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	for (int8_t n=0; n<8; n++)
		chBuforUart[n] = 0x55;
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x6C0, chBuforUart, 8);	//SUBGHZ_GSYNCR0

	//5. Define the RF frequency with Set_RfFrequency().
	chErr = UstawCzestotliwoscPLL(FREQ_LORA);

	//6. Define the modulation parameters with Set_ModulationParams().
	chErr |= UstawParametryModulacjiLoRa(ROZPROSZ5, BW_LORA125, 0,  0);

	//7. Enable RxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr = UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET + IRQ_CRC_ERROR, IRQ_RX_DONE, IRQ_TIMEOUT + IRQ_CRC_ERROR, IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET);
	UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);

	//8. Start the receiver by setting the sub-GHz radio in RX mode with Set_Rx():
	//– When in continuous receiver mode, the sub-GHz radio remains in RX mode to look for packets until stopped with Set_Standby().
	//– In single mode (with or without timeout), when the reception is finished, the sub-GHz radio enters automatically the Standby mode.
	//– In listening mode, the sub-GHz radio repeatedly switches between RX single with timeout mode and Sleep mode.
	chErr |= UstawTrybOdbioru(200000);	//timeout [us]

	if (chErr == ERR_OK)
	{
	//9. Wait for sub-GHz radio IRQ interrupt and read the interrupt status with Get_IrqStatus():
	//a) On a RxDone interrupt, a packet is received:
	//– Check received packet error status (header error, crc error) with Get_IrqStatus().
	//– When a valid packet is received, read the receive start buffer pointer and received	payload length with Get_RxBufferStatus().
	//– Read the received payload data from the receive data buffer with Read_Buffer().
		chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);
	}

	//b) On a timeout interrupt, the reception is timed out.
	//10. Clear interrupts with Clr_IrqStatus().
	chErr = KasujPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT);
	//11. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.

	chTimeout = TIMEOUT_ODB;
	do
	{
	  chTimeout--;
	  sRozmiar = 0;
	  switch (chStanProtokolu)
	  {
	  case RP_ODEBR_DANE:  	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano dane:", TIMEOUT_ODB - chTimeout);
	  	  for (uint8_t n=0; n<10; n++)
	  	  {
	  		sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, " %d,", chBuforOdbiorczy[n]);
	  		sRozmiar += sRozmiar2;
	  	  }
	  	  sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, "\n\r");
	  	  sRozmiar += sRozmiar2;
	  	  break;

	  case RP_ODEBR_SYNC:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano sync\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_ODEBR_NAGL:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano naglowek\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_ODEBR_PREAMB:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano preamule\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_BLAD_NAGL:	sRozmiar = sprintf((char*)chBuforUart, "%d: Blad naglowka\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_BLAD_CRC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad CRC\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_TIMEOUT:		sRozmiar = sprintf((char*)chBuforUart, "%d: Timeout\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_CAD:			sRozmiar = sprintf((char*)chBuforUart, "%d: CAD\r\n", TIMEOUT_ODB - chTimeout);	  break;
	  case RP_HOP_LR_FHSS:	sRozmiar = sprintf((char*)chBuforUart, "%d: HOP_LR_FHSS\r\n", TIMEOUT_ODB - chTimeout);	  break;
		  break;
	  }
	  if (sRozmiar)
		  chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	  chStanProtokolu = 0;
	  HAL_Delay(1);
	}
	while (chTimeout);

	chErr |= ZmierzRSSI(&chStatus, &chRssi);
	chErr = PobierzStatusPakietu(&chStatus, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);

	sRozmiar = sprintf((char*)chBuforUart, "RSSI LoRa2B @ %dHz: %d dBm, RSync:%d, Status: 0x%.2X, RSSI Pakietu: %d\r\n", FREQ_LORA, chRssi, chRssiSync, chStatus, chRssiPakietu);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	return ERR_OK;
}



//////////////////////////////////////////////////////////////////////////////////
// funkcja wysyła nośną
// Parametry:
// 	nCzestotliwosc - w Hz
//	czas_ms - czas transmisji nośnej w ms
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t NadawajNosna(uint32_t nCzestotliwosc, uint32_t czas_ms)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[8];
	uint32_t nRejCzest;

	nRejCzest = (uint32_t)(((double)nCzestotliwosc * (1UL << 25)) / 32000000.0);
	chBuforDanych[0] = (nRejCzest >> 24) & 0xFF;
	chBuforDanych[1] = (nRejCzest >> 16) & 0xFF;
	chBuforDanych[2] = (nRejCzest >> 8) & 0xFF;
	chBuforDanych[3] = (nRejCzest) & 0xFF;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, chBuforDanych, 4);
	if (chErr != HAL_OK) return 1;

	//Ustaw moc i ramp time
	//Power: 14 dBm, Ramp: 40 µs (0x04)
	chBuforDanych[0] = 14;   // moc w dBm
	chBuforDanych[1] = 0x04; // ramp time = 40 µs
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, chBuforDanych, 2);
	if (chErr != HAL_OK) return 2;

	//Ustaw radio w tryb "Continuous Wave" (czysta nośna)
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
	if (chErr != HAL_OK) return 3;

	//Trzymaj nośną przez zadany czas
	HAL_Delay(czas_ms);
	BSP_LED_Toggle(LED_BLUE);

	//Powrót do standby
	uint8_t mode = 0x00; // Standby RC
	HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &mode, 1);
	return ERR_OK;
}



//////////////////////////////////////////////////////////////////////////////////
// funkcja wysyła preambułę
// Parametry:
// 	nCzestotliwosc - w Hz
//	czas_ms - czas transmisji nośnej w ms
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t NadawajPrembule(uint32_t nCzestotliwosc, uint32_t czas_ms)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4];
	uint32_t nRejCzest;

	nRejCzest = (uint32_t)(((double)nCzestotliwosc * (1UL << 25)) / 32000000.0);
	chBuforDanych[0] = (nRejCzest >> 24) & 0xFF;
	chBuforDanych[1] = (nRejCzest >> 16) & 0xFF;
	chBuforDanych[2] = (nRejCzest >> 8) & 0xFF;
	chBuforDanych[3] = (nRejCzest) & 0xFF;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, chBuforDanych, 4);
	if (chErr != HAL_OK) return 1;

	//Ustaw moc i ramp time
	//Power: 14 dBm, Ramp: 40 µs (0x04)
	chBuforDanych[0] = 14;   // moc w dBm
	chBuforDanych[1] = 0x04; // ramp time = 40 µs
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, chBuforDanych, 2);
	if (chErr != HAL_OK) return 2;

	//Ustaw radio w tryb "Continuous Preamble"
	chErr = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
	if (chErr != HAL_OK) return 3;

	//Trzymaj nośną przez zadany czas
	HAL_Delay(czas_ms);
	BSP_LED_Toggle(LED_BLUE);

	//Powrót do standby
	uint8_t mode = 0x00; // Standby RC
	HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &mode, 1);
	return ERR_OK;
}

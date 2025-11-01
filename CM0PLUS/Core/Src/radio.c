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


extern SUBGHZ_HandleTypeDef hsubghz;
extern UART_HandleTypeDef huart1;



//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego nadawania
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_TxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}



//////////////////////////////////////////////////////////////////////////////////
// Callback zakończonego odbioru
//////////////////////////////////////////////////////////////////////////////////
void HAL_SUBGHZ_RxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_PreambleDetectedCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_SyncWordValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_HeaderValidCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_HeaderErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_CRCErrorCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_CADStatusCallback(SUBGHZ_HandleTypeDef *hsubghz, HAL_SUBGHZ_CadStatusTypeDef cadstatus)
{

}

void HAL_SUBGHZ_RxTxTimeoutCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

}

void HAL_SUBGHZ_LrFhssHopCallback(SUBGHZ_HandleTypeDef *hsubghz)
{

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
	union u32_8_t Unia32_8;

	Unia32_8.nDane32 = (uint32_t)(nTimeout / 15.625) & 0xFFFFFF;
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, Unia32_8.chDane8, 3);	//sprawdzić czy unia dobrze się przelicza
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
	union u32_8_t Unia32_8;

	Unia32_8.nDane32 = (uint32_t)(nTimeout / 15.625);
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, Unia32_8.chDane8, 3);	//sprawdzić czy unia dobrze się przelicza
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

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_FS, &chBuforPolecen, 0);
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
	uint64_t lCzestotliwosc;

	lCzestotliwosc = ((uint64_t)nCzestotliwosc * (1UL << 25)) / 32e6;
	chBuforDanych[0] = (lCzestotliwosc >> 24) & 0xFF;
	chBuforDanych[1] = (lCzestotliwosc >> 16) & 0xFF;
	chBuforDanych[2] = (lCzestotliwosc >> 8) & 0xFF;
	chBuforDanych[3] = (lCzestotliwosc) & 0xFF;

	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, chBuforDanych, 4);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// Mierzy wartość SSI gdy jest w trybie odbioru
// Parametry:
//	*chStatus - wskaźnik na status radia
//	*chRSSI - wskaźnik na zwracaną wartość zmierzonego RSSI
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzRSSI(uint8_t* chStatus, uint8_t* chRSSI)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[2] = {0};


	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RSSIINST, chBuforDanych, 2);
	if (chErr == ERR_OK)
	{
		*chStatus = chBuforDanych[1];
		*chRSSI = chBuforDanych[0];
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
//	chSpeadingFactor - określa liczbę bitów na symbol z zakresu 5..12
//	chKsztaltImpulsu - wartość filtra kstałtujacego impulsy: 0=bez filtra, 8=Gaussian BT 0.3; 9=Gaussian BT 0.5; 10=Gaussian BT 0.7; 11=Gaussian BT 1.0;
//	chSzerokoscPasma - patrz definicje stałych: BW4..BW467
//	nOdchylCzestotliwosci - Frequency deviation x 225 / 32 MHz
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryModulacjiFSK(uint8_t chSpeadingFactor, uint8_t chKsztaltImpulsu, uint8_t chSzerokoscPasma, uint32_t nOdchylCzestotliwosci)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4];
	union u32_8_t Unia32_8;

	chBuforDanych[0] = chSpeadingFactor & 0x0F;
	chBuforDanych[4] = chSzerokoscPasma;
	Unia32_8.nDane32 = nOdchylCzestotliwosci & 0xFFFFFF;
	for (uint8_t n=0; n<3; n++)
		chBuforDanych[n+5] = Unia32_8.chDane8[n];
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 8);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// ustawia parametry modulacji LoRa. Musi być poprzedzone funkcją: UstawTypPakietu()
// Parametry:
//	nPredkoscBit = 32 * 32MHz / prędkość transmisji
//	chKsztaltImpulsu - wartość filtra kstałtujacego impulsy: 0=bez filtra, 8=Gaussian BT 0.3; 9=Gaussian BT 0.5; 10=Gaussian BT 0.7; 11=Gaussian BT 1.0;
//	chSzerokoscPasma - patrz definicje stałych: BW4..BW467
//	nOdchylCzestotliwosci - Frequency deviation x 225 / 32 MHz
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t UstawParametryModulacjiLoRa(uint32_t nPredkoscBit, uint8_t chKsztaltImpulsu, uint8_t chSzerokoscPasma, uint32_t nOdchylCzestotliwosci)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[8];
	union u32_8_t Unia32_8;

	Unia32_8.nDane32 = nPredkoscBit & 0xFFFFFF;
	for (uint8_t n=0; n<3; n++)
		chBuforDanych[n] = Unia32_8.chDane8[n];
	chBuforDanych[3] = chKsztaltImpulsu & 0x0F;
	chBuforDanych[4] = chSzerokoscPasma;
	Unia32_8.nDane32 = nOdchylCzestotliwosci & 0xFFFFFF;
	for (uint8_t n=0; n<3; n++)
		chBuforDanych[n+5] = Unia32_8.chDane8[n];
	chErr =  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, chBuforDanych, 8);
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
//	*chBlad - wskaźnik na błąd
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStatusPakietu(uint8_t *chStatus, uint8_t *chStatusOdbioru, uint8_t *chStatusRSSISync, uint8_t *chSrednRSSI)
{
	HAL_StatusTypeDef chErr;
	uint8_t chBuforDanych[4] = {0};

	chErr =  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, chBuforDanych, 4);
	if (chErr == ERR_OK)
	{
		*chSrednRSSI = chBuforDanych[0];
		*chStatusRSSISync = chBuforDanych[1];
		*chStatus = chBuforDanych[2];
		*chStatusOdbioru = chBuforDanych[3];
	}
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// funkcja skanuje pasmo i zwraca UARTem wartość zmierzonego RSSI dla badanych częstotliwosci
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t SkanujPasmo(void)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	uint8_t chRssi[ROZMIAR_BUF_RSSI];
	uint8_t chBuforUart[60];
	uint8_t chStatus;
	uint16_t sRozmiar;
	uint8_t chStatusOdbioru;
	uint8_t chStatusRSSISync;

	chErr = UstawTrybFallbaclk(FALLBACK_STDBY_HSE);	//tryb standby z właczonym HSE
	chErr = UstawTypPakietu(PAKIET_LORA);
	chErr = PobierzTypPakietu(&chRssi[0], &chRssi[1]);	//sprawdź czy się zapisało

	chErr |= UstawCzestotliwoscPLL(868000000);

	//dziele pasmo 150-960 MHz na kawałki po 10MHz
	for (uint16_t n=0; n<ROZMIAR_BUF_RSSI; n++)
	{
		chErr |= UstawCzestotliwoscPLL(150000000 + n * 10000000);
		chErr |= UstawTrybSyntezy();

		chErr = PobierzTypPakietu(&chRssi[0], &chRssi[1]);	//sprawdź czy się zapisało

		chErr = PobierzBlad(&chStatus, &chRssi[3]);

		BSP_LED_Toggle(LED_BLUE);
		chErr |= UstawTrybOdbioru(0);	//timeout [us]
		HAL_Delay(200);
		chErr |= ZmierzRSSI(&chStatus, &chRssi[n]);
		chErr = PobierzStatusPakietu(&chStatus, &chRssi[n], &chStatusRSSISync, &chStatusOdbioru);
		sRozmiar = sprintf((char*)chBuforUart, "RSSI @ %d MHz: %d dBm, RSSI Sync:%d, Status: %X\r\n", 150 + n * 10, (int8_t)chRssi[n] / (-2), chStatusRSSISync, chStatusOdbioru);
		chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);

	}
	return chErr;
}

//HAL_StatusTypeDef SUBGHZSPI_Transmit(SUBGHZ_HandleTypeDef *hsubghz, uint8_t Data)
//HAL_StatusTypeDef SUBGHZSPI_Receive(SUBGHZ_HandleTypeDef *hsubghz, uint8_t *pData)

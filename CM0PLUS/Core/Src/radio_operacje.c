//////////////////////////////////////////////////////////////////////////////////
//  Moduł operacji podstawowych na radiu
//
//  Created on: 19 lis 2025
//  Author: Piotr
//////////////////////////////////////////////////////////////////////////////////
#include <radio_operacje.h>
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_nucleo_radio.h"
#include "czas.h"
#include "errors.h"


extern SUBGHZ_HandleTypeDef hsubghz;
extern UART_HandleTypeDef huart1;
extern volatile uint8_t chStanProtokolu;
uint8_t chLicznikRamek;
uint8_t chBuforUart[120];
uint8_t chBuforUart2[40];
uint8_t chBuforNadawczy[ROZMIAR_BUFORA_NADAWCZEGO];
uint8_t chBuforOdbiorczy[ROZMIAR_BUFORA_ODBIORCZEGO];
uint8_t SyncWord[SYNC_WORD_LEN] = {0xC1, 0x94, 0xC1, 0xC1};	//standardowy Semtech
//uint8_t SyncWord[SYNC_WORD_LEN] = {0x49, 0x54, 0x57, 0x4C};	//standardowy ITWL
//uint8_t SyncWord[SYNC_WORD_LEN] = {0xC1, 0x94, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1};
uint8_t SyncWordLoRa[2] = {0x00, 0x12};  // 0x12 - prywatny, 0x34 = public LoRa
uint32_t nCzas, nPoprzedniCzas, nCzasTx, nCzasTxRx;

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
	chErr |= UstawStandby(0);
	//chErr |= UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT, IRQ_RX_DONE, IRQ_TIMEOUT, IRQ_CAD_DETECT);
	chErr |= UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);
	chErr |= KalibrujZakresCzestotliwosci(400, 800);

	//chErr = UstawTrybFallbaclk(FALLBACK_STDBY_HSE);	//tryb standby z właczonym HSE

	chErr |= UstawTypPakietu(PAKIET_LORA);
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

		chErr |= PobierzStatus(&chStatus1);
		chErr |= ZmierzRSSI(&chStatus2, &chRssi);
		chErr |= PobierzStatusPakietu(&chStatus3, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);

		sRozmiar = sprintf((char*)chBuforUart, "RSSI @ %d MHz: %d dBm, RSync:%d, RSSI Pakietu: %d, Status1: 0x%.2X, Status2: 0x%.2X, Status3: 0x%.2X\r\n", 850 + n * 10, chRssi, chRssiSync, chRssiPakietu, chStatus1, chStatus2, chStatus3);
		chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	}
	return chErr;
}




//////////////////////////////////////////////////////////////////////////////////
// funkcja skanuje pasmo i zwraca UARTem wartość zmierzonego RSSI dla badanych częstotliwosci
// Parametry: nTimeout timeout oczekiwania na odbiór w mikrosekundach
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WlaczObiorGFSK(uint32_t nTimeout)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	int8_t chRssiPakietu;
	int8_t chRssiSync;
	uint16_t sTimeout;
	//uint16_t sStatusPrzerwania = 0;
	uint8_t chStatus, chBlad;
	uint8_t chIloscOdebrana, chWskaznikDanych;
	uint16_t sRozmiar, sRozmiar2;
	uint8_t chStatusOdbioru;

	//zapełnij bufory wzorcem
	for (uint8_t n=0; n<ROZMIAR_BUFORA_ODBIORCZEGO; n++)
	{
		chBuforNadawczy[n] = 0x11;
		chBuforOdbiorczy[n] = 0x22;
	}
	chErr |= HAL_SUBGHZ_WriteBuffer(&hsubghz, ADR_BUF_ODB, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);	//zapełnij danymi

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RX);

	//Lista operacji aby przejsć w tryb RX RM0453 Rev 6 str 205
	//The sub-GHz radio can be set in LoRa or (G)FSK receive operation mode with the following steps:
	//1. Define the location where the received payload data must be stored in the data buffer, with Set_BufferBaseAddress().
	//chErr = UstawAdresyBuforow(ADR_BUF_NAD, ADR_BUF_ODB);	//(Tx, Rx)
	chErr = UstawAdresyBuforow(0x00, 0x05);	//(Tx, Rx)

	//2. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr = UstawTypPakietu(PAKIET_FSK);

	//3. Define the frame format with Set_PacketParams().
	chErr = UstawParametryPakietowGFSK(PREAMBULA_LEN_GFSK, SYNC_WORD_LEN, PAYLOAD_LEN_GFSK);		//sDlugPreamb, chSyncWordlLength, chPayloadLenght

	//4. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GSYNCR0, SyncWord, SYNC_WORD_LEN);	//SUBGHZ_GSYNCR0

	//5. Define the RF frequency with Set_RfFrequency().
	chErr = UstawCzestotliwoscPLL(FREQ_GFSK);

	//6. Define the modulation parameters with Set_ModulationParams().
	//chErr = UstawParametryModulacjiFSK(1200, 0,  BW_FSK4,  2000);
	//chErr = UstawParametryModulacjiFSK(6, 9, BW_FSK467, 1000000);
	//chErr = UstawParametryModulacjiFSK(9600, 0,  BW_FSK19,  8000);
	//chErr |= UstawParametryModulacjiFSK(9600, 0,  BW_FSK39,  16000);	//9600bps, bez shapingu, pasmo 39k, dewiacja 16k
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK58,  25000);		//z przykładu PinPong
	//chErr |= UstawParametryModulacjiFSK(52000, FSK_SHAPE_BT05,  BW_FSK467,  26000);	//
	//chErr |= UstawParametryModulacjiFSK(60000, FSK_SHAPE_BT05,  BW_FSK467,  30000);	//
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK373,  25000);	//max transfer Tx 65300
	//chErr |= UstawParametryModulacjiFSK(106971, FSK_SHAPE_BT05,  BW_FSK117,  40114);	//max transfer
	//chErr |= UstawParametryModulacjiFSK(58650, FSK_SHAPE_BT05,  BW_FSK117,  29325);	//max transfer - brak
	chErr |= UstawParametryModulacjiFSK(89257, FSK_SHAPE_BT05,  BW_FSK117,  33471);	//max transfer Tx 61200, dla rmaki 250B=132kbps


	//7. Enable RxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr = UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET + IRQ_CRC_ERROR, IRQ_RX_DONE, IRQ_TIMEOUT + IRQ_CRC_ERROR, IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET);
	UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);

	//8. Start the receiver by setting the sub-GHz radio in RX mode with Set_Rx():
	//– When in continuous receiver mode, the sub-GHz radio remains in RX mode to look for packets until stopped with Set_Standby().
	//– In single mode (with or without timeout), when the reception is finished, the sub-GHz radio enters automatically the Standby mode.
	//– In listening mode, the sub-GHz radio repeatedly switches between RX single with timeout mode and Sleep mode.
	chErr |= UstawTrybOdbioru(nTimeout);	//timeout [us]

	sTimeout = 0;
	do
	{
		//9. Wait for sub-GHz radio IRQ interrupt and read the interrupt status with Get_IrqStatus():
		//a) On a RxDone interrupt, a packet is received:
		chStanProtokolu = 0;
		sRozmiar = 0;
		do HAL_Delay(1);
		while (chStanProtokolu == 0);

		//– Check received packet error status (header error, crc error) with Get_IrqStatus().
		switch (chStanProtokolu)
		{
		case RP_ODEBR_DANE:
			//– When a valid packet is received, read the receive start buffer pointer and received	payload length with Get_RxBufferStatus().
			//chErr |= PobierzBlad(&chStatus, &chBlad);
			chErr |= PobierzStatusBufora(&chIloscOdebrana, &chWskaznikDanych);

			//– Read the received payload data from the receive data buffer with Read_Buffer().
			sRozmiar = sprintf((char*)chBuforUart, "Odebrano: [%d B] status: [0x%.2X] Blad: [0x%.2X]:", chIloscOdebrana, chStatus, chBlad);
			chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, ADR_BUF_ODB, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);	//odczytaj bufor
			chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, ADR_BUF_NAD, chBuforNadawczy, ROZMIAR_BUFORA_NADAWCZEGO);	//odczytaj bufor

			chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, ADR_BUF_ODB, chBuforOdbiorczy + chWskaznikDanych, chIloscOdebrana);	//odczytaj bufor
			/*for (uint8_t n=0; n<chIloscOdebrana; n++)
			{
				sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, " %.2X,", chBuforOdbiorczy[chWskaznikDanych + n]);
				sRozmiar += sRozmiar2;
			}
			sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, "\n\r");
			sRozmiar += sRozmiar2;*/
			KasujPrzerwnie(IRQ_TX_DONE);
			break;

		case RP_ODEBR_SYNC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano sync\r\n", sTimeout);	  break;
		case RP_ODEBR_NAGL:		sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano naglowek\r\n", sTimeout);	  break;
		case RP_ODEBR_PREAMB:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano preamule\r\n", sTimeout);	  break;
		case RP_BLAD_NAGL:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad naglowka\r\n", sTimeout);	break;
		case RP_BLAD_CRC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad CRC\r\n", sTimeout);	  	break;

		//b) On a timeout interrupt, the reception is timed out.
		case RP_TIMEOUT:		sRozmiar = sprintf((char*)chBuforUart, "%d: Timeout\r\n", sTimeout);		break;
		case RP_CAD:			sRozmiar = sprintf((char*)chBuforUart, "%d: CAD\r\n", sTimeout);	  		break;
		case RP_HOP_LR_FHSS:	sRozmiar = sprintf((char*)chBuforUart, "%d: HOP_LR_FHSS\r\n", sTimeout);	break;
		  break;
		}
		//if (sRozmiar)
			//chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
		sTimeout++;

		//10. Clear interrupts with Clr_IrqStatus().
		//chErr = KasujPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT);
		//chErr = KasujPrzerwnie(sStatusPrzerwania);

	}
	while ((chStanProtokolu == 0) && (sTimeout < 5000));
	if (sTimeout < 5000)
		chErr = ERR_TIMEOUT;

	chErr |= PobierzBlad(&chStatus, &chBlad);
	chErr |= PobierzStatusPakietu(&chStatus, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);
	sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, "GFSK @ %dkHz: RSSIsync:%d dBm, Status: 0x%.2X\r\n", FREQ_GFSK/1000, chRssiSync, chStatus);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar2 + sRozmiar, 10);
	//11. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.
	chErr |= UstawSleep(0);
	return chErr;
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


	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RFO_LP);

	//RM0453 Rev 6 str 204
	//The sub-GHz radio can be set in LoRa, (G)MSK or (G)FSK transmit operation mode with the following steps:
	//1. Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(0x00, 250);	//(Tx, Rx)

	//2. Write the payload data to the transmit data buffer with Write_Buffer().
	for (uint8_t n=0; n<ROZMIAR_BUFORA_NADAWCZEGO; n++)
		chBuforNadawczy[n] = n;

	chErr |= HAL_SUBGHZ_WriteBuffer(&hsubghz, ADR_BUF_NAD, chBuforNadawczy, 250);	//zapełnij danymi

	//3. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr |= UstawTypPakietu(PAKIET_FSK);

	//4. Define the frame format with Set_PacketParams().
	chErr |= UstawParametryPakietowGFSK(PREAMBULA_LEN_GFSK, SYNC_WORD_LEN, PAYLOAD_LEN_GFSK);	//sDlugPreamb, chSyncWordlLength, chPayloadLenght

	//5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GSYNCR0, SyncWord, SYNC_WORD_LEN);	//SUBGHZ_GSYNCR0

	//6. Define the RF frequency with Set_RfFrequency().
	chErr |= UstawCzestotliwoscPLL(FREQ_GFSK);

	//7. Define the PA configuration with Set_PaConfig().
	chErr = UstawParametryNadajnika(1, 0, 1);	//tabela str184 moc: +10dBm		chWypelnienieCyklu, uint8_t HpMax, uint8_t chZakresMocy
	//chErr |= UstawParametryNadajnika(7, 0, 1);	//tabela str184 moc: +15dBm

	//8. Define the PA output power and ramping with Set_TxParams().
	chErr |= UstawMocNadajnika(0x00, 2);

	//9. Define the modulation parameters with Set_ModulationParams().
	//chErr = UstawParametryModulacjiFSK(1200, 0,  BW_FSK4,  2000);
	//chErr = UstawParametryModulacjiFSK(9600, 0,  BW_FSK19,  8000);
	//chErr |= UstawParametryModulacjiFSK(9600, 0,  BW_FSK39,  16000);	//9600bps, bez shapingu, pasmo 39k, dewiacja 16k
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK58,  25000);	//max transfer 2759 bps
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK117,  25000);	//max transfer 2686
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK312,  25000);	//max transfer 2686
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK312,  30000);	//max transfer 2722
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK58,  30000);	//max transfer 2759
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK58,  40000);	//max transfer - brak
	//chErr |= UstawParametryModulacjiFSK(60000, FSK_SHAPE_BT05,  BW_FSK58,  30000);	//max transfer

	//chErr |= UstawParametryModulacjiFSK(58650, FSK_SHAPE_BT05,  BW_FSK117,  29325);	//max transfer Tx 65200
	//chErr |= UstawParametryModulacjiFSK(29300, FSK_SHAPE_BT05,  BW_FSK58,  14650);	//max transfer - brak
	//chErr |= UstawParametryModulacjiFSK(104133, FSK_SHAPE_BT05,  BW_FSK117,  26033);	//max transfer - brak
	chErr |= UstawParametryModulacjiFSK(89257, FSK_SHAPE_BT05,  BW_FSK117,  33471);	//max transfer Tx 61200, dla ramki 250B = 132k
	//chErr |= UstawParametryModulacjiFSK(106971, FSK_SHAPE_BT05,  BW_FSK117,  40114);	//max transfer
	//chErr |= UstawParametryModulacjiFSK(124800, FSK_SHAPE_BT05,  BW_FSK117,  31200);	//max transfer
	//chErr |= UstawParametryModulacjiFSK(93600, FSK_SHAPE_BT05,  BW_FSK117,  46800);	//max transfer
	//chErr |= UstawParametryModulacjiFSK(117150, FSK_SHAPE_BT05,  BW_FSK117,  58575);	//max transfer
	//chErr |= UstawParametryModulacjiFSK(156000, FSK_SHAPE_BT05,  BW_FSK117,  78000);	//max transfer - brak, blokuje się na nadawaniu

	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK373,  25000);	//max transfer Tx 65300
	//chErr |= UstawParametryModulacjiFSK(50000, FSK_SHAPE_BT05,  BW_FSK467,  25000);	//max transfer Tx 65300
	//chErr |= UstawParametryModulacjiFSK(60000, FSK_SHAPE_BT05,  BW_FSK467,  30000);	// nie idzie
	//chErr |= UstawParametryModulacjiFSK(52000, FSK_SHAPE_BT05,  BW_FSK467,  26000);	// sporadycznie cos odbierze


	//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr |= UstawPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_TX_DONE, IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_SYNC_DET);
	chErr |= UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);

	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	nCzas = PobierzCzas();
	chErr |= UstawTrybNadawania(TIMEOUT_NADAJNIKA);	//timeout

	//12. Wait for sub-GHz radio IRQ interrupt and read interrupt status with Get_IrqStatus():
	//a) On a TxDone interrupt, the packet is successfully sent
	//b) On a timeout interrupt, the transmission is timeout.

	nCzasTx = MinalCzas(nCzas);
	nCzasTxRx = MinalCzas2(nPoprzedniCzas, nCzas);
	nPoprzedniCzas = nCzas;

	//sRozmiar = sprintf((char*)chBuforUart, "Ramka Tx: %d, Czas TxRx: %ldus, Transfer: %ld, Czas Tx:%ld, Transfer Tx: %ld\r\n", chLicznikRamek++, nCzasTxRx, PAYLOAD_LEN_GFSK * 1000000 / nCzasTxRx, nCzasTx, PAYLOAD_LEN_GFSK * 1000000 / nCzasTx);
	sRozmiar = sprintf((char*)chBuforUart, "Ramka Tx: %d, Czas:%ld, Transfer: %ld\r\n", chLicznikRamek++, nCzasTx, PAYLOAD_LEN_GFSK * 1000000 / nCzasTx);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);

	//13. Clear interrupt with Clr_IrqStatus().
	//chErr |= KasujPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET);
	//14. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.
	chErr |= UstawSleep(0);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// funkcja odbiera dane modulacją LoRa
// Parametry:
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WlaczObiorLoRa(uint32_t nTimeout)
{
	HAL_StatusTypeDef chErr = ERR_OK;
	int8_t chRssi;
	int8_t chRssiPakietu;
	int8_t chRssiSync;
	uint8_t chStatus;
	uint16_t sRozmiar, sRozmiar2;
	uint16_t sTimeout = 0;
	uint8_t chStatusOdbioru;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RX);
	chErr = UstawCAD();
	chStanProtokolu = 0;

	//Lista operacji aby przejsć w tryb RX: RM0453 Rev 6 str 205
	//The sub-GHz radio can be set in LoRa or (G)FSK receive operation mode with the following steps:
	//1. Define the location where the received payload data must be stored in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(ADR_BUF_NAD, ADR_BUF_ODB);	//(Tx, Rx)

	//2. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr |= UstawTypPakietu(PAKIET_LORA);
	HAL_Delay(1);

	//3. Define the frame format with Set_PacketParams().
	chErr |= UstawParametryPakietowLoRa(PREAMBULA_LEN_LORA, 0, PAYLOAD_LEN_LORA, CRC_EN_LORA, 0);	//sDlugPreamb, chStalyNagl, chPayloadLenght, chWlaczCRC, chInvertIQ

	//4. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	//chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GSYNCR0, SyncWord, SYNC_WORD_LEN);	//SUBGHZ_GSYNCR0
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x0910, SyncWordLoRa, 2);

	//5. Define the RF frequency with Set_RfFrequency().
	chErr = UstawCzestotliwoscPLL(FREQ_LORA);

	//6. Define the modulation parameters with Set_ModulationParams().
	chErr |= UstawParametryModulacjiLoRa(ROZPROSZ7, BW_LORA125, 1,  0);

	//7. Enable RxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr = UstawPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET + IRQ_CRC_ERROR, IRQ_RX_DONE, IRQ_TIMEOUT + IRQ_CRC_ERROR, IRQ_CAD_DETECT + IRQ_CAD_DETECT + IRQ_PREAMB_DET + IRQ_SYNC_DET);
	UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);
	//UstawPrzerwnie(IRQ_TIMEOUT, IRQ_TIMEOUT, 0, 0);


	//8. Start the receiver by setting the sub-GHz radio in RX mode with Set_Rx():
	//– When in continuous receiver mode, the sub-GHz radio remains in RX mode to look for packets until stopped with Set_Standby().
	//– In single mode (with or without timeout), when the reception is finished, the sub-GHz radio enters automatically the Standby mode.
	//– In listening mode, the sub-GHz radio repeatedly switches between RX single with timeout mode and Sleep mode.

	uint32_t nPrimMask = __get_PRIMASK();
	//uint32_t nBaseMask = __get_BASEPRI();
	//uint32_t nFaultMask = __get_FAULTMASK();
	sRozmiar = nPrimMask;	//blokuj ostrzezenie o nieużywanej zmiennej

	chErr |= KasujPrzerwnie(0x3FF);
	chErr |= UstawTrybOdbioru(nTimeout);	//timeout [us]

	//9. Wait for sub-GHz radio IRQ interrupt and read the interrupt status with Get_IrqStatus():
	//a) On a RxDone interrupt, a packet is received:
	//– Check received packet error status (header error, crc error) with Get_IrqStatus().
	//– When a valid packet is received, read the receive start buffer pointer and received	payload length with Get_RxBufferStatus().
	//– Read the received payload data from the receive data buffer with Read_Buffer().
		//chErr |= ZmierzRSSI(&chStatus, &chRssi);
		//chErr |= HAL_SUBGHZ_ReadBuffer(&hsubghz, ADR_BUF_ODB, chBuforOdbiorczy, ROZMIAR_BUFORA_ODBIORCZEGO);

	//b) On a timeout interrupt, the reception is timed out.
	//10. Clear interrupts with Clr_IrqStatus().
	//chErr |= KasujPrzerwnie(IRQ_RX_DONE + IRQ_TIMEOUT + IRQ_CAD_DETECT);

	sRozmiar = 0;
	do
	{
		HAL_Delay(1);
		sTimeout++;
		switch (chStanProtokolu)
		{
		case RP_ODEBR_DANE:  	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano dane:", sTimeout);
			/*for (uint8_t n=0; n<10; n++)
			{
			sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, " %d,", chBuforOdbiorczy[n]);
			sRozmiar += sRozmiar2;
			}
			sRozmiar2 = sprintf((char*)chBuforUart + sRozmiar, "\n\r");
			sRozmiar += sRozmiar2;*/
			break;

		case RP_ODEBR_SYNC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano sync\r\n", sTimeout);	  break;
		case RP_ODEBR_NAGL:		sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano naglowek\r\n", sTimeout);	  break;
		case RP_ODEBR_PREAMB:	sRozmiar = sprintf((char*)chBuforUart, "%d: Odebrano preamule\r\n", sTimeout);	  break;
		case RP_BLAD_NAGL:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad naglowka\r\n", sTimeout);	  break;
		case RP_BLAD_CRC:		sRozmiar = sprintf((char*)chBuforUart, "%d: Blad CRC\r\n", sTimeout);	  break;
		case RP_TIMEOUT:		sRozmiar = sprintf((char*)chBuforUart, "%d: Timeout\r\n", sTimeout);	  break;
		case RP_CAD:			sRozmiar = sprintf((char*)chBuforUart, "%d: CAD\r\n", sTimeout);	  break;
		case RP_HOP_LR_FHSS:	sRozmiar = sprintf((char*)chBuforUart, "%d: HOP_LR_FHSS\r\n", sTimeout);	  break;
		  break;
		}
		if (sRozmiar)
		  chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	}
	while ((chStanProtokolu == 0) && (sTimeout < nTimeout/1000 + 10));
	if (sTimeout < nTimeout/1000 + 10)
		chErr = ERR_TIMEOUT;

	uint16_t sStatusPrzerwania = 0;
	chErr |= PobierzStatusPrzerwania(&chStatus, &sStatusPrzerwania);
	nPrimMask = __get_PRIMASK();
	ZmierzRSSI(&chStatus, &chRssi);
	chErr |= PobierzStatusPakietu(&chStatus, &chStatusOdbioru, &chRssiSync, &chRssiPakietu);
	sRozmiar2 = sprintf((char*)chBuforUart+sRozmiar, "RSSI LoRa2B @ %dHz: %d dBm, RSync:%d, Status: 0x%.2X, RSSI Pakietu: %d\r\n", FREQ_LORA, chRssi, chRssiSync, chStatus, chRssiPakietu);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar + sRozmiar2, 10);

	//11. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.
	chErr |= UstawSleep(0);
	return chErr;
}



//////////////////////////////////////////////////////////////////////////////////
// wysyła ramkę danych z modulacją LoRa
// Parametry: nic
// Zwraca: kod błędu
//////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijRamkeLoRa(void)
{
	uint16_t sRozmiar;
	uint8_t chErr;
	//uint8_t chStatus;
	//uint16_t sStatusIRQ;

	BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RFO_LP);

	//str 204
	//The sub-GHz radio can be set in LoRa, (G)MSK or (G)FSK transmit operation mode with the following steps:
	//1. Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
	chErr = UstawAdresyBuforow(ADR_BUF_NAD, ADR_BUF_ODB);	//(Tx, Rx)

	//2. Write the payload data to the transmit data buffer with Write_Buffer().
	for (uint8_t n=0; n<ROZMIAR_BUFORA_NADAWCZEGO; n++)
		chBuforNadawczy[n] = n;
	chErr |= HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, chBuforNadawczy, ROZMIAR_BUFORA_NADAWCZEGO);	//zapełnij danymi

	//3. Select the packet type (generic or LoRa) with Set_PacketType().
	chErr |= UstawTypPakietu(PAKIET_LORA);
	HAL_Delay(1);

	//4. Define the frame format with Set_PacketParams().
	chErr |= UstawParametryPakietowLoRa(PREAMBULA_LEN_LORA, 0, PAYLOAD_LEN_LORA, CRC_EN_LORA, 0);	//sDlugPreamb, chStalyNagl, chPayloadLenght, chWlaczCRC, chInvertIQ

	//5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
	//chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GSYNCR0, SyncWord, SYNC_WORD_LEN);	//SUBGHZ_GSYNCR0
	chErr = HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x0910, SyncWordLoRa, 2);

	//6. Define the RF frequency with Set_RfFrequency().
	//chErr |= UstawCzestotliwoscPLL(FREQ_LORA + chLicznikRamek);
	chErr |= UstawCzestotliwoscPLL(FREQ_LORA);

	//7. Define the PA configuration with Set_PaConfig().
	//chErr = UstawParametryNadajnika(1, 0, 1);	//tabela str184 moc: +10dBm
	chErr |= UstawParametryNadajnika(7, 0, 1);	//tabela str184 moc: +15dBm

	//8. Define the PA output power and ramping with Set_TxParams().
	chErr |= UstawMocNadajnika(0x0E, 2);

	//9. Define the modulation parameters with Set_ModulationParams().
	chErr |= UstawParametryModulacjiLoRa(ROZPROSZ7, BW_LORA125, 1,  0);		//rozproszenie widma, chSzerokoscPasma, chKorekcjaBledow, chOptymalizacja

	//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
	//chErr |= UstawPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_TX_DONE, IRQ_TIMEOUT + IRQ_CAD_DETECT + IRQ_CAD_DONE, IRQ_SYNC_DET);
	chErr |= UstawPrzerwnie(0x3FF, 0x3FF, 0, 0);

	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	chErr |= UstawTrybNadawania(TIMEOUT_NADAJNIKA);	//timeout

	//12. Wait for sub-GHz radio IRQ interrupt and read interrupt status with Get_IrqStatus():
	//a) On a TxDone interrupt, the packet is successfully sent
	//b) On a timeout interrupt, the transmission is timeout.
	sRozmiar = sprintf((char*)chBuforUart, "Wyslano ramke %d\r\n", chLicznikRamek++);
	chErr |= HAL_UART_Transmit(&huart1,  chBuforUart, sRozmiar, 10);
	//13. Clear interrupt with Clr_IrqStatus().
	//chErr |= KasujPrzerwnie(IRQ_TX_DONE + IRQ_TIMEOUT + IRQ_SYNC_DET);
	//14. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.

	chErr |= UstawSleep(0);
	return chErr;
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

	chErr = UstawCzestotliwoscPLL(nCzestotliwosc);
	if (chErr != HAL_OK) return 1;

	//Ustaw moc i ramp time
	//Power: 14 dBm, Ramp: 40 µs (0x04)
	chErr = UstawMocNadajnika(14, 0x04);
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

	//UstawParametryModulacjiLoRa(ROZPROSZ5, BW_LORA52, 0, 0);
	UstawParametryModulacjiFSK(9600, 0,  BW_FSK19,  8000);
	//UstawParametryModulacjiFSK(9600, 0,  BW_FSK39,  16000);
	//UstawParametryModulacjiFSK(9600, 0,  BW_FSK78,  32000);
	//UstawParametryModulacjiFSK(19200, 0,  BW_FSK156,  64000);
	chErr = UstawParametryModulacjiFSK(10000, FSK_SHAPE_BT05,  BW_FSK117,  30000);

	chErr = UstawCzestotliwoscPLL(nCzestotliwosc);
	if (chErr != HAL_OK) return 1;

	//Ustaw moc i ramp time
	//Power: 14 dBm, Ramp: 40 µs (0x04)
	chErr = UstawMocNadajnika(14, 0x04);
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

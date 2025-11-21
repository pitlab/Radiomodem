//////////////////////////////////////////////////////////////////////////////
//
// Obsługa odmierzania i synchronizacji czasu
//
// (c) PitLab 20245
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "czas.h"
#include "errors.h"


extern TIM_HandleTypeDef htim17;


////////////////////////////////////////////////////////////////////////////////
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzCzas(void)
{
	extern volatile uint16_t sCzasH;
	return htim17.Instance->CNT + ((uint32_t)sCzasH <<16);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nPoczatek)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = PobierzCzas();
	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// nKoniec - licznik czasu na na końcu pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec)
{
	uint32_t nCzas;

	if (nKoniec >= nPoczatek)
		nCzas = nKoniec - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nKoniec;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Czekaj z timeoutem dopóki zmienna ma wartość niezerową
// Parametry: chZajety - zmienna, która ma się ustawić na zero
// nCzasOczekiwania - czas przez jaki czekamy na wyzerowanie się zmiennej w mikrosekundach
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaZero(uint8_t chZajety, uint32_t nCzasOczekiwania)
{
	uint32_t nPoczatek, nCzas;

	nPoczatek = PobierzCzas();
	do
	{
		nCzas = MinalCzas(nPoczatek);
	}
	while(chZajety && (nCzas < nCzasOczekiwania));
	if (!chZajety)
		return ERR_OK;

	return ERR_TIMEOUT;
}







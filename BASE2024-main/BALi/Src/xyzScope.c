/*
 * xyzScope.c
 *
 *  Created on: 03.01.2024
 *      Author: tobia
 */
#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <math.h>
#include <ST7735.h>
#include "xyzScope.h"


void XYZ2AlphaBeta(int16_t *XYZ_raw, float *AlphaBeta)
{
		float X = (float) XYZ_raw[0]/160;  //Skalierung 10mg/digit at +-2g
		float Y = (float) XYZ_raw[1]/160;
		float Z = (float) XYZ_raw[2]/160;

		AlphaBeta[0] = atan(X/Z);
		if (Z<0)
		{
			if (X<0)  {AlphaBeta[0] -=_pi;}
			else {AlphaBeta[0] +=_pi;}
		}

		AlphaBeta[1] = atan(Y/Z);
		if (Z<0)
		{
			if (Y<0)  {AlphaBeta[1] -=_pi;}
			else {AlphaBeta[1] +=_pi;}
		}
}

uint16_t AlBeScreen(float *AlphaBeta)
{
	// grafic emulation
#define aDots 6							// Count of Dots for alpha line
#define bDots 12

	//const float _pi=3.141;
	const int16_t a_midxyl[3] = {30,40,14};			// alpha circle xpos, ypos ,length
	const int16_t b_midxyl[3] = {100,40,38};		// beta circle xpos, ypos ,length
	const uint16_t aColor = tft_YELLOW;
	const uint16_t bColor = tft_MAGENTA;

	static int16_t j, XYa[2][aDots], XYb[2][bDots];								// aDots Wertepaare für Winkeldarstellung alpha
	float xgrad, ygrad;

	// Variables for Oszi Function
	const int16_t oszi[3] = {92,28,159};			//oszi ypos-Zero Level, y-amplitude, t_lenght
	const uint16_t osziColor = tft_GREY;

	static int16_t timepos = 0;


	xgrad = cos(AlphaBeta[0])*a_midxyl[2];
	ygrad = sin(AlphaBeta[0])*a_midxyl[2];
	for (j = 0; j<aDots; j++ )
	{
		tftDrawPixel(XYa[0][j],XYa[1][j],tft_BLACK);
		XYa[0][j] = lround(a_midxyl[0] + xgrad*(2*j - (float)aDots+1)/(aDots-1));			//(aDots-1) da j nur bis aDost -1 läuft
		XYa[1][j] = lround(a_midxyl[1] + ygrad*(2*j - (float)aDots+1)/(aDots-1));
		tftDrawPixel(XYa[0][j],XYa[1][j],aColor);
	}
	//lcd7735_drawPixel(a_midxyl[0],a_midxyl[1],ST7735_RED);

	xgrad = cos(AlphaBeta[1])*b_midxyl[2];
	ygrad = sin(AlphaBeta[1])*b_midxyl[2];
	for (j = 0; j<bDots; j++ )
	{
		tftDrawPixel(XYb[0][j],XYb[1][j],tft_BLACK);
		XYb[0][j] = lround(b_midxyl[0] + xgrad*(2*j - (float)bDots+1)/(bDots-1));			//(aDots-1) da j nur bis aDost -1 läuft
		XYb[1][j] = lround(b_midxyl[1] + ygrad*(2*j - (float)bDots+1)/(bDots-1));
		tftDrawPixel(XYb[0][j],XYb[1][j],bColor);
	}



// kleines Oszi als Zeitmitschrieb
	int16_t Ya = oszi[0] - (int16_t)(oszi[1] * AlphaBeta[0]/_pi);			// - ST7735 y = 0 upper line inverter direct to y Scale
	int16_t Yb = oszi[0] - (int16_t)(oszi[1] * AlphaBeta[1]/_pi);
	int16_t osziHight = oszi[1]*2;
	tftDrawFastVLine(timepos, (oszi[0]-oszi[1]), osziHight, osziColor);
	if (Ya == Yb)
	{
		tftDrawPixel(timepos,Ya,tft_WHITE);
	}
	else
	{
		tftDrawPixel(timepos,Ya,aColor);
		tftDrawPixel(timepos,Yb,bColor);
	}
	if (++timepos > oszi[2] )
	{
		timepos = 0;
		//lcd7735_fillRect(0, oszi[0]-oszi[1], oszi[2]+1, 2*oszi[1], ST7735_GREY);
	}
	tftDrawFastVLine(timepos, oszi[0], oszi[1]/2, tft_RED);
	return timepos;
}

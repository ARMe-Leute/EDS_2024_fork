/**
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <ST7735.h>
#include <RotaryPushButton.h>

#define GRAFIK_TESTS 21
unsigned int testcount = GRAFIK_TESTS;

int intTest = 32;
double doubleTest=34.123456789;
float floatTest=32.34;

bool pressed = false;
bool timerTrigger = false;
int position = 0;
int oldPosition = 0;

int corner1_x = 0;
int corner1_y = 0;

int corner2_x = 140;
int corner2_y = 0;

int corner3_x = 140;
int corner3_y = 100;

int corner4_x = 0;
int corner4_y = 100;

int corner_pos = 0;
int old_corner_pos = 1;

int test_value = 0;

uint32_t    ST7735_Timer = 0UL; //Timer used for delayms function

void menu(void);
void testGraphics(void);
void corner(int corner_pos);

int main(void)
{
	//Inits needed for TFT Display
	systickInit(SYSTICK_1MS);
	spiInit();
	tftInitR(INITR_REDTAB);

	//display setup
    tftSetRotation(LANDSCAPE);
    tftSetFont((uint8_t *)&SmallFont[0]);
    tftFillScreen(tft_BLACK);

    //push button setup
    initRotaryPushButton();

    //Choose which Function you want to test
    //For testGraphics() comment menu()
    //For menu() comment while loop which includes testGraphics()

    /*while (1)
    {
		   testGraphics();
		   delayms(5000);
    }
    */

    menu();
}

//Menüstruktur
void menu(void)
{
	for(;;)
	{
		/* gets the current state of the push button and the current rotary position
		 * (the rotary position is just there to show the functionality and is not used for further purpose) */
		pressed = getRotaryPushButton();
		oldPosition = position;
		position = getRotaryPosition();

		/* if the push button was pressed, the led will turn to green for one loop */
		if (pressed == true)
		{
			setRotaryColor(LED_GREEN);
		}
		else
		{
			setRotaryColor(LED_RED);
		}

		//check rot direction
		/*if (position > oldPosition)
		{
			setRotaryColor(LED_MAGENTA);
		}
		 */

		// in corner 4 change test_value value
		if(corner_pos == 4 && pressed)
		{
			while(true)
			{

			oldPosition = position;
			pressed = getRotaryPushButton();
			position = getRotaryPosition();
			tftFillRect(corner4_x,corner4_y,32,12,tft_BLACK);
			if ((position - 1)==oldPosition || (position - 2)==oldPosition)
			{
				test_value++;
			}
			else if((position + 1)==oldPosition || (position + 2)==oldPosition)
			{
				test_value--;
			}
			tftPrintInt(test_value,corner4_x,corner4_y,0);
			delayms(600);
			if(pressed)
			{
				break;
			}
			}
		}

		if ((position - 1)==oldPosition || (position - 2)==oldPosition)
		{
			setRotaryColor(LED_YELLOW);

			if(old_corner_pos == 4)
			{
				corner_pos = 1;
			}
			else
			{
			corner_pos++;
			}

			corner(corner_pos);
			tftPrintInt(corner_pos,20,20,0);
		}

		else if((position + 1)==oldPosition || (position + 2)==oldPosition)
		{
			setRotaryColor(LED_BLACK);

			if(old_corner_pos == 1)
			{
				corner_pos = 4;
			}
			else
			{
			corner_pos--;
			}


			corner(corner_pos);

			tftPrintInt(corner_pos,20,20,0);

		}/* if the push button was pressed, the led will turn to green for one loop */

		delayms(200);

	}

}

// all different functions for LCD display are tested here
void testGraphics(void)
{
	switch (testcount--)
	{
		case 21:
		{
			tftSetRotation(LANDSCAPE);
			tftSetFont((uint8_t *)&SmallFont[0]);
			tftFillScreen(tft_BLACK);
			tftPrint("Hi the 1st output",0,0,0);
			break;

		}

		// print text on multiple pages in big font
		case 20:
		{
			tftFillScreen(tft_BLACK);
			tftSetRotation(PORTRAIT);
			tftInvertDisplay(INVERT_ON);
			tftSetFont((uint8_t *)&BigFont[0]);
			tftPrint("We don't need no education We dont need no thought control"
						  "No dark sarcasm in the classroom Teacher, leave them kids alone"
					      "Hey, teacher, leave them kids alone"
					      "All in all, its just another brick in the wall"
					      "All in all, youre just another brick in the wall ",0,0,0);
			break;
		}

		// print integer in sevensegment font
		case 19:
		{
			tftFillScreen(tft_BLACK);
			int number=12345;
			tftSetFont((uint8_t *)&SevenSegNumFont[0]);
			tftPrintInt(number,0,60,0);
			break;
		}

		// print text with different background
		case 18:
		{
			tftSetFont((uint8_t *)&SmallFont[0]);
			tftFillScreen(tft_MAGENTA);
			tftPrint("Hello!",20,10,0);
			break;
		}

		// print with rotation
		case 17:
		{
			tftPrint("37deg Hello!",10,5,37);
			break;
		}

		// print picture
		case 16:
		{
			/*tftDrawBitmap(0,0,50,52,(bitmapdatatype)tux_50_ad,1);
			break;*/
		}
		case 15:
		{
			/*tftDrawBitmap(55,0,50,52,(bitmapdatatype)tux_50_ad,2);
			break;*/
		}
		// print picture in portrait mode

		case 14:
		{
			/*tftSetRotation(PORTRAIT);
			tftDrawBitmap(0,0,50,52,(bitmapdatatype)tux_50_ad,1);
			break;*/
		}

		// invert displaycolors
		case 13:
		{
			tftInvertDisplay(INVERT_ON);
			break;
		}
		case 12:
		{
			tftInvertDisplay(INVERT_OFF);
			break;
		}

		// fill screen with different colors
		case 11:
		{
			tftFillScreen(tft_RED);
			break;
		}
		case 10:
		{
			tftFillScreen(tft_GREEN);
			break;
		}
		case  9:
		{
			tftFillScreen(tft_BLUE);
			break;
		}
		case  8:
		{
			tftFillScreen(tft_BLACK);
			break;
		}

		// draw different shapes
		case  7:
		{
			tftFillRect(20,15,40,35,tft_BLUE);
			break;
		}
		case  6:
		{
			tftFillCircle(80,64,30,tft_YELLOW);
			break;
		}
		case  5:
		{
			tftDrawRect(10,20,90,100,tft_MAGENTA);
			break;
		}
		case  4:
		{
			tftSetRotation(LANDSCAPE);
			tftDrawCircle(tftGetWidth()/2,tftGetHeight()/2,35,tft_CYAN);
			break;
		}
		case  3:
		{
			tftDrawFastLine(10,5,110,120,tft_WHITE);
			break;
		}

		// invert displaycolors
		case  2:
		{
			tftInvertDisplay(INVERT_ON);
			break;
		}
		case  1:
		{
			tftInvertDisplay(INVERT_OFF);
			break;
		}

		default:
		{
			testcount = GRAFIK_TESTS;
			tftFillScreen(tft_BLACK);
			break;
		}
	}
}

void corner(int corner_pos)
{
	switch (corner_pos)
	{
		case 1:
		{
		tftFillRect(corner4_x,corner4_y,32,12,tft_BLACK);
		tftFillRect(corner3_x,corner3_y,32,12,tft_BLACK);
		tftFillRect(corner2_x,corner2_y,32,12,tft_BLACK);
		tftPrintInt(position,corner1_x,corner1_y,0);
		old_corner_pos = 1;
		break;
		}
		case 2:
		{
		tftFillRect(corner4_x,corner4_y,32,12,tft_BLACK);
		tftFillRect(corner3_x,corner3_y,32,12,tft_BLACK);
		tftFillRect(corner1_x,corner1_y,32,12,tft_BLACK);
		tftPrintInt(position,corner2_x,corner2_y,0);
		old_corner_pos = 2;
		break;
		}
		case 3:
		{
		tftFillRect(corner4_x,corner4_y,32,12,tft_BLACK);
		tftFillRect(corner2_x,corner2_y,32,12,tft_BLACK);
		tftFillRect(corner1_x,corner1_y,32,12,tft_BLACK);
		tftPrintInt(position,corner3_x,corner3_y,0);
		old_corner_pos = 3;
		break;
		}
		case 4:
		{
		tftFillRect(corner2_x,corner2_y,32,12,tft_BLACK);
		tftFillRect(corner3_x,corner3_y,32,12,tft_BLACK);
		tftFillRect(corner1_x,corner1_y,32,12,tft_BLACK);
		tftPrintInt(position,corner4_x,corner4_y,0);
		old_corner_pos = 4;
		break;
		}
	}
}


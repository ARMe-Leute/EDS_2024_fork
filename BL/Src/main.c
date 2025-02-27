/**
 * @file main.c
 * @brief  Demo program for the Bluetooth library.
 *
 * This program demonstrates the use of the Bluetooth library. After setup, pressing the button
 * on the rotary encoder lights it up blue while checking the HM17 module's status. If the status
 * is OK, the button lights up green; otherwise, it lights up red. After two seconds, the light turns off.
 *
 * @author c0deberry
 * @author nrs00
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <mcalSysTick.h>
#include <mcalUsart.h>
#include <mcalGPIO.h>
#include <mcalDMAC.h>

#include <bluetooth.h>
#include <RotaryPushButton.h>
#include <ST7735.h>
#include <menu.h>

/**
 * @brief Time interval for Bluetooth setup steps (in milliseconds).
 *
 * Each step in the Bluetooth setup process is separated by this interval.
 * The USART pins produce noise when activated, which can confuse the HM17 module.
 * A delay of about 500 ms is recommended to avoid this issue.
 */
#define BLUETOOTH_SETUP_TIME 500 //ms

/**
 * @brief Application modes.
 *
 * Used to differentiate between initialization and the main loop.
 */
typedef enum
   {
   MAIN_INIT = 0,
   MAIN_LOOP
   } MAIN_MODE;

/**
 * @brief Timer trigger flag, updated in the SysTick interrupt handler.
 *
 * This flag indicates that the base time interval has elapsed, and
 * all timers need to be updated.
 */
bool timerTrigger;

volatile char usart2BufferRX[USART2_BUFFER_SIZE];
volatile char usart2BufferTX[USART2_BUFFER_SIZE];
volatile uint16_t usart2BufferIndex = 0;

uint32_t ST7735_Timer = 0UL;

MenuManager_t menuManager_1;
MenuPage_t submenu1;
MenuPage_t submenu2;
MenuPage_t submenu3;
MenuPage_t submenu4;
MenuEntry_t feldBack =
   {
   .color = tft_WHITE, .title = "BACK", .type = Back, .page = NULL
   };
MenuEntry_t subfeld1 =
   {
   .color = tft_WHITE, .title = "Features", .type = Page, .page = &submenu1
   };
MenuEntry_t subfeld2 =
   {
   .color = tft_WHITE, .title = "Nested", .type = Page, .page = &submenu2
   };
MenuEntry_t subfeld3 =
   {
   .color = tft_WHITE, .title = "submenu_3_", .type = Page, .page = &submenu3
   };
MenuEntry_t subfeld4 =
   {
   .color = tft_WHITE, .title = "submenu_4_", .type = Page, .page = &submenu4
   };
MenuEntry_t feld2 =
   {
   .color = tft_BLUE, .title = "Text", .type = Entry, .page = NULL
   };
MenuEntry_t feld3 =
   {
   .color = tft_RED, .title = "Counter", .type = Entry, .page = NULL
   };
MenuEntry_t feld4 =
   {
   .color = tft_GREEN, .title = "GetStatus", .type = Entry, .page = NULL
   };

MenuPage_t menuPage1;

int main(void)
    {

    uint32_t BluetoothTimer = 0UL;      // Timer for Bluetooth setup steps.
    uint32_t BluetoothFetchTimer = 0UL; // Timer for calling bluetoothFetchBuffer().
    uint32_t Button = 0UL;// Timer for button polling, helps with debouncing.
    uint32_t ButtonLEDOff = 0UL;        // Timer for turning off the button LED.

    uint32_t *timerList[] =
        {
        &BluetoothTimer, &BluetoothFetchTimer, &Button, &ButtonLEDOff, &ST7735_Timer
        };
    uint8_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

        BluetoothModule_t HM17_1;   // Bluetooth module instance.
int initStatus = -100;


      int lastRotaryPosition;

      menuPage1.TL = &subfeld1;
      menuPage1.TR = &subfeld2;
      menuPage1.BL = &subfeld3;
      menuPage1.BR = &subfeld4;

      submenu1.TL = &feldBack;
      submenu1.BL = &feld2;
      submenu1.BR = &feld3;
      submenu1.TR = &feld4;

      submenu2.TL = &feldBack;
      submenu2.BL = &subfeld1;
      submenu2.BR = &feld3;
      submenu2.TR = &feld4;

      submenu3.TL = &feldBack;
      submenu3.BL = &feld3;
      submenu3.BR = &feld2;
      submenu3.TR = &feld4;

      submenu4.TL = &feldBack;
      submenu4.BL = &feld4;
      submenu4.BR = &feld3;
      submenu4.TR = &feld2;


      menuManager_1.activeMode = Page;
      menuManager_1.activePage = &menuPage1;

      MAIN_MODE mode = MAIN_INIT;


      for (;;)
         {
            switch (mode)
               {
               // Initialization mode
               case MAIN_INIT:
                  systickInit(SYSTICK_1MS); // Configure SysTick with a 1 ms interval.

                  HM17_1.initStatus = -10;
                  initRotaryPushButton();
                  initRotaryPushButtonLED();
                  systickSetTicktime(&Button, 20);

                  spiInit();
                  tftInitR(INITR_REDTAB);

                  //display setup
                  tftSetRotation(LANDSCAPE);
                  tftSetFont((uint8_t*) &SmallFont[0]);
                  tftFillScreen(tft_BLACK);
                  tftSetColor(tft_WHITE, tft_BLACK);
                  tftPrint((char*) "Bluetooth Test", 0, 0, 0);

                  while (initStatus < 0)
                     {
                        if (timerTrigger == true)
                           {
                              systickUpdateTimerList((uint32_t*) timerList, arraySize);

                           }
                        if (isSystickExpired(BluetoothFetchTimer))
                           {
                              bluetoothFetchBuffer(&HM17_1);

                              systickSetTicktime(&BluetoothFetchTimer,
                              BLUETOOTH_FETCH_TIME);
                           }
                        if (isSystickExpired(BluetoothTimer))
                           {
                              initStatus = bluetoothInit(&HM17_1, USART2, 9600, usart2BufferTX );
                              systickSetTicktime(&BluetoothTimer, BLUETOOTH_SETUP_TIME);
                           }
                     }




                  mode = MAIN_LOOP; // Transition to main loop
                  showMenuPage(&menuManager_1, (MenuPosition_t)0);
                  break;

                  // Main application loop
               case MAIN_LOOP:
                  if (timerTrigger == true)
                     {
                        systickUpdateTimerList((uint32_t*) timerList, arraySize);

                     }
                  if (isSystickExpired(BluetoothFetchTimer))
                     {
                        bluetoothFetchBuffer(&HM17_1);

                        systickSetTicktime(&BluetoothFetchTimer,
                        BLUETOOTH_FETCH_TIME);
                     }

                  if (isSystickExpired(Button))
                     {
                        systickSetTicktime(&Button, 200);
                        switch (menuManager_1.activeMode)
                           {
                           case Page:
                              if (getMenuRotaryPosition(getRotaryPosition())
                                    != menuManager_1.currentPosition)
                                 {
                                    menuManager_1.currentPosition = getMenuRotaryPosition(
                                          getRotaryPosition());
                                    lastRotaryPosition = getMenuRotaryPosition(getRotaryPosition());
                                    drawGrid();
                                    higlightEntry(menuManager_1.currentPosition);
                                 }
                              if (getRotaryPushButton() == true)
                                 {
                                    if (getEntryFromPosition(&menuManager_1,
                                          menuManager_1.currentPosition)->type == Page)
                                       {
                                          getEntryFromPosition(&menuManager_1,
                                                menuManager_1.currentPosition)->page->lastMenu =
                                                menuManager_1.activePage;
                                          menuManager_1.activePage =
                                                getEntryFromPosition(&menuManager_1,
                                                      menuManager_1.currentPosition)->page;
                                       }
                                    else if (getEntryFromPosition(&menuManager_1,
                                          menuManager_1.currentPosition)->type == Entry)
                                       {
                                          menuManager_1.activeEntry = getEntryFromPosition(
                                                &menuManager_1, menuManager_1.currentPosition);
                                          menuManager_1.activeMode = Entry;
                                       }
                                    else
                                       {
                                          menuManager_1.activePage =
                                                menuManager_1.activePage->lastMenu;
                                       }
                                    showMenuPage(&menuManager_1, menuManager_1.currentPosition);
                                 }
                              break;
                           case Entry:
                              if (menuManager_1.activeEntry == &feld2)
                                 {
                                    tftPrint((char*) "sdofihv", 50, 50, 0);
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              else if (menuManager_1.activeEntry == &feld3)
                                 {
                                    tftPrintInt(getRotaryPosition(), 50, 50, 0);
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              else if (menuManager_1.activeEntry == &feld4)
                                 {
                                    static int status = -127;
                                    static bool reply;
                                    static bool active = false;

                                    if (getRotaryPushButton() == true)
                                       {
                                          if (status >= 0)
                                             {
                                                status = -127;
                                                active = false;
                                                menuManager_1.activeMode = Page;
                                                showMenuPage(&menuManager_1,
                                                      menuManager_1.currentPosition);
                                             }
                                          else
                                             {
                                                active = true;
                                                tftPrint((char*) "Getting Status...", 0, 50, 0);
                                             }

                                       }
                                    if (active == true)
                                       {
                                          status = bluetoothGetStatus(&HM17_1, &reply);
                                          if (status == BluetoothFinish)
                                             {
                                                if (reply == true)
                                                   {
                                                      tftPrintColor((char*) "OK", 0, 60, tft_GREEN);
                                                   }
                                                else
                                                   {
                                                      tftPrintColor((char*) "Unknown Error", 0, 60,
                                                      tft_RED);
                                                   }
                                                active = false;
                                             }
                                          else if (status > 0)
                                             {
                                                tftPrintColor((char*) "Error:", 0, 60, tft_RED);
                                                tftPrintInt(status, 0, 70, 0);
                                                active = false;
                                             }
                                       }
                                 }
                              else
                                 {
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);
                                       }
                                 }
                              break;
                           }

                     }

               }

            if (isSystickExpired(ButtonLEDOff))
               {

                  setRotaryColor(LED_BLACK); // Turn off LED after timeout.
               }

         }
   }

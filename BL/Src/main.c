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
volatile uint16_t usart2BufferIndex = 0;

//(Length of name plus max size of number + semicolon) times entrys plus + newline +  terminator
volatile char usart2BufferTX[(BLUETOOTH_MAX_NAME_LENGTH + 21)*BLUETOOTH_NUMBER_OF_LOG_ENTRYS + 2];
volatile bool usart2TXComplete;

uint32_t ST7735_Timer = 0UL;

MenuManager_t menuManager_1;
MenuPage_t featuresPage;
MenuPage_t nestedPage;
MenuPage_t bluetoothPage;
MenuPage_t submenu4;
MenuEntry_t feldBack =
   {
   .color = tft_WHITE, .title = "BACK", .type = Back, .page = NULL
   };
MenuEntry_t featuresEntry =
   {
   .color = tft_WHITE, .title = "Features", .type = Page, .page = &featuresPage
   };
MenuEntry_t nestedEntry =
   {
   .color = tft_WHITE, .title = "Nested", .type = Page, .page = &nestedPage
   };
MenuEntry_t bluetoothEntry =
   {
   .color = tft_WHITE, .title = "Bluetooth", .type = Page, .page = &bluetoothPage
   };
MenuEntry_t subfeld4 =
   {
   .color = tft_WHITE, .title = "submenu_4_", .type = Page, .page = &submenu4
   };
MenuEntry_t textEntry =
   {
   .color = tft_BLUE, .title = "Text", .type = Entry, .page = NULL
   };
MenuEntry_t counterEntry =
   {
   .color = tft_RED, .title = "Counter", .type = Entry, .page = NULL
   };
MenuEntry_t getStatusEntry =
   {
   .color = tft_GREEN, .title = "GetStatus", .type = Entry, .page = NULL
   };
MenuEntry_t sendTestStringEntry =
   {
   .color = tft_GREEN, .title = "SendString", .type = Entry, .page = NULL
   };
MenuEntry_t setBaudRateEntry =
   {
   .color = tft_GREEN, .title = "Set BAUD", .type = Entry, .page = NULL
   };
MenuPage_t menuPage1;

int main(void)
   {

      uint32_t BluetoothTimer = 0UL;      // Timer for Bluetooth setup steps.
      uint32_t BluetoothFetchTimer = 0UL; // Timer for calling bluetoothFetchBuffer().
      uint32_t BluetoothLogTimer = 0UL;
      uint32_t Button = 0UL; // Timer for button polling, helps with debouncing.
      uint32_t ButtonLEDOff = 0UL;        // Timer for turning off the button LED.

      uint32_t *timerList[] =
         {
         &BluetoothTimer, &BluetoothFetchTimer, &Button, &ButtonLEDOff, &ST7735_Timer, &BluetoothLogTimer
         };
      uint8_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

      BluetoothModule_t HM17_1;   // Bluetooth module instance.
      int initStatus = -100;

      uint32_t runtime = 0;
      int32_t rotaryPosition = 0;

     // HM17_1.logEntrys[0].name = (char*)"runtime\0";
      HM17_1.logEntrys[0].type = BluetoothLogEntryType_uint32_t;
      HM17_1.logEntrys[0].data.uint32_ptr = &runtime;
      HM17_1.logEntrys[1].type = BluetoothLogEntryType_int32_t;
      HM17_1.logEntrys[1].data.int32_ptr = &rotaryPosition;

      int lastRotaryPosition = 0;

      menuPage1.TL = &featuresEntry;
      menuPage1.TR = &nestedEntry;
      menuPage1.BL = &bluetoothEntry;
      menuPage1.BR = &subfeld4;

      featuresPage.TL = &feldBack;
      featuresPage.BL = &textEntry;
      featuresPage.BR = &counterEntry;
      featuresPage.TR = &getStatusEntry;

      nestedPage.TL = &feldBack;
      nestedPage.BL = &featuresEntry;
      nestedPage.BR = &counterEntry;
      nestedPage.TR = &getStatusEntry;

      bluetoothPage.TL = &feldBack;
      bluetoothPage.BL = &sendTestStringEntry;
      bluetoothPage.BR = &setBaudRateEntry;
      bluetoothPage.TR = &getStatusEntry;

      submenu4.TL = &feldBack;
      submenu4.BL = &getStatusEntry;
      submenu4.BR = &counterEntry;
      submenu4.TR = &textEntry;

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
				if (isSystickExpired(BluetoothTimer)) {
					initStatus = bluetoothInit(&HM17_1, USART2, bluetoothBaud_9600, usart2BufferTX);
					systickSetTicktime(&BluetoothTimer, BLUETOOTH_SETUP_TIME);
				}
				if (isSystickExpired(BluetoothFetchTimer)) {
					bluetoothFetchBuffer(&HM17_1);

                              systickSetTicktime(&BluetoothFetchTimer,
                              BLUETOOTH_FETCH_TIME);
                           }
                        if (isSystickExpired(BluetoothTimer))
                           {
                              initStatus = bluetoothInit(&HM17_1, USART2, 9600, usart2BufferTX);
                              systickSetTicktime(&BluetoothTimer, BLUETOOTH_SETUP_TIME);
                           }
                     }

                  mode = MAIN_LOOP; // Transition to main loop
                  showMenuPage(&menuManager_1, (MenuPosition_t) 0);
                  break;

                  // Main application loop
               case MAIN_LOOP:
                  if (timerTrigger == true)
                     {
                        setRotaryColor(LED_RED);
                        systickUpdateTimerList((uint32_t*) timerList, arraySize);
                        runtime ++;
                        setRotaryColor(LED_BLACK);
                     }
                  if (isSystickExpired(BluetoothFetchTimer))
                     {
                        bluetoothFetchBuffer(&HM17_1);

                        systickSetTicktime(&BluetoothFetchTimer,
                        BLUETOOTH_FETCH_TIME);
                     }
                  if (isSystickExpired(BluetoothLogTimer))
                     {
                        rotaryPosition = getRotaryPosition();
                     //   bluetoothCreateLog(&HM17_1);
                       // systickSetTicktime(&BluetoothLogTimer, 1000);
                        switch (HM17_1.mode)
                           {
                           case bluetoothConfigure:
                              systickSetTicktime(&BluetoothLogTimer, 5000);
                              break;

                           case bluetoothTransmit:
                              bluetoothCreateLog(&HM17_1);
                              systickSetTicktime(&BluetoothLogTimer, BLUETOOTH_TRANSMIT_TIME);

                              break;

                           default:
                              systickSetTicktime(&BluetoothLogTimer, 5000);
                              break;

                           }
                     }




                  if (isSystickExpired(Button))
                     {
                        systickSetTicktime(&Button, 200);
                        switch (menuManager_1.activeMode)
                           {
                           case Page:

                              handleMenu(&menuManager_1, lastRotaryPosition);
                              break;
                           case Entry:
                              if (menuManager_1.activeEntry == &textEntry)
                                 {
                                    tftPrint((char*) "sdofihv", 50, 50, 0);
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              else if (menuManager_1.activeEntry == &counterEntry)
                                 {
                                    tftPrintInt(getRotaryPosition(), 50, 50, 0);
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              else if (menuManager_1.activeEntry == &getStatusEntry)
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

                                       }
                                    if (status == BluetoothFinish && reply == true)
                                       {

                                          tftPrintColor((char*) "OK", 0, 60, tft_GREEN);

                                          active = false;
                                       }
                                    else if (status > 0)
                                       {
                                          tftPrintColor((char*) "Error:", 0, 60, tft_RED);
                                          tftPrintInt(status, 0, 70, 0);
                                          active = false;
                                       }

                                 }
                              else if (menuManager_1.activeEntry == &setBaudRateEntry)
                                 {
                                    static uint8_t step = 0;
                                    static uint8_t fromBaud = 0;
                                    static uint8_t toBaud = 0;

                                    switch (step)
                                       {
                                       case 0:
                                          tftPrint((char*) "From:", 0, 50, 0);
                                          tftPrint((char*) "To:", tftGetWidth() / 2, 50, 0);
                                          step++;
                                          break;
                                       case 1:
                                          if (lastRotaryPosition != getRotaryPosition())
                                             {
                                                lastRotaryPosition = getRotaryPosition();
                                                fromBaud = (uint8_t) getRotaryPosition() % 9;
                                                tftPrintInt(
                                                      bluetoothBaudToInt(fromBaud),
                                                      0, 65, 0);
                                             }
                                          break;

                                       case 2:
                                          if (lastRotaryPosition != getRotaryPosition())
                                             {
                                                lastRotaryPosition = getRotaryPosition();
                                                toBaud = (uint8_t) getRotaryPosition() % 9;
                                                tftPrintInt(
                                                      bluetoothBaudToInt(getRotaryPosition() % 9),
                                                      tftGetWidth() / 2, 65, 0);
                                             }
                                          break;

                                       case 3:
                                          int16_t reply = bluetoothSetBaudRate(&HM17_1, fromBaud, toBaud);

                                          if (reply == 0){
                                                tftPrint("Done", 0, 70, 0);
                                                step++;
                                          }
                                          break;
                                       case 4:
                                          break;


                                       default:
                                          step = 0;
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }

                                    if (getRotaryPushButton() == true){
                                          step++;
                                    }

                                    //status = bluetoothSetBaudRate(&HM17_1, bluetoothBaud_9600, status);
                                    //tftPrintInt(status, 0, 70, 0);

                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);
                                       }
                                 }
                              else if (menuManager_1.activeEntry == &sendTestStringEntry)
                                 {
                                    static bool active = false;
                                    if (getRotaryPushButton() == true)
                                       {
                                          if (active == false)
                                             {
                                                dmacUsartSendString(&HM17_1,
                                                      "Das ist ein sehr langer Text der hier verschickt wird, um zu überprüfen ob das Asyncrone senden funktioniert");
                                                active = true;
                                                tftPrint((char*) "Sending test string", 0, 50, 0);
                                             }
                                          else
                                             {
                                                active = false;
                                                menuManager_1.activeMode = Page;
                                                showMenuPage(&menuManager_1,
                                                      menuManager_1.currentPosition);
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

/**
 * @file main.c
 * @brief  Demo program for the Bluetooth library.
 *
 * This application showcases the Bluetooth library capabilities through an interactive
 * menu-driven interface. Users can verify module status, configure settings, and test
 * communication with the HM17 Bluetooth module. The interface uses a rotary encoder with
 * integrated RGB LED for input and status indication, and an ST7735 TFT display for output.
 *
 * Key features include:
 * - Real-time module status monitoring
 * - Baud rate configuration
 * - Test message transmission
 * - Module reset functionality
 * - Data logging and wireless transmission
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
 * @brief USART2 receive buffer for Bluetooth communication
 *
 * This flag indicates that the base time interval has elapsed, and
 * all timers need to be updated.
 */
bool timerTrigger;

/**
 * @brief Global UART2 RX buffer
 *
 * Stores incoming data from the HM17 Bluetooth module via USART2.
 * Populated asynchronously by the ::USART2_IRQHandler interrupt service routine.
 * Includes space for null terminator to facilitate string operations.
 */
volatile char usart2BufferRX[USART2_RX_BUFFER_SIZE+1];

/**
 * @brief Global UART2RX buffer Index
 *
 * The index indicates the position where the next character is written
 */
volatile uint16_t usart2BufferIndex = 0;

/**
 * @brief USART2 transmit buffer for outgoing Bluetooth messages
 *
 * Stores data to be transmitted to the HM17 module via DMA-assisted USART2.
 * Buffer size is dynamically calculated based on logging requirements:
 * - Maximum 20 bytes per numeric value + 1 byte delimiter
 * - Multiplied by number of log entries
 * - Plus 2 bytes for newline and null terminator
 *
 * If the calculated size is smaller than ::BLUETOOTH_NUMBER_OF_LOG_ENTRYS, this size is used instead.
 *
 */
#if ((20 + 1) * BLUETOOTH_NUMBER_OF_LOG_ENTRYS + 2) > USART2_MIN_TX_BUFFER_SIZE
volatile char usart2BufferTX[ (20 + 1) * BLUETOOTH_NUMBER_OF_LOG_ENTRYS + 2];
#else
volatile char usart2BufferTX[USART2_MIN_TX_BUFFER_SIZE];
#endif


/**
 * @brief USART2 Transfer finish
 *
 * Indicates that transfer over USART2 with DMA is finishd
 *
 * @note Interrupt doesn't work, so his does nothing
 */
volatile bool usart2TXComplete;

/**
 * @brief ST7735 Timer
 *
 * Timer used for the ST7735 display. The timer needs to be global.
 */
uint32_t ST7735_Timer = 0UL;

/**************************************** Menu pages and entrys ****************************************/

/**
@brief Menu system manager
 *
 * Central controller for the menu hierarchy, tracking the current state
 * including active page, selected entry, cursor position, and navigation mode.
 * Coordinates all user interactions with the menu interface.
 */
MenuManager_t menuManager_1;

/**
 * @brief The main menu
 */
MenuPage_t mainMenu;

/**
 *  @brief Features Menu Page
 *
 *  Menu Page showing features such as showing static text, updating a number or calling a function
 */
MenuPage_t featuresPage;

/**
 * @brief Menu page demonstraiting the nested capabilities
 */
MenuPage_t nestedPage;

/**
 * @brief Stuff dedicated to test bluetooth
 */
MenuPage_t bluetoothPage;

/**
 * @brief Stuff for configuring the module
 */
MenuPage_t bluetoothConfigPage;

/**
 * @brief Navigation entry for menu hierarchical traversal
 *
 * Standard "BACK" option enabling users to return to parent menus.
 * Must be included in all submenu pages to maintain proper navigation flow.
 */
MenuEntry_t feldBack =
   {
   .color = tft_WHITE, .title = "BACK", .type = Back, .page = NULL
   };

/**
 * @brief Entry for ::featuresPage
 */
MenuEntry_t featuresEntry =
   {
   .color = tft_WHITE, .title = "Features", .type = Page, .page = &featuresPage
   };

/**
 * @brief Entry for ::nestedPage
 */
MenuEntry_t nestedEntry =
   {
   .color = tft_WHITE, .title = "Nested", .type = Page, .page = &nestedPage
   };

/**
 * @brief Entry for ::bluetoothPage
 */
MenuEntry_t bluetoothEntry =
   {
   .color = tft_WHITE, .title = "Bluetooth", .type = Page, .page = &bluetoothPage
   };

/**
 * @brief Entry for ::bluetoothConfigPage
 */
MenuEntry_t bluetoothConfigEntry =
   {
   .color = tft_WHITE, .title = "BL Config", .type = Page, .page = &bluetoothConfigPage
   };

/**
 * @brief Page showing static text
 */
MenuEntry_t textEntry =
   {
   .color = tft_BLUE, .title = "Text", .type = Entry, .page = NULL
   };

/**
 * @brief Page showing the rotButton position
 */
MenuEntry_t counterEntry =
   {
   .color = tft_RED, .title = "Counter", .type = Entry, .page = NULL
   };

/**
 * @brief Check HM17 status
 *
 * This page sends "AT" and checks if the answer is "OK"
 */
MenuEntry_t getStatusEntry =
   {
   .color = tft_GREEN, .title = "GetStatus", .type = Entry, .page = NULL
   };

/**
 * @brief Send a predefined string over UART
 *
 * This is helpfull for debugging the serial interface or check if a connected device can receive something.
 */
MenuEntry_t sendTestStringEntry =
   {
   .color = tft_GREEN, .title = "SendString", .type = Entry, .page = NULL
   };

/**
 * @brief Set the BAUD rate of HM17
 *
 * For detailed description, see ::bluetoothSetBaudRate
 */
MenuEntry_t setBaudRateEntry =
   {
   .color = tft_GREEN, .title = "Set BAUD", .type = Entry, .page = NULL
   };

/**
 * @brief Reset the HM17
 * @warning Not guaranteed to work
 *
 * For detailed description, see ::bluetoothResetModule
 */
MenuEntry_t resetModulePage =
   {
   .color = tft_RED, .title = "Reset HM17", .type = Entry, .page = NULL
   };

/**************************************** main() ****************************************/
int main(void)
   {

      uint32_t BluetoothTimer = 0UL;      // Commands to HM17 module
      uint32_t BluetoothFetchTimer = 0UL; // Data retrieval from module
      uint32_t BluetoothLogTimer = 0UL;   // Periodic data transmission
      uint32_t MenuTimer = 0UL;              // User interface updates

      uint32_t *timerList[] =
         {
               &BluetoothTimer,
               &BluetoothFetchTimer,
               &MenuTimer,
               &ST7735_Timer,
               &BluetoothLogTimer
         };
      uint8_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

      BluetoothModule_t HM17_1;   // Bluetooth module instance.
      int initStatus = -100;

      /*** Logging ***/
      uint32_t runtime = 0;
      int32_t rotaryPosition = 0;

      strcpy(HM17_1.logEntrys[0].name, "Runtime"); //Assign the name,
      HM17_1.logEntrys[0].type = BluetoothLogEntryType_uint32_t; //datatype
      HM17_1.logEntrys[0].data.uint32_ptr = &runtime; // and pointer to variable

      strcpy(HM17_1.logEntrys[1].name, "Rot-Pos");
      HM17_1.logEntrys[1].type = BluetoothLogEntryType_int32_t;
      HM17_1.logEntrys[1].data.int32_ptr = &rotaryPosition;

      HM17_1.TXComplete = &usart2TXComplete; // Assign the complete sign to the HM17 instance
      *(HM17_1.TXComplete) = true;

      int lastRotaryPosition = 0; //Stores the last Rotary position


      /**************************************** Menu pages ****************************************/
      mainMenu.TL = &featuresEntry;
      mainMenu.TR = &nestedEntry;
      mainMenu.BL = &bluetoothEntry;
      mainMenu.BR = &bluetoothConfigEntry;

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

      bluetoothConfigPage.TL = &feldBack;
      bluetoothConfigPage.BL = &getStatusEntry;
      bluetoothConfigPage.BR = &setBaudRateEntry;
      bluetoothConfigPage.TR = &resetModulePage;

      menuManager_1.activeMode = Page;
      menuManager_1.activePage = &mainMenu;


      //This are variables used by various entry pages
      bool menuActive = false;
      int menuStatus = -127;
      bool menuReply = false;
      uint8_t menuStep = 0;
      uint8_t menuFromBaud = 0;
      uint8_t menuToBaud = 0;

      MAIN_MODE mode = MAIN_INIT;

      for (;;)
         {
            switch (mode)
               {
               default:
                  mode = MAIN_INIT;
                  break;
               // Initialization mode
               case MAIN_INIT:
                  systickInit(SYSTICK_1MS); // Configure SysTick with a 1 ms interval.

                  HM17_1.initStatus = -10; // Set initial status

                  initRotaryPushButton();
                  initRotaryPushButtonLED();
                  systickSetTicktime(&MenuTimer, 20);

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
                              initStatus = bluetoothInit(&HM17_1, USART2, bluetoothBaud_1200,
                                    usart2BufferTX);
                              systickSetTicktime(&BluetoothTimer, BLUETOOTH_SETUP_TIME);
                           }
                     }

                  mode = MAIN_LOOP; // Transition to main loop
                  showMenuPage(&menuManager_1, (MenuPosition_t) 0); //Show the menu page
                  break;

                  // Main application loop
               case MAIN_LOOP:


                  if (timerTrigger == true)
                     {
                        /*
                         * System timer event processing
                         * Updates all active timers and increments runtime counter
                         * for data logging purposes. LED toggle provides visual
                         * feedback of timer operation for diagnostic purposes.
                         */
                        setRotaryColor(LED_RED); // Visual timing indicator (diagnostic)
                        systickUpdateTimerList((uint32_t*) timerList, arraySize);
                        runtime++; // Track total application uptime for logging
                        setRotaryColor(LED_BLACK);// Complete timing indicator cycle
                     }

                  if (isSystickExpired(BluetoothFetchTimer))
                     {
                        /*
                         * Bluetooth communication data acquisition
                         * Retrieves and processes incoming data from the HM17 module
                         * at regular intervals. Parser interprets module responses and
                         * updates application state accordingly.
                         */

                        bluetoothFetchBuffer(&HM17_1);
                        if (HM17_1.available > 0) // Process only when data is available
                           {
                              bluetoothParser(&HM17_1); // Parse data
                           }

                        systickSetTicktime(&BluetoothFetchTimer,
                        BLUETOOTH_FETCH_TIME);
                     }

                  if (isSystickExpired(BluetoothLogTimer))
                     {
                        rotaryPosition = getRotaryPosition(); // Capture current encoder position

                        switch (HM17_1.mode)
                           {
                           // If the module is not connected, it is checked again after 5 seconds
                           case bluetoothConfigure:
                              systickSetTicktime(&BluetoothLogTimer, 5000);
                              break;

                            // If a device is connected, the log is send and the timer is set to the log interval
                           case bluetoothTransmit:
                              if (HM17_1.bluetoothSendLogTitle == true)
                                 {
                                    HM17_1.bluetoothSendLogTitle = false;
                                    bluetoothSendLogTitle(&HM17_1);
                                 }
                              else
                                 {
                                    bluetoothSendLog(&HM17_1);
                                 }
                              systickSetTicktime(&BluetoothLogTimer, BLUETOOTH_TRANSMIT_TIME);

                              break;

                           default:
                              systickSetTicktime(&BluetoothLogTimer, 5000);
                              break;

                           }
                     }

                  if (isSystickExpired(MenuTimer))
                     {
                        systickSetTicktime(&MenuTimer, 200); // Set the standard timer time for the menu
                        switch (menuManager_1.activeMode)
                           {
                           case Page:
                              handleMenu(&menuManager_1);
                              break;
                           case Entry:
                              /*
                               * Here happens the handling of the entry pages.
                               *
                               * Each entry is in its own condition. In there it can do pretty much everything it likes to do.
                               * If a higher (or lower) refresh time is needed, it can be done by setting systickSetTicktime again.
                               *
                               * It even is possible to not provide the possibility to go back to the menu page and stay in this entry.
                               * This can be used e. g to only provide the menu while setting up the balancer.
                               * Once it starts balancing it then shows only a stripped down menu to set the parameters with
                               * as less delay as possible.
                               */
                              if (menuManager_1.activeEntry == &textEntry)
                                 {
                                    tftPrint((char*) "sdofihv", 50, 50, 0); // Just show some static text

                                    // Go back to the menu after a button press
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              /**************************************** Counter ****************************************/
                              else if (menuManager_1.activeEntry == &counterEntry)
                                 {
                                    tftPrintInt(getRotaryPosition(), 50, 50, 0); // Show the rotary position
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                 }
                              /**************************************** getStatus ****************************************/
                              else if (menuManager_1.activeEntry == &getStatusEntry)
                                 {
                                  //  static int status = -127;
                                 //   static bool reply;
                                    //static bool active = false;

                                    if (getRotaryPushButton() == true)
                                       {
                                          if (menuStatus >= 0)// routine is finished, reset all variables
                                             {
                                                menuStatus = -127;
                                                menuActive = false;
                                                menuManager_1.activeMode = Page;
                                                showMenuPage(&menuManager_1,
                                                      menuManager_1.currentPosition);
                                             }
                                          else // Begin new status check operation
                                             {
                                                menuActive = true;
                                                tftPrint((char*) "Getting Status...", 0, 50, 0);
                                             }

                                       }
                                    if (menuActive == true)
                                       {
                                          //Execute Bluetooth status verification
                                          menuStatus = bluetoothGetStatus(&HM17_1, &menuReply);

                                       }
                                    if (menuStatus == BluetoothFinish && menuReply  == true)
                                       {
                                          /*  Process status check results
                                           * Displays appropriate feedback based on command outcome
                                           */
                                          tftPrintColor((char*) "OK", 0, 60, tft_GREEN);

                                          menuActive = false;
                                       }
                                    else if (menuStatus > 0) // Error condition
                                       {
                                          tftPrintColor((char*) "Error:", 0, 60, tft_RED);
                                          tftPrintInt(menuStatus, 0, 70, 0);
                                          menuActive = false;
                                       }

                                 }
                              /**************************************** set Baud rate ****************************************/
                              else if (menuManager_1.activeEntry == &setBaudRateEntry)
                                 {


                                    switch (menuStep)
                                       {
                                       case 0:
                                          /*
                                           *  * Initialize baud rate configuration wizard
                                           * Creates a two-column layout with labels for current
                                           * and target baud rate values for intuitive selection
                                           *
                                           */
                                          tftPrint((char*) "From:", 0, 50, 0);
                                          tftPrint((char*) "To:", tftGetWidth() / 2, 50, 0);
                                          menuStep++;
                                          break;
                                       case 1: // Select current baud rate
                                          if (lastRotaryPosition != getRotaryPosition())
                                             {

                                                lastRotaryPosition = getRotaryPosition();
                                                menuFromBaud = (uint8_t) getRotaryPosition() % 9;
                                                tftPrintInt((int)bluetoothBaudToInt(menuFromBaud), 0, 65, 0);
                                             }
                                          break;

                                       case 2:  // Select target baud rate
                                          if (lastRotaryPosition != getRotaryPosition())
                                             {
                                                lastRotaryPosition = getRotaryPosition();
                                                menuToBaud = (uint8_t) getRotaryPosition() % 9;
                                                tftPrintInt( (int)
                                                      bluetoothBaudToInt(getRotaryPosition() % 9),
                                                      tftGetWidth() / 2, 65, 0);
                                             }
                                          break;

                                       case 3: // Execute baud rate change
                                           menuStatus = bluetoothSetBaudRate(&HM17_1, menuFromBaud,
                                                menuToBaud);

                                          if (menuStatus == 0)// Successful configuration
                                             {
                                                tftPrint("Done", 0, 70, 0);
                                                menuStatus=-127;
                                                menuStep++;// Proceed to confirmation step
                                             }
                                          break;
                                       case 4: // Wait for user acknowledgment
                                          // User must confirm by pressing button to return to menu
                                          break;

                                       default:// Return to menu system
                                          menuStep = 0;
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);

                                       }
                                    // Advances through configuration steps on button press, except during the actual rate change operation (step 3)
                                    if (getRotaryPushButton() == true && menuStep != 3)
                                       {
                                          menuStep++;
                                       }

                                    //status = bluetoothSetBaudRate(&HM17_1, bluetoothBaud_9600, status);
                                    //tftPrintInt(status, 0, 70, 0);

                                  /*  if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);
                                       }*/
                                 }
                              /**************************************** send test string ****************************************/
                              else if (menuManager_1.activeEntry == &sendTestStringEntry)
                                 {
                                  //  static bool active = false;
                                    if (getRotaryPushButton() == true)
                                       {
                                          if (menuActive == false) //Send test pattern
                                             {
                                                dmacUsartSendString(&HM17_1,
                                                      "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789");
                                                menuActive = true;
                                                tftPrint((char*) "Sending test string", 0, 50, 0);
                                             }
                                          else // Second button press returns to menu
                                             {
                                                menuStatus = false;
                                                menuManager_1.activeMode = Page;
                                                showMenuPage(&menuManager_1,
                                                      menuManager_1.currentPosition);
                                             }
                                       }

                                 }
                              /**************************************** reset module ****************************************/
                              else if (menuManager_1.activeEntry == &resetModulePage)
                                 {
                                    //static int status = -127;
                                 //   static bool reply;
                                  //  static bool active = false;

                                    if (getRotaryPushButton() == true)
                                       {
                                          if (menuStatus >= 0)// Reset operation completed
                                             {
                                                menuStatus = -127;
                                                menuActive = false;
                                                menuManager_1.activeMode = Page;
                                                showMenuPage(&menuManager_1,
                                                      menuManager_1.currentPosition);
                                             }
                                          else                       // Initiate new reset operation
                                             {
                                                menuActive = true;
                                             }

                                       }
                                    if (menuActive == true)
                                       {
                                          //Execute module reset sequence
                                          menuStatus = bluetoothResetModule(&HM17_1);
                                          systickSetTicktime(&MenuTimer, 1100);

                                          if (menuStatus == BluetoothFinish)
                                             {

                                                tftPrintColor((char*) "Done", 0, 60, tft_GREEN);

                                                menuActive = false;
                                             }
                                          else if (menuStatus > 0)
                                             {
                                                tftPrintColor((char*) "Error:", 0, 60, tft_RED);
                                                tftPrintInt(menuStatus, 0, 70, 0);
                                                menuActive = false;
                                             }

                                       }

                                 }
                              else // If the entry page hasn't been specified, go back to the menu after a button press
                                 {
                                    if (getRotaryPushButton() == true)
                                       {
                                          menuManager_1.activeMode = Page;
                                          showMenuPage(&menuManager_1,
                                                menuManager_1.currentPosition);
                                       }
                                 }
                              break;
                           case Back:
                           default:
                              break; //This should never happen
                           }

                     }

               }


         }
   }

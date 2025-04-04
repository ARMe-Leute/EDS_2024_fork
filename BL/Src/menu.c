/*
 * menu.c
 *
 *  Created on: Feb 11, 2025
 *      Author: c0deberry
 */

#include <menu.h>

/**
 * @brief Draw a 4x4 grid
 *
 * Draw a 4 by 4 grid on the display to segment the  menu entrys
 */
void drawGrid()
   {
      tftDrawRect(0, 0, tftGetWidth() - 1, tftGetHeight() - 1, tft_WHITE);
      tftDrawRect(tftGetWidth() / 2, 0, tftGetWidth() / 2, tftGetHeight(), tft_WHITE);
      tftDrawRect(0, tftGetHeight() / 2, tftGetWidth(), tftGetHeight() / 2, tft_WHITE);
   }

/**
 * @brief Highlight a menu entry
 *
 * @param position The field to highlight
 *
 * Draws a yellow rectangle to highlight a grid element.
 */
void higlightEntry(MenuPosition_t position)
   {
      switch (position)
         {
         case MenuTL:
            tftDrawRect(0, 0, tftGetWidth() / 2, tftGetHeight() / 2, tft_YELLOW);
            break;
         case MenuTR:
            tftDrawRect(tftGetWidth() / 2, 0, tftGetWidth() - 1, tftGetHeight() / 2, tft_YELLOW);
            break;
         case MenuBL:
            tftDrawRect(0, tftGetHeight() / 2, tftGetWidth() / 2, tftGetHeight() - 1, tft_YELLOW);
            break;
         case MenuBR:
            tftDrawRect(tftGetWidth() / 2, tftGetHeight() / 2, tftGetWidth() - 1,
                  tftGetHeight() - 1, tft_YELLOW);
            break;
         default:
            break;

         }

   }

/**
 * @brief Get pointer to entry based on position
 * @param manager Instance of ::MenuManager_t
 * @param position The position from where to get the pointer
 * @return Pointer to ::MenuEntry_t
 *
 * Get the pointer to a menu entry based on a position
 */
MenuEntry_t* getEntryFromPosition(MenuManager_t *manager, MenuPosition_t position)
   {

      switch (position)
         {
         case MenuTL:
            return manager->activePage->TL;
         case MenuTR:
            return manager->activePage->TR;
         case MenuBL:
            return manager->activePage->BL;
         case MenuBR:
            return manager->activePage->BR;
         default:
            return NULL;

         }
   }

/**
 * @brief Convert rotary position to menu position
 * @param rotaryCounter The rotary position to convert
 * @return menu position
 *
 * This functio converts an int coming from the rotary encoder to a menu position. Zero-crossing is not a problem.
 */
MenuPosition_t getMenuRotaryPosition(int rotaryCounter)
   {
      return ((uint16_t) rotaryCounter) % 4;
   }

/**
 * @brief Show the menu page
 * @param manager Instance of ::MenuManager_t
 * @param position entry to highlight
 *
 * Depending on the active mode either the entry page is shown or the menu page is shown.
 * The title is printed in the specified color.
 */
void showMenuPage(MenuManager_t *manager, MenuPosition_t position)
   {
      tftFillScreen(tft_BLACK);
      switch (manager->activeMode)
         {
         case Entry:
            tftPrintColor(manager->activeEntry->title, TL_OFFSET_X, TL_OFFSET_Y,
                  manager->activeEntry->color);
            break;

         case Page:
            tftPrintColor(manager->activePage->TL->title, TL_OFFSET_X, TL_OFFSET_Y,
                  manager->activePage->TL->color);
            tftPrintColor(manager->activePage->TR->title, TR_OFFSET_X, TR_OFFSET_Y,
                  manager->activePage->TR->color);
            tftPrintColor(manager->activePage->BL->title, BL_OFFSET_X, BL_OFFSET_Y,
                  manager->activePage->BL->color);
            tftPrintColor(manager->activePage->BR->title, BR_OFFSET_X, BR_OFFSET_Y,
                  manager->activePage->BR->color);
            drawGrid();
            higlightEntry(position);
            break;
         default:
            break;

         }

   }

/**
 * @brief Do menu stuff
 * @param menuManager Instance of ::MenuManager_t
 *
 * Handles the menu control. If the rotary encoder is turned, the grid ist updated to highlight the new selected entry.
 * If the button is pressed, the selected submenu or entry page is opend.
 */
void handleMenu(MenuManager_t *menuManager)
   {
         // check if button wass turned since last check
      if (getMenuRotaryPosition(getRotaryPosition()) != menuManager->currentPosition)
         {
            menuManager->currentPosition = getMenuRotaryPosition(getRotaryPosition());
            drawGrid();
            higlightEntry(menuManager->currentPosition);
         }
      if (getRotaryPushButton() == true)
         {
               //The selected entry is a menu page
            if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type == Page)
               {
                  getEntryFromPosition(menuManager, menuManager->currentPosition)->page->lastMenu =
                        menuManager->activePage; // set the last menu to be able to go back
                  menuManager->activePage = getEntryFromPosition(menuManager,
                        menuManager->currentPosition)->page; // Set the new page as active
               }
            //The selected entry is an entry
            else if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type == Entry)
               {
                  menuManager->activeEntry = getEntryFromPosition(menuManager,
                        menuManager->currentPosition); // Set the selected entry as active
                  menuManager->activeMode = Entry; // Change mode to entry
               }
            // The selected menu is the back button
            else
               {
                  menuManager->activePage = menuManager->activePage->lastMenu;
               }
            showMenuPage(menuManager, menuManager->currentPosition); //Render the result
         }
   }

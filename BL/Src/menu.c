/*
 * menu.c
 *
 * Copy this file to the `Src` folder of your project.
 *
 * @author c0deberry
 * @author nrs00
 *
 *  Created on: 11.02.2025
 */

#include <menu.h>

/**
 * @brief Draws a 4x4 grid on the display.
 *
 * This function renders a 4x4 grid by dividing the screen into four quadrants.
 * The grid is used to visually segment menu entries on the display.
 */
void drawGrid()
   {
      // Draw the outer boundary of the display
      tftDrawRect(0, 0, tftGetWidth() - 1, tftGetHeight() - 1, tft_WHITE);

      // Draw a vertical line to split the screen into left and right halves
      tftDrawRect(tftGetWidth() / 2, 0, tftGetWidth() / 2, tftGetHeight(), tft_WHITE);

      // Draw a horizontal line to split the screen into top and bottom halves
      tftDrawRect(0, tftGetHeight() / 2, tftGetWidth(), tftGetHeight() / 2, tft_WHITE);
   }

/**
 * @brief Highlights a specific menu entry on the grid.
 *
 * @param position The position of the menu entry to highlight (top-left, top-right,
 * bottom-left, or bottom-right).
 *
 * This function visually highlights a menu entry by drawing a yellow rectangle
 * over the corresponding quadrant of the grid.
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
 * @brief Retrieves a pointer to a menu entry based on its position.
 *
 * @param manager Pointer to an instance of ::MenuManager_t that manages the menu state.
 * @param position The position of the desired menu entry (top-left, top-right,
 * bottom-left, or bottom-right).
 * @return Pointer to ::MenuEntry_t representing the menu entry at the specified position,
 * or NULL if the position is invalid.
 *
 * This function accesses the active page managed by `MenuManager_t` and returns a pointer
 * to a specific menu entry based on its position within the grid.
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
            // Return NULL for invalid positions
            return NULL;

         }
   }

/**
 * @brief Converts a rotary encoder position to a menu position.
 *
 * @param rotaryCounter The current position of the rotary encoder.
 * @return The corresponding menu position as a value of ::MenuPosition_t.
 *
 * This function maps an integer value from the rotary encoder to one of the four
 * menu positions (top-left, top-right, bottom-left, bottom-right). The typecasting
 * ensures that the result wraps around correctly, even if the rotary counter crosses zero.
 */
MenuPosition_t getMenuRotaryPosition(int rotaryCounter)
   {
      return ((uint16_t) rotaryCounter) % 4;
   }

/**
 * @brief Displays the menu page and highlights a specific entry.
 *
 * @param manager Pointer to an instance of ::MenuManager_t managing the menu state.
 * @param position The menu entry to highlight (e.g., top-left, top-right).
 *
 * This function clears the screen and displays the appropriate menu page based on
 * the active mode. If in `Entry` mode, it displays its title. If in
 * `Page` mode, it displays all entries on the current page along with their titles
 * and colors. The specified entry is highlighted for user interaction.
 */
void showMenuPage(MenuManager_t *manager, MenuPosition_t position)
   {
      // Clear the screen with a black background
      tftFillScreen(tft_BLACK);
      switch (manager->activeMode)
         {
         case Entry:
            // Display the title of the active entry in its designated color
            tftPrintColor(manager->activeEntry->title, TL_OFFSET_X, TL_OFFSET_Y,
                  manager->activeEntry->color);
            break;

         case Page:
            // Display titles and colors for all entries on the active page
            tftPrintColor(manager->activePage->TL->title, TL_OFFSET_X, TL_OFFSET_Y,
                  manager->activePage->TL->color);
            tftPrintColor(manager->activePage->TR->title, TR_OFFSET_X, TR_OFFSET_Y,
                  manager->activePage->TR->color);
            tftPrintColor(manager->activePage->BL->title, BL_OFFSET_X, BL_OFFSET_Y,
                  manager->activePage->BL->color);
            tftPrintColor(manager->activePage->BR->title, BR_OFFSET_X, BR_OFFSET_Y,
                  manager->activePage->BR->color);

            // Draw grid lines and highlight the selected entry
            drawGrid();
            higlightEntry(position);
            break;

         case Back:
         default:
            // No action for invalid modes
            break;

         }

   }

/**
 * @brief Handles user interaction with the menu system.
 *
 * @param menuManager Pointer to an instance of ::MenuManager_t managing the menu state.
 *
 * This function processes input from a rotary encoder and a push button to control
 * navigation within the menu. It updates the highlighted grid element when the rotary
 * encoder is turned and changes pages or modes when the button is pressed. The following
 * actions are supported:
 *
 * - **Rotary Encoder Turned:** Updates the highlighted grid element to reflect the new selection.
 * - **Button Pressed:** Opens a submenu or entry page based on the selected item. If "Back" is selected,
 *   it navigates to the previous page.
 */
void handleMenu(MenuManager_t *menuManager)
   {
      // Check if rotary encoder has moved since last check
      if (getMenuRotaryPosition(getRotaryPosition()) != menuManager->currentPosition)
         {
            // Update current position and redraw grid with new highlight
            menuManager->currentPosition = getMenuRotaryPosition(getRotaryPosition());
            drawGrid();
            higlightEntry(menuManager->currentPosition);
         }

      // Check if push button has been pressed
      if (getRotaryPushButton() == true)
         {
            // Retrieve selected entry based on current position and handle different types of entries
            if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type == Page)
               {
                  // Navigate to submenu: set last menu for return navigation and update active page
                  getEntryFromPosition(menuManager, menuManager->currentPosition)->page->lastMenu =
                        menuManager->activePage;
                  menuManager->activePage = getEntryFromPosition(menuManager,
                        menuManager->currentPosition)->page; // Set the new page as active
               }
            // Open entry: set as active entry and switch mode to Entry
            else if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type == Entry)
               {
                  menuManager->activeEntry = getEntryFromPosition(menuManager,
                        menuManager->currentPosition); // Set the selected entry as active
                  menuManager->activeMode = Entry; // Change mode to entry
               }

            else
               {
                  // Navigate back to previous page if "Back" is selected
                  menuManager->activePage = menuManager->activePage->lastMenu;
               }
            // Render updated menu state after action
            showMenuPage(menuManager, menuManager->currentPosition); //Render the result
         }
   }

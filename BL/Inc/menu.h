/**
 * @file menu.h
 * @brief Header file for creating and managing menu layouts on a graphical display.
 *
 * This library provides functionality for defining menus
 * on a screen using the ST7735 display driver and RotaryPushButton input handling.
 *
 * Copy this file to the `Inc` folder of your project.
 *
 * @author c0deberry
 * @author nrs00
 *
 *
 *  Created on: 11.02.2025
 */

#ifndef MENU_H_
#define MENU_H_

#include <string.h>

#include <ST7735.h>
#include <RotaryPushButton.h>

/**
 * @brief X-axis offset for the top-left corner of the menu grid (::MenuTL).
 *
 * This offset adds 1 pixel to account for the grid alignment, ensuring proper
 * positioning of menu elements within the defined layout.
 */
#define TL_OFFSET_X (0+1)

/**
 * @brief Y-axis offset for the top-left corner of the menu grid (::MenuTL).
 *
 * No additional offset is applied here.
 */
#define TL_OFFSET_Y (0)


/**
 * @brief X-axis offset for the top-right corner of the menu grid (::MenuTR).
 *
 * The offset is calculated as half of the screen width plus 1 pixel to account
 * for grid alignment. This ensures that elements in the top-right quadrant
 * are properly positioned.
 */
#define TR_OFFSET_X ((tftGetWidth() /2)+1)

/**
 * @brief Y-axis offset for the top-right corner of the menu grid (::MenuTR).
 *
 * No additional offset is applied here.
 */
#define TR_OFFSET_Y (0)


/**
 * @brief X-axis offset for the bottom-left corner of the menu grid (::MenuBL).
 *
 * Similar to ::TL_OFFSET_X, this offset adds 1 pixel to account for grid alignment
 * in the bottom-left quadrant of the screen.
 */
#define BL_OFFSET_X (0+1)

/**
 * @brief Y-axis offset for the bottom-left corner of the menu grid (::MenuBL).
 *
 * The offset is calculated as half of the screen height, positioning elements
 * in the lower half of the display.
 */
#define BL_OFFSET_Y (tftGetHeight() /2)


/**
 * @brief X-axis offset for the bottom-right corner of the menu grid (::MenuBR).
 *
 * The offset is calculated as half of the screen width plus 1 pixel to account
 * for grid alignment in the bottom-right quadrant of the screen.
 */
#define BR_OFFSET_X ((tftGetWidth() /2)+1)

/**
 * @brief Y-axis offset for the bottom-right corner of the menu grid (::MenuBR).
 *
 * The offset is calculated as half of the screen height, positioning elements
 * in the lower half of the display.
 */
#define BR_OFFSET_Y (tftGetHeight() /2)

/**
 * @brief Enumeration of available menu positions.
 *
 * This typedef defines the possible positions for menu entries on the screen.
 * Positions are defined in landscape mode.
 */
typedef enum MenuPosition
   {
   MenuTL = 0,  /**< Top Left*/
   MenuTR,      /**< Top Right*/
   MenuBR,      /**< Bottom Left*/
   MenuBL       /**< Bottom Right*/
   } MenuPosition_t;

/**
 * @brief Enumeration of menu entry types.
 *
 * This type is used to classify a ::MenuEntry_t as either a menu page (::Page),
 * an entry within a menu page (::Entry), or a special back entry (::Back) that
 * navigates to the previous menu layer. The ::Back type simplifies the code.
 *
 * - **Page**: Represents a menu page containing multiple entries. It serves as an overview
 *   of available options, which can lead to submenus or value-setting pages.
 * - **Entry**: Represents a specific page where users can modify settings or values.
 * - **Back**: A special entry that allows navigation to the parent menu page.
 */
typedef enum MenuType
   {
   Page,        /**< Entry represents a menu page */
   Entry,       /**< Entry represents a menu item */
   Back         /**< Entry navigates to the previous menu layer */
   } MenuType_t;


typedef struct MenuPage MenuPage_t;

/**
 * @brief Structure representing a single menu entry.
 *
 * A menu entry can either represent a submenu (::Page) or an actionable item (::Entry).
 * If the type is ::Page, the `page` pointer stores the associated submenu. Only one global
 * instance of type ::Back is required for navigation purposes (if used at all).
 */
typedef struct MenuEntry
   {
      uint16_t color; /**< Display color for the entry title */
      char *title; /**< Title text of the menu entry */
      const MenuType_t type; /**< Type of the menu entry (e.g., Page, Entry, Back) */
      MenuPage_t *page; /**< Pointer to the submenu if the type is ::Page */

   } MenuEntry_t;

/**
 * @brief Structure representing a menu page containing multiple entries.
 *
 * This struct stores pointers to all entries within a specific menu page. Each
 * position (Top Left, Top Right, Bottom Left, Bottom Right) must be filled with
 * valid entries; otherwise, the program may hang when attempting to access an empty field.
 *
 * @warning Ensure all positions are initialized to avoid runtime errors.
 */
typedef struct MenuPage
   {
      MenuEntry_t *TL; /**< Entry at Top Left position */
      MenuEntry_t *TR; /**< Entry at Top Right position */
      MenuEntry_t *BL; /**< Entry at Bottom Left position */
      MenuEntry_t *BR; /**< Entry at Bottom Right position */
      MenuPage_t *lastMenu; /**< Pointer to the parent menu page (used by Back navigation) */

   } MenuPage_t;

/**
 * @brief Structure managing active menu state and navigation.
 *
 * This struct keeps track of important information such as the currently active
 * menu page and entry. It also determines whether the user is interacting with
 * a menu page or an entry page via ::activeMode. Upon exiting an active entry page,
 * navigation returns to ::activePage.
 */
typedef struct MenuManager
   {
      MenuPage_t *activePage; /**< Pointer to the currently active menu page */
      MenuEntry_t *activeEntry; /**< Pointer to the currently active entry page */
      MenuType_t activeMode; /**< Type indicating whether a page or entry is active */
      MenuPosition_t currentPosition; /**< Current cursor position within the menu */
   } MenuManager_t;

extern void showMenuPage(MenuManager_t *manager, MenuPosition_t position);
extern void higlightEntry(MenuPosition_t position);
extern void drawGrid();
extern void higlightEntry(MenuPosition_t position);
extern MenuEntry_t* getEntryFromPosition(MenuManager_t *manager, MenuPosition_t position);
extern MenuPosition_t getMenuRotaryPosition(int rotaryCounter);
extern void handleMenu(MenuManager_t *menuManager);

#endif /* MENU_H_ */

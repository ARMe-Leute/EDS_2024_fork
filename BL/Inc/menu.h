/**
 * @file menu.h
 * @brief Library for creating menus
 *

 *
 * Copy this file to the Inc folder of your project.
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
 * @brief X Offset for the ::MenuTL
 *
 * +1 to account for the grid
 */
#define TL_OFFSET_X (0+1)

/**
 * @brief Y Offset for the ::MenuTL
 */
#define TL_OFFSET_Y (0)


/**
 * @brief X Offset for the ::MenuTR
 *
 * Half screed width +1 to account for the grid
 */
#define TR_OFFSET_X ((tftGetWidth() /2)+1)

/**
 * @brief Y Offset for the ::MenuTR
 */
#define TR_OFFSET_Y (0)


/**
 * @brief X Offset for the ::MenuBL
 *
 * +1 to account for the grid
 */
#define BL_OFFSET_X (0+1)

/**
 * @brief Y Offset for ::MenuBL
 */
#define BL_OFFSET_Y (tftGetHeight() /2)


/**
 * @brief X Offset for the ::MenuBR
 *
 * Half screed width +1 to account for the grid
 */
#define BR_OFFSET_X ((tftGetWidth() /2)+1)

/**
 * @brief Y Offset for ::MenuBR
 */
#define BR_OFFSET_Y (tftGetHeight() /2)

/**
 * @brief Available Menu positions
 *
 * This typedef defines the available menu positions. The position are defined in landscape mode.
 */
typedef enum MenuPosition
   {
   MenuTL = 0,  /**< Top Left*/
   MenuTR,      /**< Top Right*/
   MenuBR,      /**< Bottom Left*/
   MenuBL       /**< Bottom Right*/
   } MenuPosition_t;

/**
 * @brief Type of menu entry
 *
 * This type is used to determine wether a ::MenuEntry_t is menu page (::MenuPage_t) or an entry in a menu page (::MenuEntry_t).
 * ::Back is an menu entry that brings you to a menu page one layer above. It is a seperate type for code simplicity.
 *
 *  - A ::Page contains different entrys and is an overview over the options you have. An entry in that page can either be a entry page
 * (e. g. where you set a value) or a submenu where more settings are available.
 *  - An ::Entry opens a page where you e. g. modify a value.
 */
typedef enum MenuType
   {
   Page,        /**< Entry is a menu page**/
   Entry,       /**< Entry is a menu entry**/
   Back         /**< Entry brings you one layer abov**/
   } MenuType_t;


typedef struct MenuPage MenuPage_t;

/**
 * @brief Menu entry listed in a menu page
 *
 * A menu entry can either be a Menu page (::Page) or a menu entry (::Entry), only one global instance of type ::Back is needed (If at all)
 */
typedef struct MenuEntry
   {
      uint16_t color; /**< The color in which the title is displayd*/
      char *title; /**< The title of the entry*/
      const MenuType_t type; /**< The type of the entry*/
      MenuPage_t *page; /**< If the type is ::Page, a pointer to the page is stored here*/

   } MenuEntry_t;

/**
 * @brief Menu page containging the Entrys
 *
 * This struct stores a pointer to all entrys.
 *
 * @warning Each position must be filled, otherwise the the programm hangs once you try to open an emty field.
 */
typedef struct MenuPage
   {
      MenuEntry_t *TL; /**< Menu item Top Left*/
      MenuEntry_t *TR; /**< Menu item Top Right*/
      MenuEntry_t *BL; /**< Menu item Bottom Left*/
      MenuEntry_t *BR; /**< Menu item Bottom Right*/
      MenuPage_t *lastMenu; /**< The last menu page (one layer above), used by the back field*/

   } MenuPage_t;

/**
 * @brief Stores important information
 *
 * Here are pointers to thee current active menu page and  active entry are stored. Whether we currently are in a menu page or an entry page is
 * determined by ::activeMode. On leaving the active Entry page we go back to ::activePage.
 *
 */
typedef struct MenuManager
   {
      MenuPage_t *activePage; /**< The menu age that is active currently*/
      MenuEntry_t *activeEntry; /**< The entry page that is active currently*/
      MenuType_t activeMode; /**< The type which is currently open*/
      MenuPosition_t currentPosition; /**< The current position of the cursor*/
   } MenuManager_t;

extern void showMenuPage(MenuManager_t *manager, MenuPosition_t position);
extern void higlightEntry(MenuPosition_t position);
extern void drawGrid();
extern void higlightEntry(MenuPosition_t position);
extern MenuEntry_t* getEntryFromPosition(MenuManager_t *manager, MenuPosition_t position);
extern MenuPosition_t getMenuRotaryPosition(int rotaryCounter);
extern void handleMenu(MenuManager_t *menuManager);

#endif /* MENU_H_ */

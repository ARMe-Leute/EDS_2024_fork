/*
 * menu.h
 *
 *  Created on: Feb 11, 2025
 *      Author: c0deberry
 */

#ifndef MENU_H_
#define MENU_H_

#include <string.h>

#include <ST7735.h>

#define TL_OFFSET_X (0+1)
#define TL_OFFSET_Y (0)

#define TR_OFFSET_X ((tftGetWidth() /2)+1)
#define TR_OFFSET_Y (0)

#define BL_OFFSET_X (0+1)
#define BL_OFFSET_Y (tftGetHeight() /2)

#define BR_OFFSET_X ((tftGetWidth() /2)+1)
#define BR_OFFSET_Y (tftGetHeight() /2)




typedef enum MenuPosition{
    MenuTL =0,
    MenuTR,
    MenuBR,
    MenuBL
}MenuPosition_t;

typedef enum MenuType{
    Page,
    Entry,
    Back
}MenuType_t;
typedef struct MenuPage MenuPage_t;

typedef struct MenuEntry{
    uint16_t color;
    char *title;
    const MenuType_t type;
    MenuPage_t *page;


   // void (*onMenuPress) (MenuEntry_t);

}MenuEntry_t;

typedef struct MenuPage{
    MenuEntry_t *TL;
    MenuEntry_t *TR;
    MenuEntry_t *BL;
    MenuEntry_t *BR;
    MenuPage_t *lastMenu;

}MenuPage_t;






typedef struct MenuManager{
    MenuPage_t* activePage;
    MenuEntry_t* activeEntry;
    MenuType_t activeMode;
    MenuPosition_t currentPosition;
}MenuManager_t;

void showMenuPage(MenuManager_t * manager, MenuPosition_t position);
void higlightEntry(MenuPosition_t position);
void drawGrid();
void higlightEntry(MenuPosition_t position);
MenuEntry_t * getEntryFromPosition(MenuManager_t* manager, MenuPosition_t position);
uint8_t getMenuRotaryPosition(int rotaryCounter);

#endif /* MENU_H_ */

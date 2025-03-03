/*
 * menu.c
 *
 *  Created on: Feb 11, 2025
 *      Author: c0deberry
 */

#include <menu.h>

void drawGrid(){
    tftDrawRect(0, 0, tftGetWidth()-1, tftGetHeight()-1, tft_WHITE);
    tftDrawRect(tftGetWidth()/2, 0, tftGetWidth()/2, tftGetHeight(), tft_WHITE);
    tftDrawRect(0, tftGetHeight()/2, tftGetWidth(), tftGetHeight()/2, tft_WHITE);
}

void higlightEntry(MenuPosition_t position){
    switch (position){
    case MenuTL:
    	tftDrawRect(0, 0, tftGetWidth()/2, tftGetHeight()/2, tft_YELLOW);
    	break;
    case MenuTR:
    	tftDrawRect(tftGetWidth()/2, 0, tftGetWidth()-1, tftGetHeight()/2, tft_YELLOW);
    	break;
    case MenuBL:
    	tftDrawRect(0, tftGetHeight()/2, tftGetWidth()/2, tftGetHeight()-1, tft_YELLOW);
    	break;
    case MenuBR:
    	tftDrawRect(tftGetWidth()/2, tftGetHeight()/2, tftGetWidth()-1, tftGetHeight()-1, tft_YELLOW);
    	break;

    }

}

MenuEntry_t * getEntryFromPosition(MenuManager_t* manager, MenuPosition_t position){

    switch(position){
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
uint8_t getMenuRotaryPosition(int rotaryCounter){
      return ((uint16_t)rotaryCounter)%4;
}

void showMenuPage(MenuManager_t * manager, MenuPosition_t position){
    tftFillScreen(tft_BLACK);
    switch(manager->activeMode){
    case Entry:
	tftPrintColor(manager->activeEntry->title, TL_OFFSET_X , TL_OFFSET_Y, manager->activeEntry->color);
	break;

    case Page:
	    tftPrintColor(manager->activePage->TL->title, TL_OFFSET_X , TL_OFFSET_Y, manager->activePage->TL->color);
	    tftPrintColor(manager->activePage->TR->title, TR_OFFSET_X , TR_OFFSET_Y, manager->activePage->TR->color);
	    tftPrintColor(manager->activePage->BL->title, BL_OFFSET_X , BL_OFFSET_Y, manager->activePage->BL->color);
	    tftPrintColor(manager->activePage->BR->title, BR_OFFSET_X , BR_OFFSET_Y, manager->activePage->BR->color);
	    drawGrid();
	    higlightEntry(position);
	    break;

    }



}

void handleMenu(MenuManager_t * menuManager, int lastRotaryPosition)
   {

      if (getMenuRotaryPosition(getRotaryPosition()) != menuManager->currentPosition)
         {
            menuManager->currentPosition = getMenuRotaryPosition(getRotaryPosition());
             lastRotaryPosition = getMenuRotaryPosition(getRotaryPosition());
            drawGrid();
            higlightEntry(menuManager->currentPosition);
         }
      if (getRotaryPushButton() == true)
         {
            MenuPosition_t test = menuManager->currentPosition;
            MenuEntry_t *test1 = getEntryFromPosition(menuManager, menuManager->currentPosition);
            if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type == Page)
               {
                  getEntryFromPosition(menuManager, menuManager->currentPosition)->page->lastMenu =
                        menuManager->activePage;
                  menuManager->activePage = getEntryFromPosition(menuManager,
                        menuManager->currentPosition)->page;
               }
            else if (getEntryFromPosition(menuManager, menuManager->currentPosition)->type
                  == Entry)
               {
                  menuManager->activeEntry = getEntryFromPosition(menuManager,
                        menuManager->currentPosition);
                  menuManager->activeMode = Entry;
               }
            else
               {
                  menuManager->activePage = menuManager->activePage->lastMenu;
               }
            showMenuPage(menuManager, menuManager->currentPosition);
         }
   }

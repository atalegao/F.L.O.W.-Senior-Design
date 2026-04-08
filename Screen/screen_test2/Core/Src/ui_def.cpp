/*
 * ui_def.c
 *
 *  Created on: Mar 6, 2026
 *      Author: aditi
 */
#include "ui_def.h"

// testing a change??

ScreenState currentScreen;
Button node1Btn = {100, 200, 220, 80, "NODE 1", RA8875_BLUE};
Button node2Btn = {420, 200, 220, 80, "NODE 2", RA8875_GREEN};
Button backBtn  = {420, 200, 220, 80, "BACK", RA8875_YELLOW};

void draw_button(Button b) {
    tft->fillRectArea(b.x, b.y, b.w, b.h, b.color);
    tft->drawRect(b.x, b.y, b.w, b.h, RA8875_WHITE);

    tft->textMode();
    tft->textSetCursor(b.x + 10, b.y + b.h/2);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite(b.label);
    tft->graphicsMode();
}

void drawHome(void) {
    currentScreen = SCREEN_HOME;
    tft->graphicsMode();
    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textSetCursor(300, 50);
    tft->textEnlarge(1);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite("Node Menu");

    tft->graphicsMode();

    draw_button(node1Btn);
    draw_button(node2Btn);
}

void drawNode1(void) {
    currentScreen = SCREEN_NODE1;
    tft->graphicsMode();
    tft->fillScreen(RA8875_WHITE);
    HAL_Delay(1000);
    tft->fillScreen(RA8875_BLACK);
    //HAL_Delay(1000);
    tft->textMode();

    tft->textSetCursor(300, 50);
    tft->textEnlarge(1);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite("Node 1");
    //HAL_Delay(1000);
    tft->textSetCursor(300, 200);
    tft->textWrite("water high");
    tft->graphicsMode();
    //HAL_Delay(1000);
    draw_button(backBtn);
}

void drawNode2(void) {
    currentScreen = SCREEN_NODE2;

    tft->graphicsMode();
    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textColor(RA8875_WHITE, RA8875_RED);
    tft->textEnlarge(1);

    tft->textSetCursor(50, 50);
    tft->textWrite("Node 2");

    tft->textSetCursor(50, 120);
    tft->textWrite("water low");
    tft->graphicsMode();
        //HAL_Delay(1000);
    draw_button(backBtn);

}

bool buttonContains(Button b, uint16_t x, uint16_t y) {
    return (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h);
}

void uiHandleTouch(uint16_t x, uint16_t y) {
	if ( x==0 && y==0){
		return;
	}
    switch(currentScreen) {
        case SCREEN_HOME:
            if(buttonContains(node1Btn, x, y))
            {
            	drawNode1();
            }
            if(buttonContains(node2Btn, x, y))
            	{
            	drawNode2();
            	}
            break;

        case SCREEN_NODE1:
        case SCREEN_NODE2:
            if(buttonContains(backBtn, x, y)) {
            	drawHome();
            }
            break;
    }

}

void uiInit(Adafruit_RA8875* display) {
    tft = display;
    drawHome();
}

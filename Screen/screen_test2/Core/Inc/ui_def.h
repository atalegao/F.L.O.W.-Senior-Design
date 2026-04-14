/*
 * ui_def.h
 *
 *  Created on: Mar 2, 2026
 *      Author: aditi
 */

#ifndef INC_UI_DEF_H_
#define INC_UI_DEF_H_


#include <Adafruit_RA8875.hpp>

typedef struct {
    uint16_t x, y, w, h;
    const char* label;
    uint16_t color;
} Button;

typedef enum {
	SCREEN_HOME,
	SCREEN_NODE1,
	SCREEN_NODE2 }
ScreenState;

extern ScreenState currentScreen;
extern Adafruit_RA8875* tft;

void uiInit(Adafruit_RA8875* display);
void drawHome(void);
void drawNode1(void);
void drawNode2(void);
void uiHandleTouch(uint16_t x, uint16_t y);

#ifdef __cplusplus

#endif

#endif

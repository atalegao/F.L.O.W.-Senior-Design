/*
 * ui_def.h
 *
 *  Created on: Mar 2, 2026
 *      Author: aditi
 */

#ifndef INC_UI_DEF_H_
#define INC_UI_DEF_H_

#define MAX_NODES 10
#define MAX_PHONES 10
#define PHONE_LEN 16

#include <Adafruit_RA8875.hpp>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t x, y, w, h;
    const char* label;
    uint16_t color;
} Button;

typedef enum {
    SCREEN_HOME,
    SCREEN_NODES,
    SCREEN_NODE_DETAIL,
    SCREEN_PHONES,
    SCREEN_KEYPAD
} ScreenState;

typedef struct {
    uint16_t nodeId;
    const char* name;
    const char* status;
} Node;

extern ScreenState currentScreen;
extern Adafruit_RA8875* tft;

/* Core UI */
void uiInit(Adafruit_RA8875* display);
void uiHandleTouch(uint16_t x, uint16_t y);

/* Node handling */
void addNode(uint8_t id, const char* name, const char* status);

/* Phone handling */
void addPhone(const char* number);

/* Screens */
void drawHome(void);
void drawAllNodes(void);
void drawNodeDetail(Node* n);
void drawPhones(void);
void drawKeypad(void);

#endif

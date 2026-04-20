/*
 * ui_def.c
 *
 *  Created on: Mar 6, 2026
 *      Author: aditi
 */
#include "ui_def.h"
#include <string.h>

ScreenState currentScreen;
Adafruit_RA8875* tft;

/* Data storage */
Node nodes[MAX_NODES];
uint16_t nodeCount = 0;

char phoneNumbers[MAX_PHONES][PHONE_LEN];
uint16_t phoneCount = 0;

char keypadBuffer[PHONE_LEN] = {0};
uint8_t keypadIndex = 0;

Node* activeNode = NULL;

/* Buttons */
Button nodeMenu = {100, 200, 220, 80, "Nodes", RA8875_BLUE};
Button phoneMenu = {420, 200, 220, 80, "Phones", RA8875_GREEN};
Button backBtn  = {20, 20, 150, 60, "BACK", RA8875_YELLOW};
Button addPhoneBtn = {300, 380, 200, 80, "Add Phone", RA8875_GREEN};

/* ---------- Helpers ---------- */

void draw_button(Button b) {
    tft->fillRectArea(b.x, b.y, b.w, b.h, b.color);
    tft->drawRect(b.x, b.y, b.w, b.h, RA8875_WHITE);

    tft->textMode();
    tft->textSetCursor(b.x + 10, b.y + b.h/2);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite(b.label);
    tft->graphicsMode();
}

bool buttonContains(Button b, uint16_t x, uint16_t y) {
    return (x >= b.x && x <= b.x + b.w &&
            y >= b.y && y <= b.y + b.h);
}

/* ---------- Screens ---------- */

void drawHome(void) {
    currentScreen = SCREEN_HOME;
    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textSetCursor(300, 50);
    tft->textEnlarge(2);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite("Home");
    tft->graphicsMode();

    draw_button(nodeMenu);
    draw_button(phoneMenu);
}

void drawAllNodes(void) {
    currentScreen = SCREEN_NODES;
    tft->fillScreen(RA8875_BLACK);

    for (int i = 0; i < nodeCount; i++) {
        Button b = {
            50 + (i % 2) * 300,
            150 + (i / 2) * 120,
            220,
            80,
            nodes[i].name,
            RA8875_BLUE
        };
        draw_button(b);
    }

    draw_button(backBtn);
}

void drawNodeDetail(Node* n) {
    currentScreen = SCREEN_NODE_DETAIL;
    activeNode = n;

    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textSetCursor(50, 50);
    tft->textEnlarge(2);
    tft->textWrite(n->name);

    tft->textSetCursor(50, 150);
    tft->textWrite(n->status);

    tft->graphicsMode();

    draw_button(backBtn);
}

void drawPhones(void) {
    currentScreen = SCREEN_PHONES;
    tft->fillScreen(RA8875_BLACK);

    for (int i = 0; i < phoneCount; i++) {
        tft->textMode();
        tft->textSetCursor(50, 50 + i * 40);
        tft->textWrite(phoneNumbers[i]);
    }

    tft->graphicsMode();
    draw_button(addPhoneBtn);
    draw_button(backBtn);
}

void drawKeypad(void) {
    currentScreen = SCREEN_KEYPAD;
    tft->fillScreen(RA8875_BLACK);

    int num = 1;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            char label[2];

            if (r == 3 && c == 1) {
                label[0] = '0';
            } else if (r == 3) {
                continue;
            } else {
                label[0] = '0' + num++;
            }
            label[1] = '\0';

            Button b = {
                100 + c * 150,
                100 + r * 100,
                100,
                80,
                strdup(label),
                RA8875_BLUE
            };

            draw_button(b);
        }
    }

    draw_button(backBtn);
}

/* ---------- Logic ---------- */

void uiHandleTouch(uint16_t x, uint16_t y) {
    if (x == 0 && y == 0) return;

    switch(currentScreen) {

        case SCREEN_HOME:
            if (buttonContains(nodeMenu, x, y))
                drawAllNodes();

            if (buttonContains(phoneMenu, x, y))
                drawPhones();
            break;

        case SCREEN_NODES:
            for (int i = 0; i < nodeCount; i++) {
                Button b = {
                    50 + (i % 2) * 300,
                    150 + (i / 2) * 120,
                    220,
                    80,
                    nodes[i].name,
                    RA8875_BLUE
                };

                if (buttonContains(b, x, y)) {
                    drawNodeDetail(&nodes[i]);
                }
            }

            if (buttonContains(backBtn, x, y))
                drawHome();
            break;

        case SCREEN_NODE_DETAIL:
            if (buttonContains(backBtn, x, y))
                drawAllNodes();
            break;

        case SCREEN_PHONES:
            if (buttonContains(addPhoneBtn, x, y))
                drawKeypad();

            if (buttonContains(backBtn, x, y))
                drawHome();
            break;

        case SCREEN_KEYPAD:
            if (buttonContains(backBtn, x, y))
                drawPhones();
            break;
    }
}

/* ---------- Data ---------- */

void addNode(uint8_t id, const char* name, const char* status) {
    if (nodeCount >= MAX_NODES) return;

    nodes[nodeCount].nodeId = id;
    nodes[nodeCount].name = name;
    nodes[nodeCount].status = status;
    nodeCount++;
}

void addPhone(const char* number) {
    if (phoneCount >= MAX_PHONES) return;

    strncpy(phoneNumbers[phoneCount], number, PHONE_LEN);
    phoneCount++;
}

/* ---------- Init ---------- */

void uiInit(Adafruit_RA8875* display) {
    tft = display;

    addNode(1, "Node 1", "Water High");
    addNode(2, "Node 2", "Water Low");

    drawHome();
}

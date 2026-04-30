/*
 * ui_def.h
 *
 *  Created on: Mar 2, 2026
 *      Author: aditi
 */
/*
 * ui_def.h
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef INC_UI_DEF_H_
#define INC_UI_DEF_H_

#define MAX_NODES 10
#define MAX_PHONES 10
#define PHONE_LEN 16
#define HISTORY_LEN 5
#define KEYPAD_H 2


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
    SCREEN_KEYPAD,
    SCREEN_NODE_HISTORY,
	SCREEN_POLL

} ScreenState;
typedef struct {
    float waterHeight;
    uint32_t timestamp;
} WaterSample;
typedef struct {
    uint16_t nodeId;
    const char* name;
    const char* status;

    float latestWaterHeight;
    uint32_t latestTimestamp;
    float batteryPercent;

    WaterSample history[HISTORY_LEN];
    uint8_t historyIndex;

} Node;

extern ScreenState currentScreen;
extern Adafruit_RA8875* tft;

void uiInit(Adafruit_RA8875* display);
void uiHandleTouch(uint16_t x, uint16_t y);

void addNode(uint16_t id, const char* name, const char* status);

void addPhone(const char* number);

void drawHome(void);
void draw7to10Nodes(void);
void draw1to6Nodes(void);
void drawNodeDetail(Node* n);

void drawPhones(void);
void drawKeypad(void);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);
void updateNodeData(uint16_t nodeId, float water, uint32_t timestamp, float battery, const char* name);

void updateFloodStatus(void);
void commitPollingFrequency(void);


#endif

#ifdef __cplusplus
}
#endif

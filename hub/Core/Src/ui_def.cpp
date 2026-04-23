/*
 * ui_def.c
 *
 *  Created on: Mar 6, 2026
 *      Author: aditi
 */
/*
 * ui_def.cpp
 */


#include "ui_def.h"
#include <string.h>
#include <math.h>

#define WATER_THRESHOLD 120 //mm
ScreenState currentScreen;

Node nodes[MAX_NODES];
uint16_t nodeCount = 0;

static uint8_t nodeAboveThreshold[MAX_NODES];

volatile uint16_t freq_req = 120;
volatile uint16_t poll_freq_active = 120;
volatile bool new_freq_flag = false;

char keypadBuffer[PHONE_LEN] = {0};
uint8_t keypadIndex = 0;
volatile bool change_polling = false;
Node* activeNode = NULL;
uint8_t node_screen = 1;

volatile bool floodImminent = false;
bool disp_ok = false;


Button nodeMenu = {100, 200, 220, 80, "Nodes", RA8875_BLUE};
Button pollingScreen = {420, 200, 220, 80, "Polling", RA8875_GREEN};
Button backBtn  = {630, 380, 150, 80, "BACK", RA8875_GREEN};
Button addPhoneBtn = {300, 380, 300, 80, "Add Phone", RA8875_GREEN};
Button saveBtn = {550, 20, 200, 80, "Save", RA8875_GREEN};
Button historyBtn = {300, 320, 250, 80, "History", RA8875_BLUE};
Button upBtn = {630, 180, 100, 80, "UP", RA8875_YELLOW};
Button downBtn = {630, 300, 100, 80, "DWN", RA8875_YELLOW};
Button incBtn = {400, 180, 100, 80, "+", RA8875_BLUE};
Button decBtn = {400, 300, 100, 80, "-", RA8875_BLUE};
Button okBtn = {500, 300, 100, 80, "OK", RA8875_BLUE};



Button floodBtn = {200, 300, 300, 80, (floodImminent? "Dec. Polling" : "Dec. Polling"), (floodImminent? RA8875_RED: RA8875_GREEN)};

void draw_button(Button b) {
    tft->fillRectArea(b.x, b.y, b.w, b.h, b.color);
    tft->drawRect(b.x, b.y, b.w, b.h, RA8875_WHITE);

    tft->textMode();
    tft->textSetCursor(b.x + 20, b.y + 20);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite(b.label);
    tft->graphicsMode();
}

bool buttonContains(Button b, uint16_t x, uint16_t y) {
    return (x >= b.x && x <= b.x + b.w &&
            y >= b.y && y <= b.y + b.h);
}


void drawHome(void) {
    currentScreen = SCREEN_HOME;
    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textSetCursor(300, 50);
    tft->textEnlarge(2);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite("Home");
    if (floodImminent)
        {
            tft->textSetCursor(120, 350);
            tft->textTransparent(RA8875_RED);
            tft->textWrite("FLOODING OCCURRING!!!");
        }
    tft->graphicsMode();

    draw_button(nodeMenu);
    draw_button(pollingScreen);
}

void draw1to6Nodes(void) {
    currentScreen = SCREEN_NODES;
    node_screen = 1;
    tft->fillScreen(RA8875_BLACK);

    for (int i = 0; (i < (nodeCount>6? 6 : nodeCount)); i++) {
        Button b = {
            50 + (i % 2) * 300,
            50 + (i / 2) * 120,
            220,
            80,
            nodes[i].name,
            RA8875_BLUE
        };
        draw_button(b);
    }

    if(nodeCount> 6) draw_button(downBtn);

    draw_button(backBtn);
}
void draw7to10Nodes(void) {
    currentScreen = SCREEN_NODES;
    node_screen = 2;
    tft->fillScreen(RA8875_BLACK);

    for (int i = 0; i < 4; i++) {
        Button b = {
            50 + (i % 2) * 300,
            50 + (i / 2) * 120,
            220,
            80,
            nodes[i+6].name,
            RA8875_BLUE
        };
        draw_button(b);
    }
    draw_button(upBtn);
    draw_button(backBtn);
}

void drawNodeDetail(Node* n) {
    currentScreen = SCREEN_NODE_DETAIL;
    activeNode = n;

    tft->fillScreen(RA8875_BLACK);

    tft->textMode();
    tft->textSetCursor(300, 50);
    tft->textEnlarge(2);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite(n->name);
//TODO sprintf will not work so i need to figure out something else
    tft->textSetCursor(50, 130);
    char buf[32];

    // Water
    tft->textSetCursor(50, 130);
    tft->textWrite("Latest data:");

    tft->textSetCursor(400, 130);
    ftoa(n->latestWaterHeight, buf, 2);
    tft->textWrite(buf);

    // Time
    tft->textSetCursor(50, 180);
    tft->textWrite("Time stamp:");

    tft->textSetCursor(400, 180);
    intToStr(n->latestTimestamp, buf, 0);
    tft->textWrite(buf);

    // Battery
    tft->textSetCursor(50, 230);
    tft->textWrite("Battery status:");
    tft->textSetCursor(400, 230);
    if(n->batteryPercent <= 0.2)
    {
        tft->textWrite("Battery low");

    }else if(n->batteryPercent > 0.2)
    {
        tft->textWrite("Battery Good");

    }else
    {
        tft->textWrite("No Data");

    }




    tft->graphicsMode();

    draw_button(historyBtn);
    draw_button(backBtn);
}
void drawNodeHistory(Node* n)
{
    currentScreen = SCREEN_NODE_HISTORY;
    tft->fillScreen(RA8875_BLACK);

    char buf[32];

    tft->textMode();

    // Title
    tft->textSetCursor(220, 10);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite("History");

    // Column headers
    tft->textSetCursor(50, 60);
    tft->textWrite("Timestamp");

    tft->textSetCursor(350, 60);
    tft->textWrite("Water Level");

    int count = (n->historyIndex < 5) ? n->historyIndex : 5;

    for (int i = 0; i < count; i++)
    {
        int y = 70 + i * 30;

        float w = n->history[i].waterHeight;
        uint32_t t = n->history[i].timestamp;

        // Timestamp label + value
        tft->textSetCursor(50, y);
        tft->textTransparent(RA8875_WHITE);
        tft->textWrite("T:");
        intToStr(t, buf, 0);
        tft->textWrite(buf);

        // Water level label + value
        tft->textSetCursor(350, y);

        if (w >= WATER_THRESHOLD)
            tft->textTransparent(RA8875_RED);
        else
            tft->textTransparent(RA8875_WHITE);

        tft->textWrite("W:");
        ftoa(w, buf, 2);
        tft->textWrite(buf);
    }

    tft->graphicsMode();
    draw_button(backBtn);
}
void drawPollScreen(void)
{
	currentScreen = SCREEN_POLL;
	tft->fillScreen(RA8875_BLACK);

	tft->textMode();
	tft->textSetCursor(50, 50);
    tft->textTransparent(RA8875_WHITE);

	tft->textWrite("Polling Frequency Changes");
	tft->textSetCursor(300, 200);
	char buf[4];
	ftoa(freq_req, buf, 0);
    tft->textTransparent(RA8875_WHITE);
	tft->textWrite(buf);
	if(disp_ok){

		tft->textSetCursor(500, 220);
		tft->textTransparent(RA8875_GREEN);
		tft->textWrite("Saved");
	}


	tft->graphicsMode();
	draw_button(incBtn);
	draw_button(decBtn);
	draw_button(okBtn);
	draw_button(backBtn);

}
//void drawPhones(void) {
//    currentScreen = SCREEN_PHONES;
//    tft->fillScreen(RA8875_BLACK);
//
//    for (int i = 0; i < phoneCount; i++) {
//        tft->textMode();
//        tft->textSetCursor(300, 50 + i * 40);
//        tft->textTransparent(RA8875_WHITE);
//
//        tft->textWrite(phoneNumbers[i]);
//    }
//
//    tft->graphicsMode();
//    draw_button(addPhoneBtn);
//    draw_button(backBtn);
//}

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
                KEYPAD_H + c * 150,
                100 + r * 100,
                100,
                80,
                strdup(label),
                RA8875_BLUE
            };

            draw_button(b);
        }
    }

    tft->textMode();
    tft->textSetCursor(300, 50);
    tft->textTransparent(RA8875_WHITE);
    tft->textWrite(keypadBuffer);

//    if (showSaved) {
//        tft->textSetCursor(300, 50);
//        tft->textTransparent(RA8875_GREEN);
//        tft->textWrite("Saved!");
//    }

    tft->graphicsMode();

    draw_button(saveBtn);
    draw_button(backBtn);
}

void uiHandleTouch(uint16_t x, uint16_t y) {
    if (x == 0 && y == 0) return;

    switch(currentScreen) {

        case SCREEN_HOME:
            if (buttonContains(nodeMenu, x, y))
                draw1to6Nodes();

            if (buttonContains(pollingScreen, x, y))
            	drawPollScreen();
            break;

        case SCREEN_NODES:
        	if(node_screen == 1)
        	{
				for (int i = 0; i < 6; i++) {
					Button b = {
						50 + (i % 2) * 300,
						50 + (i / 2) * 120,
						220,
						80,
						nodes[i].name,
						RA8875_BLUE
					};

					if (buttonContains(b, x, y)) {
						drawNodeDetail(&nodes[i]);
					}
				}
        	}else if(node_screen == 2)
        	{
				for (int i = 0; i < 4; i++) {
					Button b = {
					50 + (i % 2) * 300,
					50 + (i / 2) * 120,
					220,
					80,
					nodes[i+6].name,
					RA8875_BLUE
				};

				if (buttonContains(b, x, y)) {
				drawNodeDetail(&nodes[i+6]);
				}
			}
        	}

            if (buttonContains(backBtn, x, y))
                drawHome();
            if(buttonContains(downBtn,x,y) && node_screen == 1 && nodeCount > 6)
            {
            	draw7to10Nodes();
            }else if(buttonContains(upBtn,x,y) && node_screen == 2){
            	draw1to6Nodes();
            }
            break;

        case SCREEN_NODE_DETAIL:

        	if (buttonContains(historyBtn, x, y)) {
        	     drawNodeHistory(activeNode);
        	    }
            if (buttonContains(backBtn, x, y))
                draw1to6Nodes();
            break;


        case SCREEN_POLL:



            if (buttonContains(incBtn, x, y) && freq_req < 180) {
            	freq_req = freq_req + 30;
                drawPollScreen();
            }
            if (buttonContains(decBtn, x, y) && freq_req > 60) {
            	freq_req = freq_req - 30;
                drawPollScreen();
            }
            if (buttonContains(okBtn, x, y))
            {
            	disp_ok = true;
                commitPollingFrequency();
            	drawPollScreen();
            }

            if (buttonContains(backBtn, x, y))
            {
            	disp_ok = false;
                drawHome();
            }
            break;
        case SCREEN_NODE_HISTORY:
        	if (buttonContains(backBtn, x, y))
        	   {
        	            	drawNodeDetail(activeNode);
        	   }

    }

}
void commitPollingFrequency(void)
{
    poll_freq_active = freq_req;
    new_freq_flag = true;
}
void updateNodeData(uint16_t nodeId, float water, uint32_t timestamp, float battery, bool status)
{
    int found = 0;

    for (int i = 0; i < nodeCount; i++)
    {
        if (nodes[i].nodeId == nodeId)
        {
            Node *n = &nodes[i];

            // update latest values
            n->latestWaterHeight = water;
            n->latestTimestamp = timestamp;
            n->batteryPercent = battery;


            // store in circular history buffer
            n->history[n->historyIndex].waterHeight = water;
            n->history[n->historyIndex].timestamp = timestamp;


            n->historyIndex = (n->historyIndex + 1) % HISTORY_LEN;
            found = 1;
            break;
        }
    }
    if (!found && nodeCount < MAX_NODES)
        {
            nodes[nodeCount].nodeId = nodeId;
            nodes[nodeCount].latestWaterHeight = water;
            nodes[nodeCount].latestTimestamp = timestamp;
            nodes[nodeCount].batteryPercent = battery;
            nodes[nodeCount].historyIndex = 0;

            nodeCount++;
        }
    updateFloodStatus();


}
void addNode(uint8_t id, const char* name, const char* status) {
    if (nodeCount >= MAX_NODES) return;

    nodes[nodeCount].nodeId = id;
    nodes[nodeCount].name = name;
    nodes[nodeCount].status = status;

    nodes[nodeCount].latestWaterHeight = 0;
    nodes[nodeCount].latestTimestamp = 0;
    nodes[nodeCount].batteryPercent = 0;
    nodes[nodeCount].historyIndex = 0;

    for (int i = 0; i < HISTORY_LEN; i++) {
        nodes[nodeCount].history[i].waterHeight = 0;
        nodes[nodeCount].history[i].timestamp = 0;
    }

    nodeCount++;
}

//void addNode(uint8_t id, const char* name, const char* status)
//{
//    if (nodeCount >= MAX_NODES) return;
//
//    Node *n = &nodes[nodeCount];
//
//    n->nodeId = id;
//
//    strncpy(n->name, name, sizeof(n->name) - 1);
//    n->name[sizeof(n->name) - 1] = '\0';
//
//    strncpy(n->status, status, sizeof(n->status) - 1);
//    n->status[sizeof(n->status) - 1] = '\0';
//
//    n->latestWaterHeight = 0;
//    n->latestTimestamp = 0;
//    n->batteryPercent = 0;
//    n->historyIndex = 0;
//
//    for (int i = 0; i < HISTORY_LEN; i++) {
//        n->history[i].waterHeight = 0;
//        n->history[i].timestamp = 0;
//    }
//
//    nodeCount++;
//}
//void addPhone(const char* number) {
//    if (phoneCount >= MAX_PHONES) return;
//
//    strncpy(phoneNumbers[phoneCount], number, PHONE_LEN);
//    phoneCount++;
//}
void updateFloodStatus(void)
{
    int above = 0;
    int validNodes = nodeCount;

    for (int i = 0; i < nodeCount; i++)
    {
        if (nodes[i].latestWaterHeight >= WATER_THRESHOLD)
        {
            nodeAboveThreshold[i] = 1;
            above++;
        }
        else
        {
            nodeAboveThreshold[i] = 0;
        }
    }

    if (validNodes == 0)
    {
        floodImminent = false;
        return;
    }

    float ratio = (float)above / (float)validNodes;

    floodImminent = (ratio >= 0.60f);
}
void uiInit(Adafruit_RA8875* display) {
    tft = display;

//    addNode(1, "Node 1", "Water High");
//    addNode(2, "Node 2", "Water Low");
//    addNode(3, "Node 3", "Water High");
//    addNode(4, "Node 4", "Water Low");
//    addNode(5, "Node 5", "Water High");
//    addNode(6, "Node 6", "Water Low");
//    addNode(7, "Node 7", "Water High");
//    addNode(8, "Node 8", "Water Low");
//    addNode(9, "Node 9", "Water High");
//    addNode(10, "Node 10", "Water Low");
    drawHome();
}
//from geeksforgeeks
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}



/*
 * entryPointCPP.cpp
 *
 *  Created on: Jan 26, 2025
 *      Author: johng
 */

#include <Adafruit_RA8875.h>
#include "entryPointCPP.hpp"


Adafruit_RA8875 *tft;

void initTest(SPI_HandleTypeDef *halSPI) {
	HAL_GPIO_WritePin(RA8875_CS_GPIO_Port, RA8875_CS_Pin, CS_DISABLE);

	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	delay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	delay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LCD_WAIT_GPIO_Port, LCD_WAIT_Pin, GPIO_PIN_SET);

	tft = new Adafruit_RA8875(halSPI);

	int stat = tft->begin(RA8875_800x480);

	if (stat) {
	  printf("RA8875 Found\r\n");
	} else {
	  Error_Handler();
	  }
}

void testLCD(bool buildTest) {
	if (buildTest) {
		printf("================  Graphics Mode =============\r\n");

	  tft->displayOn(true);
	  HAL_Delay(1);
	  tft->GPIOX(true);
	  HAL_Delay(1);
	  tft->PWM1config(true, RA8875_PWM_CLK_DIV1024);
	  tft->PWM1out(255);
	  HAL_Delay(1);

	  tft->graphicsMode();

	  tft->fillScreen(RA8875_WHITE);
	  HAL_Delay(1000);
	  // Play with PWM
	  printf("Start: Play with PWM\r\n");

	  for (uint8_t i=255; i!=0; i-=5 )
	  {
		tft->PWM1out(i);
		delay(100);
	  }
	  for (uint8_t i=0; i!=255; i+=5 )
	  {
		tft->PWM1out(i);
		delay(100);
	  }
	  tft->PWM1out(255);

	  printf("END: Play with PWM\r\n");
	  HAL_Delay(2000);

	  printf("FillSreen Red\r\n");
	  tft->fillScreen(RA8875_RED);
	  HAL_Delay(1000);
	  printf("FillSreen Yellow\r\n");
	  tft->fillScreen(RA8875_YELLOW);
	  HAL_Delay(1000);
	  printf("FillSreen Green\r\n");
	  tft->fillScreen(RA8875_GREEN);
	  HAL_Delay(1000);
	  printf("FillSreen Cyan\r\n");
	  tft->fillScreen(RA8875_CYAN);
	  HAL_Delay(1000);
	  printf("FillSreen Magenta\r\n");
	  tft->fillScreen(RA8875_MAGENTA);
	  HAL_Delay(1000);
	  printf("FillSreen Black\r\n");
	  tft->fillScreen(RA8875_BLACK);
	  HAL_Delay(1000);

	  // Try some GFX acceleration!
	  printf("drawCircle\r\n");
	  tft->drawCircle(100, 100, 50, RA8875_BLACK);
	  printf("fillCircle\r\n");
	  tft->fillCircle(100, 100, 49, RA8875_GREEN);

	  HAL_Delay(1000);

	  printf("fillRect\r\n");
	  tft->fillRect(11, 11, 398, 198, RA8875_BLUE);
	  HAL_Delay(1000);
	  printf("drawRect\r\n");
	  tft->drawRect(10, 10, 400, 200, RA8875_GREEN);
	  HAL_Delay(1000);
	  printf("fillRoundRect\r\n");
	  tft->fillRoundRect(200, 10, 200, 100, 10, RA8875_RED);
	  HAL_Delay(1000);
	  printf("drawPixel\r\n");
	  tft->drawPixel(10,10,RA8875_BLACK);
	  HAL_Delay(1000);
	  printf("drawPixel\r\n");
	  tft->drawPixel(11,11,RA8875_BLACK);
	  HAL_Delay(1000);
	  printf("drawLine\r\n");
	  tft->drawLine(10, 10, 200, 100, RA8875_RED);
	  HAL_Delay(1000);
	  printf("drawTriangle\r\n");
	  tft->drawTriangle(200, 15, 250, 100, 150, 125, RA8875_BLACK);
	  HAL_Delay(1000);
	  printf("fillTriangle\r\n");
	  tft->fillTriangle(200, 16, 249, 99, 151, 124, RA8875_YELLOW);
	  HAL_Delay(1000);
	  printf("drawEllipse\r\n");
	  tft->drawEllipse(300, 100, 100, 40, RA8875_BLACK);
	  HAL_Delay(1000);
	  printf("fillEllipse\r\n");
	  tft->fillEllipse(300, 100, 98, 38, RA8875_GREEN);
	  HAL_Delay(1000);
	  // Argument 5 (curvePart) is a 2-bit value to control each corner (select 0, 1, 2, or 3)
	  printf("drawCurve\r\n");
	  tft->drawCurve(50, 100, 80, 40, 2, RA8875_BLACK);
	  HAL_Delay(1000);
	  printf("fillCurve\r\n");
	  tft->fillCurve(50, 100, 78, 38, 2, RA8875_WHITE);
	} else {
		printf("================  Text Mode =============\r\n");
	  tft->displayOn(true);
	  tft->GPIOX(true);      // Enable TFT - display enable tied to GPIOX
	  tft->PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
	  tft->PWM1out(255);
	  printf("fillScreen\r\n");
	  tft->fillScreen(RA8875_BLACK);

	  /* Switch to text mode */
	  tft->textMode();
	  tft->cursorBlink(32);


	  /* Set a solid for + bg color ... */

	  /* ... or a fore color plus a transparent background */


	  /* Set the cursor location (in pixels) */
	  tft->textSetCursor(10, 10);

	  /* Render some text! */
	  printf("Render lots of Texts\r\n");
	  char string[80] = "Hello, World! ";
	  tft->textTransparent(RA8875_WHITE);
	  tft->textWrite(string);
	  HAL_Delay(500);
	  tft->textColor(RA8875_WHITE, RA8875_RED);
	  tft->textWrite(string);
	  HAL_Delay(500);
	  tft->textTransparent(RA8875_CYAN);
	  tft->textWrite(string);
	  HAL_Delay(500);
	  tft->textTransparent(RA8875_GREEN);
	  tft->textWrite(string);
	  HAL_Delay(500);
	  tft->textColor(RA8875_YELLOW, RA8875_CYAN);
	  tft->textWrite(string);
	  HAL_Delay(500);
	  tft->textColor(RA8875_BLACK, RA8875_MAGENTA);
	  tft->textWrite(string);

	  HAL_Delay(1000);

	  tft->textSetCursor(100, 100);
	  tft->textEnlarge(1);
	  tft->textTransparent(RA8875_YELLOW);
	  tft->textWrite(string);

	  tft->textSetCursor(100, 150);
	  tft->textEnlarge(2);
	  tft->textTransparent(RA8875_YELLOW);
	  tft->textWrite(string);

	  tft->textSetCursor(100, 200);
	  tft->textEnlarge(3);
	  tft->textTransparent(RA8875_YELLOW);
	  tft->textWrite(string);

	  tft->textSetCursor(100, 300);
	  tft->textEnlarge(0);
	  tft->textTransparent(RA8875_YELLOW);
	  tft->textWrite(string);
	}
}

/*
 * entryPointCPP.cpp
 *
 *  Created on: Jan 26, 2025
 *      Author: johng
 */

#include <Adafruit_RA8875.h>
#include "entryPointCPP.hpp"
//#include <Adafruit_RA8875.cpp>


Adafruit_RA8875 *tft;

uint16_t width = 800;
uint16_t height = 480;

void initTest(SPI_HandleTypeDef *halSPI) {
	HAL_GPIO_WritePin(RA8875_CS_GPIO_Port, RA8875_CS_Pin, CS_DISABLE);

	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
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
		HAL_Delay(100);
	  }
	  for (uint8_t i=0; i!=255; i+=5 )
	  {
		tft->PWM1out(i);
		HAL_Delay(100);
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
	  tft->fillRectArea(11, 11, 398, 198, RA8875_BLUE);
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

//touchscreen stuff below

//Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);
tsPoint_t       _tsLCDPoints[3];
tsPoint_t       _tsTSPoints[3];
tsMatrix_t      _tsMatrix;

#define EEPROMLOCATION 100

// Use to force a recalibration
#define FORCE_CALIBRATION false

/**************************************************************************/
/*!
    @brief Calculates the difference between the touch screen and the
           actual screen co-ordinates, taking into account misalignment
           and any physical offset of the touch screen.

    @note  This is based on the public domain touch screen calibration code
           written by Carlos E. Vidales (copyright (c) 2001).

           For more information, see the following app notes:

           - AN2173 - Touch Screen Control and Calibration
             Svyatoslav Paliy, Cypress Microsystems
           - Calibration in touch-screen systems
             Wendy Fang and Tony Chang,
             Analog Applications Journal, 3Q 2007 (Texas Instruments)
*/
/**************************************************************************/
int setCalibrationMatrix( tsPoint_t * displayPtr, tsPoint_t * screenPtr, tsMatrix_t * matrixPtr)
{
  int  retValue = 0;

  matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                       ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

  if( matrixPtr->Divider == 0 )
  {
    retValue = -1 ;
  }
  else
  {
    matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                    ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

    matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) -
                    ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

    matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

    matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) -
                    ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;

    matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) -
                    ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

    matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;

  }

  return( retValue ) ;
}

/**************************************************************************/
/*!
    @brief  Converts raw touch screen locations (screenPtr) into actual
            pixel locations on the display (displayPtr) using the
            supplied matrix.

    @param[out] displayPtr  Pointer to the tsPoint_t object that will hold
                            the compensated pixel location on the display
    @param[in]  screenPtr   Pointer to the tsPoint_t object that contains the
                            raw touch screen co-ordinates (before the
                            calibration calculations are made)
    @param[in]  matrixPtr   Pointer to the calibration matrix coefficients
                            used during the calibration process (calculated
                            via the tsCalibrate() helper function)

    @note  This is based on the public domain touch screen calibration code
           written by Carlos E. Vidales (copyright (c) 2001).
*/
/**************************************************************************/
int calibrateTSPoint( tsPoint_t * displayPtr, tsPoint_t * screenPtr, tsMatrix_t * matrixPtr )
{
  int  retValue = 0 ;

  if( matrixPtr->Divider != 0 )
  {
    displayPtr->x = ( (matrixPtr->An * screenPtr->x) +
                      (matrixPtr->Bn * screenPtr->y) +
                       matrixPtr->Cn
                    ) / matrixPtr->Divider ;

    displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) +
                      (matrixPtr->En * screenPtr->y) +
                       matrixPtr->Fn
                    ) / matrixPtr->Divider ;
  }
  else
  {
    return -1;
  }

  return( retValue );
}

/**************************************************************************/
/*!
    @brief  Waits for a touch event
*/
/**************************************************************************/
void waitForTouchEvent(tsPoint_t * point)
{
  /* Clear the touch data object and placeholder variables */
  memset(point, 0, sizeof(tsPoint_t));

  /* Clear any previous interrupts to avoid false buffered reads */
  uint16_t x, y;
  tft->touchRead(&x, &y);
  delay(1);

  /* Wait around for a new touch event (INT pin goes low) */
  //while (digitalRead(RA8875_INT))
  while(HAL_GPIO_ReadPin(RA8875_INT_GPIO_Port, RA8875_INT_Pin) == 0)
  {
  }

  /* Make sure this is really a touch event */
  if (tft->touched())
  {
    tft->touchRead(&x, &y);
    point->x = x;
    point->y = y;
    printf("Touch: ");
    printf("%ld",point->x); printf(", "); printf("%ld",point->y);
  }
  else
  {
    point->x = 0;
    point->y = 0;
  }
}

/**************************************************************************/
/*!
    @brief  Renders the calibration screen with an appropriately
            placed test point and waits for a touch event
*/
/**************************************************************************/
tsPoint_t renderCalibrationScreen(uint16_t x, uint16_t y, uint16_t radius)
{
  tft->fillScreen(RA8875_WHITE);
  tft->drawCircle(x, y, radius, RA8875_RED);
  tft->drawCircle(x, y, radius + 2, 0x8410);  /* 50% Gray */

  // Wait for a valid touch events
  tsPoint_t point = { 0, 0 };

  /* Keep polling until the TS event flag is valid */
  bool valid = false;
  while (!valid)
  {
    waitForTouchEvent(&point);
    if (point.x || point.y)
    {
      valid = true;
    }
  }

  return point;
}

/**************************************************************************/
/*!
    @brief  Starts the screen calibration process.  Each corner will be
            tested, meaning that each boundary (top, left, right and
            bottom) will be tested twice and the readings averaged.
*/
/**************************************************************************/
void tsCalibrate(void)
{
  tsPoint_t data;

  /* --------------- Welcome Screen --------------- */
  printf("Starting the calibration process");
  data = renderCalibrationScreen(width / 2, height / 2, 5);
  HAL_Delay(250);

  /* ----------------- First Dot ------------------ */
  // 10% over and 10% down
  data = renderCalibrationScreen(width / 10, height / 10, 5);
  _tsLCDPoints[0].x = width / 10;
  _tsLCDPoints[0].y = height / 10;
  _tsTSPoints[0].x = data.x;
  _tsTSPoints[0].y = data.y;
  printf("Point 1 - LCD");
  printf(" X: ");
  printf("%ld",_tsLCDPoints[0].x);
  printf(" Y: ");
  printf("%ld",_tsLCDPoints[0].y);
  printf(" TS X: ");
  printf("%ld",_tsTSPoints[0].x);
  printf(" Y: ");
  printf("%ld",_tsTSPoints[0].y);
  HAL_Delay(250);

  /* ---------------- Second Dot ------------------ */
  // 50% over and 90% down
  data = renderCalibrationScreen(width / 2, height - height / 10, 5);
  _tsLCDPoints[1].x = width / 2;
  _tsLCDPoints[1].y = height - height / 10;
  _tsTSPoints[1].x = data.x;
  _tsTSPoints[1].y = data.y;
  printf("Point 2 - LCD");
  printf(" X: ");
  printf("%ld",_tsLCDPoints[1].x);
  printf(" Y: ");
  printf("%ld",_tsLCDPoints[1].y);
  printf(" TS X: ");
  printf("%ld",_tsTSPoints[1].x);
  printf(" Y: ");
  printf("%ld",_tsTSPoints[1].y);
  HAL_Delay(500);

  /* ---------------- Third Dot ------------------- */
  // 90% over and 50% down
  data = renderCalibrationScreen(width - width / 10, height / 2, 5);
  _tsLCDPoints[2].x = width - width / 10;
  _tsLCDPoints[2].y = height / 2;
  _tsTSPoints[2].x = data.x;
  _tsTSPoints[2].y = data.y;
  printf("Point 3 - LCD");
  printf(" X: ");
  printf("%ld",_tsLCDPoints[2].x);
  printf(" Y: ");
  printf("%ld",_tsLCDPoints[2].y);
  printf(" TS X: ");
  printf("%ld",_tsTSPoints[2].x);
  printf(" Y: ");
  printf("%ld",_tsTSPoints[2].y);
  HAL_Delay(500);

  /* Clear the screen */
  tft->fillScreen(RA8875_WHITE);

  // Do matrix calculations for calibration and store to EEPROM
  setCalibrationMatrix(&_tsLCDPoints[0], &_tsTSPoints[0], &_tsMatrix);
}

/**************************************************************************/
/*!
this is for testing the touchscreen
call initTest then this and then loop to test the touchscreen
*/
/**************************************************************************/
void setup()
{
  //Serial.begin(9600);
//  printf("Hello, RA8875!");
//
//  /* Initialize the display using 'RA8875_480x272' or 'RA8875_800x480' */
//    if (!tft->begin(RA8875_480x272))
//  {
//    printf("RA8875 not found ... check your wires!");
//    while (1);
//  }

  /* Enables the display and sets up the backlight */
  printf("Found RA8875");
  tft->displayOn(true);
  tft->GPIOX(true); // Enable TFT - display enable tied to GPIOX
  tft->PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft->PWM1out(255);

  /* Enable the touch screen */
  printf("Enabled the touch screen");
  //pinMode(RA8875_INT, INPUT);
  //digitalWrite(RA8875_INT, HIGH);
  HAL_GPIO_WritePin(RA8875_INT_GPIO_Port, RA8875_INT_Pin, GPIO_PIN_SET);
  tft->touchEnable(true);

  // Try some GFX acceleration!
  //tft.drawCircle(100, 100, 50, RA8875_BLACK);
  //tft.fillCircle(100, 100, 49, RA8875_GREEN);
  //tft.drawPixel(10,10,RA8875_BLACK);
  //tft.drawPixel(11,11,RA8875_BLACK);
  //tft.drawRect(10, 10, 400, 200, RA8875_GREEN);
  //tft.fillRect(11, 11, 398, 198, RA8875_BLUE);
  //tft.drawLine(10, 10, 200, 100, RA8875_RED);

  tft->fillScreen(RA8875_WHITE);
  HAL_Delay(100);

#if defined(EEPROM_SUPPORTED)
  /* Start the calibration process */
  if (FORCE_CALIBRATION || tft.readCalibration(EEPROMLOCATION, &_tsMatrix) == false ){
    printf("Calibration not found.  Calibrating..\n");
    tsCalibrate();
  }
  else
    printf("Calibration found\n");
#else
  tsCalibrate();
#endif
  /* _tsMatrix should now be populated with the correct coefficients! */
  printf("Waiting for touch events ...");
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void loop()
{
  tsPoint_t raw;
  tsPoint_t calibrated;

  /* Wait around for a touch event */
  waitForTouchEvent(&raw);

  /* Calcuate the real X/Y position based on the calibration matrix */
  calibrateTSPoint(&calibrated, &raw, &_tsMatrix );

  /* Draw a single pixel at the calibrated point */
  tft->fillCircle(calibrated.x, calibrated.y, 3, RA8875_BLACK);
}


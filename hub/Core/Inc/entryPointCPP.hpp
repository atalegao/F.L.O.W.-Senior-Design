/*
 * entryPointCPP.hpp
 *
 *  Created on: Jan 26, 2025
 *      Author: johng
 */

#ifndef INC_ENTRYPOINTCPP_HPP_
#define INC_ENTRYPOINTCPP_HPP_


#include <Adafruit_RA8875.hpp>
#include "main.h"
#include <stdbool.h>

// Define all C function calls that will be called from main.c in the following extern "C" group
// Using extern "C" stops name mangling and allows "C" code to call C++ code

#ifdef __cplusplus
extern "C" {
#endif
	void update_on_touch();
	void initTest(SPI_HandleTypeDef *halSPI);
	void testLCD(bool buildTest);
	int setCalibrationMatrix( tsPoint_t * displayPtr, tsPoint_t * screenPtr, tsMatrix_t * matrixPtr);
	int calibrateTSPoint( tsPoint_t * displayPtr, tsPoint_t * screenPtr, tsMatrix_t * matrixPtr );
	void waitForTouchEvent(tsPoint_t * point);
	tsPoint_t renderCalibrationScreen(uint16_t x, uint16_t y, uint16_t radius);
	void tsCalibrate(void);
	void setup();
	void loop();
#ifdef __cplusplus
}
#endif


#endif /* INC_ENTRYPOINTCPP_HPP_ */

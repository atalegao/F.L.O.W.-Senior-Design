/*
 * entryPointCPP.hpp
 *
 *  Created on: Jan 26, 2025
 *      Author: johng
 */

#ifndef INC_ENTRYPOINTCPP_HPP_
#define INC_ENTRYPOINTCPP_HPP_


#include "main.h"

// Define all C function calls that will be called from main.c in the following extern "C" group
// Using extern "C" stops name mangling and allows "C" code to call C++ code

#ifdef __cplusplus
extern "C" {
#endif
	void initTest(SPI_HandleTypeDef *halSPI);
	void testLCD(bool buildTest);
#ifdef __cplusplus
}
#endif


#endif /* INC_ENTRYPOINTCPP_HPP_ */

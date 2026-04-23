/*
 * ui_api.h
 *
 *  Created on: Apr 23, 2026
 *      Author: aditi
 */

#ifndef INC_UI_API_H_
#define INC_UI_API_H_
#ifndef UI_API_H
#define UI_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize _ system (call once at boot)
void UI_Init(void);

// Update node data from mesh / sensors
void UI_UpdateNode(uint16_t nodeId,
                   float water,
                   uint32_t timestamp,
                   float battery, bool status);

// Check flood condition (e.g. 60% nodes above threshold)
void UI_CheckFlood(void);

// Optional: trigger alert manually
void UI_SendFloodAlert(uint16_t nodeId, uint32_t timestamp);

#ifdef __cplusplus
}
#endif

#endif




#endif /* INC_UI_API_H_ */

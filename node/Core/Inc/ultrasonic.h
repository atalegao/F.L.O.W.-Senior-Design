#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

// Enum for system states
typedef enum {
    STATE_IDLE,
    STATE_SAMPLING,
    STATE_REPORTING
} SystemState_t;

// Extern declarations for variables (defined in ultrasonic.c)
extern SystemState_t current_state;
extern uint32_t state_timer;
extern uint32_t trigger_timer;
extern volatile uint8_t rx_data [1];
extern uint8_t packet[4];
extern uint8_t packet_idx;
extern uint32_t readings[20];  // FILTER_SIZE = 20
extern uint8_t read_idx;
extern uint32_t distance_mm;
extern uint64_t long_term_sum;
extern uint32_t sample_count;
extern volatile uint8_t data_ready;
extern const uint32_t BREAK_TIME;
extern const uint32_t ON_TIME;
extern const uint32_t TRIGGER_RATE;

// Function prototypes
void ultrasonic_init(void);              // Initialize ultrasonic variables
uint32_t ultrasonic_update(void);            // Handle state machine and updates
void ultrasonic_process_rx(uint8_t data); // Process received UART data
void trigger_sensor_reading(void);       // Trigger a sensor reading

#endif // ULTRASONIC_H

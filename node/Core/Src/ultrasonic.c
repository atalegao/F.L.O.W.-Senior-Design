#include "ultrasonic.h"
#include "main.h"  // For HAL functions and GPIO

// Variable definitions
SystemState_t current_state = STATE_IDLE;
uint32_t state_timer = 0;
uint32_t trigger_timer = 0;
//uint8_t rx_data [1];
uint8_t packet[4];
uint8_t packet_idx = 0;
#define FILTER_SIZE 20
uint32_t readings[FILTER_SIZE] = {0};
uint8_t read_idx = 0;
uint32_t distance_mm = 0;
uint64_t long_term_sum = 0;
uint32_t sample_count = 0;
volatile uint8_t data_ready = 0;
const uint32_t BREAK_TIME = 1 * 60 * 1000; // Ten minutes idle
const uint32_t ON_TIME = 1 * 60 * 1000;    // One minute sampling
const uint32_t TRIGGER_RATE = 200;         // Polling freq, 5 times per sec during polling

// Initialize ultrasonic variables (call after HAL inits in main.c)
void ultrasonic_init(void) {
    current_state = STATE_IDLE;
    state_timer = 0;
    trigger_timer = 0;
    packet_idx = 0;
    read_idx = 0;
    distance_mm = 0;
    long_term_sum = 0;
    sample_count = 0;
    data_ready = 0;
}

// Handle state machine and periodic updates (call in main loop)
void ultrasonic_update(void) {
    uint32_t now = HAL_GetTick();

    switch (current_state) {
        case STATE_IDLE:
            if (now - state_timer >= BREAK_TIME) { // Ten minutes
                state_timer = now;
                long_term_sum = 0;
                sample_count = 0;
                current_state = STATE_SAMPLING;
                printf("Starting Sample \r\n");
            }
            break;

        case STATE_SAMPLING:
            // Trigger reading at specific rate
            if (now - trigger_timer >= TRIGGER_RATE) {
                trigger_sensor_reading();
                trigger_timer = now;
            }
            // Optional real-time printing during minute poll
            if (data_ready) {
                printf("current distance reading: %lu mm\r\n", distance_mm);
                data_ready = 0;
            }
            // End sampling window
            if (now - state_timer >= ON_TIME) {
                current_state = STATE_REPORTING;
            }
            break;

        case STATE_REPORTING:
            if (sample_count > 0) {
                uint32_t final_report = (uint32_t)(long_term_sum / sample_count);
                printf("average reading from poll: %lu mm (from %lu samples)\r\n", final_report, sample_count);
            }
            state_timer = now;
            current_state = STATE_IDLE;
            printf("entering idle\r\n");
            break;
    }
}

// Process received UART data (call from HAL_UART_RxCpltCallback in main.c)
void ultrasonic_process_rx(uint8_t data) {
    rx_data[0] = data;
    // State Machine to align packet
    if (packet_idx == 0 && rx_data[0] != 0xFF) {
        // Wait for header byte
        packet_idx = 0;
    } else {
        packet[packet_idx++] = rx_data[0];
    }

    // Once 4 bytes
    if (packet_idx == 4) {
        // Sum the first three bytes and mask to 8 bits
        uint8_t sum = (packet[0] + packet[1] + packet[2]) & 0xFF;

        if (sum == packet[3]) {
            uint32_t raw_val = (packet[1] << 8) | packet[2];
            readings[read_idx] = raw_val;
            read_idx = (read_idx + 1) % FILTER_SIZE;
            uint32_t total = 0;
            for (int i = 0; i < FILTER_SIZE; i++) {
                total += readings[i];
            }
            distance_mm = total / FILTER_SIZE;

            if (current_state == STATE_SAMPLING) {
                long_term_sum += raw_val;
                sample_count++;
            }
            data_ready = 1; // Signal finish valid reading
        }
        packet_idx = 0; // Reset index
    }
}

// Trigger a sensor reading
void trigger_sensor_reading(void) {
    // Reset state for new packet
    packet_idx = 0;
    data_ready = 0;

    // Pull RX low to trigger
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(1);
    // Set back to high aka idle
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

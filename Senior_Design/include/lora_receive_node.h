
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//comment out one of the 2 below
#define USE_CUSTOM_SETTINGS
//#define USE_DEFAULT_SETTINGS

#ifdef USE_CUSTOM_SETTINGS
#define BANDWIDTH           RH_RF95_BW_125KHZ //values in RH_RF95.h
#define CODING_RATE         RH_RF95_CODING_RATE_4_8 //chart on page 24, values in RH_RF95.h
#define SPREADING_FACTOR    RH_RF95_SPREADING_FACTOR_4096CPS //chart on page 24, values in RH_RF95.h
#define CRC_ON              RH_RF95_RX_PAYLOAD_CRC_ON //values in RH_RF95.h, 0 if no CRC else RH_RF95_RX_PAYLOAD_CRC_ON for CRC

#endif

#ifdef USE_DEFAULT_SETTINGS
#define MODEM_CONFIG_CHOICE Bw125Cr45Sf128 //(info page 22 of datasheet) Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on
    //is preset combo for Bandwidth, coding rate, spreading factor, CRC on/off (not using this)
#endif


void lora_uart_init();
bool lora_init();
void lora_read_fifo_all(uint8_t* data, uint8_t length);
void set_mode_continuous_receive();
void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length);
void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length);
void lora_write_single(uint8_t reg, uint8_t value);
uint8_t lora_read_single(uint8_t reg);
uint8_t uart_read();
uint8_t uart_read_single_only();
void uart_write(uint8_t data);
bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear);
void setup_leds(void);
int main(void);

#include "stm32f0xx.h"
void internal_clock();

void nano_wait(unsigned int n);

//#include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>

#define RH_WRITE_MASK 0x80
#define PREAMBLE_LENGTH 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13
#define FIFOSIZE 16 //number of bytes in a message
#define ADDRTO 10 //address of message that should receive any sent message
#define ADDRFROM 10 //address of this node (should be same as ADDRTO)
#define HEADERID 0 //this is one of the lora headers, but don't know what it is
#define HEADERFLAGS 0 //this is one of the lora headers, but don't know what it is

#define MESSAGE_LENGTH 2 //length of the message without headers

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
uint8_t lora_read_fifo_single();
void set_mode_continuous_receive();
void set_mode_sleep();
void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length);
void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length);
void lora_write_single(uint8_t reg, uint8_t value);
uint8_t lora_read_single(uint8_t reg);
uint8_t uart_read();
void uart_write(uint8_t data);
bool lora_send(uint8_t* data, uint8_t length);
void setup_leds(void);
//int main(void);
void initb();
bool connected_test();
// Place the pins of the serial adapter into the breadboard. (red chip)
// Connect Rx (Pin 2) of the serial adapter to PC12 of the devboard.
// Connect Tx (Pin 3) of the serial adapter to PD2 of the devboard.
// Connect GND on the devboard to the breadboard’s ground rail (blue).
// Connect GND (Pin 6) of the serial adapter to the breadboard’s ground rail (blue).
// Connect CTS (Pin 5) of the serial adapter to the breadboard’s ground rail (blue).

#include "stm32f0xx.h"
void internal_clock();

void nano_wait(unsigned int n);

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>
#include <lora_receive_node.h>

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

#define SLEEP_CYCLE_TIME 5 //length of time to stay in sleep mode during receive cycle (unit is 1/10 of a second), so 5 = 1/2 second

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
void uart_init_for_print();
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
void check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear);
void lora_read_fifo_all(uint8_t* data, uint8_t length);
 void setup_leds(void);
void set_mode_cad(void);
bool cad_cyle(void);
void sleep_cycle(void);
bool continous_receive_for_cycle(uint8_t *rxdone,uint8_t *valid_header, uint8_t *crc_error, uint8_t *data);
void receive_cycle(void);
void data_processing(void);
uint8_t rxdone_global;
uint8_t valid_header_global;
uint8_t crc_error_global; 
uint8_t data_global [MESSAGE_LENGTH];
int main(void);
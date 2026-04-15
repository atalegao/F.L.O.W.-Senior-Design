#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>
#include <main.h>
#include "stm32l0xx_hal.h"

//#define RECEIVE_BOL //for receive mode
//#define SEND_BOL //for send mode
#define CAD_BOL //for cad cycle

#define RH_WRITE_MASK 0x80
#define PREAMBLE_LENGTH 0xF0 //was 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13
#define ADDRTO 0x10 //address of message that should receive any sent message
#define ADDRFROM 0x10 //address of this node (should be same as ADDRTO)
#define HEADERID 0 //this is one of the lora headers, but don't know what it is
#define HEADERFLAGS 0 //this is one of the lora headers, but don't know what it is


#define FIFOSIZE_RX 1 //number of bytes in a received message (always 1)
#define FIFOSIZE_TX_NORM 150 //max number of bytes in a sent message (generic reads and writes)
#define FIFOSIZE_TX_SEND 150 //max number of bytes in a sent message (not just one write, but an actual sent LoRa message)
#define FIFOSIZE_TX_REC 150 //max number of bytes in a messaage to read the FIFO buffer

#define LORA_SEND_TIME 2000 //this is the time in ms between each node data send, was 1000

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
bool connected_test(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void enable_tty_interrupt(void);
void enable_tty_interrupt_send(void);
bool lora_init();
void set_mode_sleep(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool lora_send(uint8_t* data, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void lora_read_fifo_all(uint8_t* data, uint8_t length, bool clear_header, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
uint8_t lora_read_fifo_single(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void set_mode_continuous_receive();
void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length, uint8_t message_type);
void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t send_type);
void lora_write_single(uint8_t reg, uint8_t value, uint8_t message_type);
uint8_t lora_read_single(uint8_t reg, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t message_type, bool re_enable);
void set_mode_standby(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
uint8_t uart_read();
uint8_t uart_read_single_only();
void uart_write_normal(uint8_t data);
void uart_write_rx(uint8_t data);
void uart_write_tx(uint8_t data);
bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void setup_leds(void);
void lora_dma_write_send(int length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t send_type, bool re_enable);
void set_mode_cad(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool cad_cycle(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void sleep_cycle(void);
void change_lora_timer_period(int cause, TIM_HandleTypeDef * htim);
void setup_lora_send_timer(TIM_HandleTypeDef * htim, uint32_t lora_send_time);
int main(void);

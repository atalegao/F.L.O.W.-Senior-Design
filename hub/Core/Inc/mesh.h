#ifndef MESH_H
#define MESH_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "stm32l0xx_hal.h"
//#include "ultrasonic.h"


#define MESH_MAX_NEIGHBORS      7
#define MESH_DEFAULT_HELLO_SEC  60            // 1 minute default at the moment
#define MESH_DEFAULT_TIMEOUT    (4 * MESH_DEFAULT_HELLO_SEC)
#define MESH_MAX_PACKET         64            // I need to look into what the actual mesh packet limit it, but I have put it as 64 for now

#define ADDR_LENGTH 2
#define BATTERY_LENGTH 1
#define WATER_LENGTH 1

#define MESH_MAX_MESSAGE_LENGTH 30 //the size of the largest packet

#define RESEND_THRESHOLD (30 * 1) //0.5 minute

#define DEAD_MESSAGE_THRESHOLD (60 * 1) //1 minute

typedef enum {
    MESH_MSG_DATA  = 1, //works, crc
    MESH_MSG_POLL  = 2, //works, crc
    MESH_MSG_ACK   = 3, //works, crc
    MESH_MSG_DEAD  = 4, //works, updated for crc
    MESH_MSG_HELLO = 5, //works, updated for crc
    MESH_MSG_ADD   = 6, //works, updated for crc
} mesh_msg_type;  // using this to enumerate the different message types. Currently, we have 6 types.

typedef struct {
	volatile uint8_t addr [ADDR_LENGTH];
	volatile uint8_t  battery [BATTERY_LENGTH * 2];
	volatile uint64_t last_seen; //seconds, minutes, hours, day, month, year
	volatile bool valid;
} mesh_neighbor;  // this struct to detail characteristics for each neighbor in the table.



typedef struct{
	volatile uint8_t message_id [4];
	volatile bool valid; //1 if an actual message, 0 if just an initialized default message_id
} message_id_history;

typedef struct __attribute__((packed)){
	volatile uint8_t message [MESH_MAX_MESSAGE_LENGTH];
	volatile uint8_t length;
	volatile uint8_t attempt;
	volatile bool valid;
	volatile mesh_msg_type type;
} sending_buffer_entry;

typedef struct{
	volatile sending_buffer_entry entry;
	volatile time_t last_sent_time;

}sent_message_buff_entry;

typedef struct{
	volatile sent_message_buff_entry entry1;
	volatile sent_message_buff_entry entry2;
	volatile sent_message_buff_entry entry3;
	volatile sent_message_buff_entry entry4;
	volatile sent_message_buff_entry entry5;
	volatile sent_message_buff_entry entry6;
	volatile sent_message_buff_entry entry7;
	volatile sent_message_buff_entry entry8;
	volatile sent_message_buff_entry entry9;
	volatile sent_message_buff_entry entry10;
}sent_message_buffer;

typedef struct{
	volatile sending_buffer_entry entry1;
	volatile sending_buffer_entry entry2;
	volatile sending_buffer_entry entry3;
	volatile sending_buffer_entry entry4;
	volatile sending_buffer_entry entry5;
	volatile sending_buffer_entry entry6;
	volatile sending_buffer_entry entry7;
	volatile sending_buffer_entry entry8;
	volatile sending_buffer_entry entry9;
	volatile sending_buffer_entry entry10;
}sending_buffer_type;

void mesh_init();  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics.
void mesh_set_hello_interval(uint32_t seconds);
uint32_t random_number_gen(void);
time_t get_time_in_seconds(volatile RTC_TimeTypeDef *time, volatile RTC_DateTypeDef *date);
bool send_item_off_send_buffer(void);
void handle_node_dead_send(mesh_neighbor neighbor);
void convert_time_t_to_dead_since(time_t time, uint8_t *dead_since);
void handle_send_hello(void);
bool calc_crc(volatile uint8_t * data, uint8_t length);
void check_node_deads(time_t current_time_s);
bool add_new_neighbor_node(uint8_t * sending_addr, uint8_t * battery);
bool update_neighbor_nodes(uint8_t * sending_addr, uint8_t * battery);
void clear_sending_buffer(uint8_t * message_id, volatile sending_buffer_entry * buffer);
void replace_neighbor_node(volatile mesh_neighbor * replaced,volatile mesh_neighbor * replacing);
bool check_and_handle_neighbor_match(uint8_t * addr, uint8_t * battery, volatile mesh_neighbor * neighbor);
volatile sending_buffer_entry * get_sending_buffer_entry(uint8_t index);
bool add_one_send_to_sending_buffer(sending_buffer_entry new_entry);
void handle_one_resending(time_t current_time, volatile sent_message_buff_entry * sent_message);
//void add_to_sent_message_ids(uint8_t * message_id, uint8_t attempt);
bool check_message_id_sent(sent_message_buff_entry entry,uint8_t * message_id);
void handle_resending(DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
sending_buffer_entry make_sending_buffer_entry(uint8_t * message, uint8_t attempt, uint8_t length, mesh_msg_type type);
bool decide_if_past_time(time_t current_time, time_t stored_time);
bool check_addr_farther_from_hub(volatile uint8_t * first_addr,volatile uint8_t * second_addr);
bool check_addr_closer_to_hub(volatile uint8_t * first_addr,volatile uint8_t * second_addr);
bool check_addr_correct_dir(uint8_t * sending_addr, volatile mesh_msg_type * type);
bool check_addr_any_dir(uint8_t * sending_addr, volatile mesh_msg_type * type);
void find_dest_addr_to_hub(uint8_t * dest_addr, uint8_t attempt);
void find_dest_addr_away_hub(uint8_t * dest_addr, uint8_t attempt);
void set_self_addr(uint8_t * addr);
void define_addr_any_direction();
void define_addr_right_direction();
bool mesh_send_hello(uint8_t * battery, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_hello(volatile uint8_t * data, uint8_t * message_id, uint8_t * sending_addr, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_send_dead(uint8_t * dest_addr, volatile uint8_t * dead_addr, uint8_t * dead_since, uint8_t * battery, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_dead(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_send_add(uint8_t * dest_addr,uint8_t * new_addr,uint8_t * coords, uint8_t * distance, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_add(volatile uint8_t * data, uint8_t * message_id, uint8_t * send_addr, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_poll(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_ack(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
int mesh_send_add_header(uint8_t *message, uint8_t * message_id, uint8_t * dest_addr, mesh_msg_type type);
bool mesh_send_data(uint8_t * message_id, uint8_t * dest_addr, uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, uint8_t * time, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
uint8_t check_message_id(message_id_history past_message, uint8_t * message_id);
//void message_id_init_sent(message_id_history_sent message);
void replace_one_sent_buffer_entry(volatile sending_buffer_entry * new_entry, volatile sent_message_buff_entry * old_entry);
void add_to_sent_message_buffer(volatile sending_buffer_entry * entry);
void message_id_struct_init();
void check_message_id_all(uint8_t * message_id, bool * match);
void replace_one_message_id_struct(message_id_history changing, message_id_history values);
//void replace_one_message_id_struct_sent(message_id_history_sent changing, message_id_history_sent values);
void clear_sent_message_buffer(uint8_t * message_id);
bool sent_message_buffer_clear(uint8_t * message_id, volatile sent_message_buff_entry * entry);
void handle_sending_own_data(void);
void shift_all_messages(uint8_t * message_id, bool this_node_sent);
bool check_message_struct_match(uint8_t * message_id, volatile uint8_t * message_id2);
void clear_sent_message_struct(uint8_t * message_id);
bool mesh_handle_id_and_message_type(volatile uint8_t * z,volatile uint8_t * data, volatile mesh_msg_type * type, uint8_t * message_id);
bool mesh_message_type_helper(volatile uint8_t * data, mesh_msg_type type, uint8_t * message_id, uint8_t * sending_addr,DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_main_rec(volatile uint8_t * data,DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart);
bool mesh_rec_data(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef *huart);
void send_usb_ttl_message(bool sent, mesh_msg_type type, uint8_t * message_id, uint8_t time_or_ignore_reason, uint8_t * send_or_rec_addr, UART_HandleTypeDef *huart);

#endif

#ifndef MESH_H
#define MESH_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "stm32l0xx_hal.h"


#define MESH_MAX_NEIGHBORS      7
#define MESH_DEFAULT_HELLO_SEC  60            // 1 minute default at the moment
#define MESH_DEFAULT_TIMEOUT    (4 * MESH_DEFAULT_HELLO_SEC)
#define MESH_MAX_PACKET         64            // I need to look into what the actual mesh packet limit it, but I have put it as 64 for now

#define ADDR_LENGTH 2
#define BATTERY_LENGTH 1
#define WATER_LENGTH 1

#define MESH_MAX_MESSAGE_LENGTH 30 //the size of the largest packet

#define RESEND_THRESHOLD (60 * 1) //1 minute

typedef enum {
    MESH_MSG_DATA  = 1, //works
    MESH_MSG_POLL  = 2, //works
    MESH_MSG_ACK   = 3, //works
    MESH_MSG_DEAD  = 4,
    MESH_MSG_HELLO = 5,
    MESH_MSG_ADD   = 6,
} mesh_msg_type;  // using this to enumerate the different message types. Currently, we have 6 types. 

typedef struct {
    uint8_t addr [ADDR_LENGTH];
    uint8_t  battery [BATTERY_LENGTH];
    uint64_t last_seen; //seconds, minutes, hours, day, month, year
    bool valid;
} mesh_neighbor;  // this struct to detail characteristics for each neighbor in the table.  



typedef struct{
    bool this_node_sent; //true if this node is one that originally sent the message
    uint8_t message_id [4];
    bool valid; //1 if an actual message, 0 if just an initialized default message_id
} message_id_history;

typedef struct{
	uint8_t message [20]; //max length, not the actual length
	uint8_t attempt;
	bool valid;
	uint8_t message_id [4];
	time_t last_sent_time;
	uint8_t length; //actual length of message, depends on message type
}sent_message_buff_entry;

typedef struct{
	sent_message_buff_entry entry1;
	sent_message_buff_entry entry2;
	sent_message_buff_entry entry3;
	sent_message_buff_entry entry4;
	sent_message_buff_entry entry5;
	sent_message_buff_entry entry6;
	sent_message_buff_entry entry7;
	sent_message_buff_entry entry8;
	sent_message_buff_entry entry9;
	sent_message_buff_entry entry10;
}sent_message_buffer;

typedef struct{
	uint8_t message [20];
	uint8_t length;
	uint8_t attempt;
	bool valid;
} sending_buffer_entry;

typedef struct{
	sending_buffer_entry entry1;
	sending_buffer_entry entry2;
	sending_buffer_entry entry3;
	sending_buffer_entry entry4;
	sending_buffer_entry entry5;
	sending_buffer_entry entry6;
	sending_buffer_entry entry7;
	sending_buffer_entry entry8;
	sending_buffer_entry entry9;
	sending_buffer_entry entry10;
}sending_buffer_type;

void mesh_init();  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics.
void mesh_set_hello_interval(uint32_t seconds);
uint32_t random_number_gen(void);
time_t get_time_in_seconds(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
sending_buffer_entry get_sending_buffer_entry(uint8_t index);
void add_one_resend_to_send_buffer(sent_message_buff_entry sent_message);
void handle_one_resending(time_t current_time, sent_message_buff_entry sent_message);
bool decide_if_past_time(time_t current_time, time_t stored_time);
bool check_addr_closer_to_hub(uint8_t * first_addr,uint8_t * second_addr);
bool check_addr_farther_from_hub(uint8_t * first_addr,uint8_t * second_addr);
bool check_addr_correct_dir(uint8_t * sending_addr, mesh_msg_type * type);
bool check_addr_any_dir(uint8_t * sending_addr, mesh_msg_type * type);
void find_dest_addr_to_hub(uint8_t * dest_addr, uint8_t attempt);
void find_dest_addr_away_hub(uint8_t * dest_addr, uint8_t attempt);
void set_self_addr(uint8_t * addr);
void define_addr_any_direction();
void define_addr_right_direction();
bool mesh_send_hello(uint8_t * battery, uint8_t * sending_addr, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_hello(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_hello(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_dead(uint8_t * dest_addr, uint8_t * dead_addr, uint8_t * dead_since, uint8_t * battery, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_dead(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_add(uint8_t * dest_addr,uint8_t * new_addr,uint8_t * coords, uint8_t * distance, uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_add(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_poll(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_ack(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
int mesh_send_add_header(uint8_t *message, uint8_t * message_id, uint8_t * dest_addr, mesh_msg_type type);
bool mesh_send_data(uint8_t * message_id, uint8_t * dest_addr, uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, uint8_t * time, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void message_id_init(message_id_history message);
void message_id_struct_init();
void check_message_id_all(uint8_t * message_id, bool * match);
void replace_one_message_id_struct(message_id_history changing, message_id_history values);
void shift_all_messages(uint8_t * message_id, bool this_node_sent);
bool check_message_struct_match(uint8_t * message_id, uint8_t * message_id2);
void clear_sent_message_struct(uint8_t * message_id);
bool mesh_handle_id_and_message_type(mesh_msg_type * type, uint8_t * message_id);
bool mesh_message_type_helper(mesh_msg_type type, uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_main_rec(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_data(uint8_t *message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
void send_usb_ttl_message(bool sent, mesh_msg_type type, uint8_t * message_id, uint8_t time_or_ignore_reason, uint8_t * send_or_rec_addr, UART_HandleTypeDef huart);

#endif 

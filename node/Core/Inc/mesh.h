#ifndef MESH_H
#define MESH_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32l0xx_hal.h"


#define MESH_MAX_NEIGHBORS      7
#define MESH_DEFAULT_HELLO_SEC  60            // 1 minute default at the moment
#define MESH_DEFAULT_TIMEOUT    (4 * MESH_DEFAULT_HELLO_SEC)
#define MESH_MAX_PACKET         64            // I need to look into what the actual mesh packet limit it, but I have put it as 64 for now

#define ADDR_LENGTH 2
#define BATTERY_LENGTH 1
#define WATER_LENGTH 1

typedef enum {
    MESH_MSG_DATA  = 1,
    MESH_MSG_POLL  = 2,
    MESH_MSG_ACK   = 3,
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

mesh_neighbor neighbor_to_hub1, neighbor_to_hub2, neighbor_to_hub3, neighbor_away_hub1, neighbor_away_hub2, neighbor_away_hub3;
//to hub is closer to hub, away is farther from, 1 is closest to node, 3 is farthest, populates 1->3

typedef struct{
    bool this_node_sent; //true if this node is one that originally sent the message
    uint8_t message_id [4];
    bool valid; //1 if an actual message, 0 if just an initialized default message_id
} message_id_history;

message_id_history message1, message2, message3, message4, message5, message6, message7, message8, message9, message10;
message_id_history sent_message1, sent_message2, sent_message3, sent_message4, sent_message5;

void mesh_init(bool isHub, uint8_t ownAddress);  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics. 
void mesh_set_hello_interval(uint32_t seconds);
uint32_t random_number_gen(void);
bool check_addr_closer_to_hub(uint8_t * first_addr,uint8_t * second_addr);
bool check_addr_farther_from_hub(uint8_t * first_addr,uint8_t * second_addr);
bool check_addr_correct_dir(uint8_t * sending_addr, mesh_msg_type * type);
bool check_addr_any_dir(uint8_t * sending_addr, mesh_msg_type * type);
void find_dest_addr_to_hub(uint8_t * dest_addr, uint8_t attempt);
void find_dest_addr_away_hub(uint8_t * dest_addr, uint8_t attempt);
void set_self_addr(uint8_t * addr);
void define_addr_any_direction(uint8_t * addr);
void define_addr_right_direction(uint8_t * addr);
bool mesh_send_hello(uint8_t * battery, uint8_t * sending_addr, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_hello(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_hello(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_dead(uint8_t * dest_addr, uint8_t * dead_addr, uint8_t * dead_since, uint8_t * battery, uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_dead(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_add(uint8_t * dest_addr,uint8_t * new_addr,uint8_t * coords, uint8_t * distance, uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_add(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_poll(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_rec_ack(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart);
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
void send_usb_ttl_message(bool sent, mesh_msg_type type, uint8_t * message_id, uint8_t time_or_ignore_reason, UART_HandleTypeDef huart);

#endif 

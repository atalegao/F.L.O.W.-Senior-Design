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
    uint8_t [ADDR_LENGTH] addr;
    uint8_t [BATTERY_LENGTH] battery;
    uint32_t last_seen;       
} mesh_neighbor;  // this struct to detail characteristics for each neighbor in the table.  

//typedef struct{
//    mesh_msg_type type;
//    uint8_t [ADDR_LENGTH] src;
//    uint8_t [ADDR_LENGTH] dest;
//    unit32_t msg_id;
//} mesh_header;

typedef struct{
    bool this_node_sent; //true if this node is one that originally sent the message
    uint8_t [4] message_id;
    bool valid; //1 if an actual message, 0 if just an initialized default message_id
} message_id_history;

message_id_history message1, message2, message3, message4, message5, message6, message7, message8, message9. message10;
message_id_history sent_message1, sent_message2, sent_message3, sent_message4, sent_message5;

void mesh_init(bool isHub, uint8_t ownAddress);  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics. 
void mesh_set_hello_interval(uint32_t seconds);   
bool mesh_send_hello(uint8_t battery);
bool mesh_send_data(uint8_t dest, const uint8_t *data, uint8_t len);
bool mesh_send_ack(uint8_t dest, uint8_t acked_msg_id);
bool mesh_send_dead(uint8_t dest, uint8_t dead_addr, uint32_t dead_since, uint8_t battery); 
bool mesh_send_poll(uint8_t dest, uint8_t new_frequency);
bool mesh_send_add(uint8_t dest,uint8_t new_addr,uint32_t coords, uint16_t distance);
#endif 

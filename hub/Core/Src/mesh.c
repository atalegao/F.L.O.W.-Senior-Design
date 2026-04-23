#include "main.h"
#include "mesh.h"
#include <string.h>
//#include "ui_def.h"

extern volatile RTC_TimeTypeDef current_time;
extern volatile RTC_DateTypeDef current_date;
extern bool isHub;
extern volatile bool usb_ttl_done;
extern volatile uint8_t last_sent_msg_id [4];
extern volatile uint8_t node_distance [2];

extern volatile uint8_t rx_data [1];

//extern ADC_HandleTypeDef hadc;

extern volatile bool banned_addr1_valid;
extern volatile bool banned_addr2_valid;
extern volatile uint8_t banned_addr1 [ADDR_LENGTH];
extern volatile uint8_t banned_addr2 [ADDR_LENGTH];

extern uint8_t self_addr [ADDR_LENGTH];
extern uint8_t addr_any_direction [ADDR_LENGTH];
extern uint8_t addr_right_direction [ADDR_LENGTH];
extern RNG_HandleTypeDef hrng;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart2_tx;

const uint16_t ADC_MIN = 1861; // Corresponds to 3.0V battery
const uint16_t ADC_MAX = 2606;

extern volatile uint32_t adc_val;

volatile mesh_neighbor neighbor_to_hub1, neighbor_to_hub2, neighbor_to_hub3, neighbor_away_hub1, neighbor_away_hub2, neighbor_away_hub3;
//to hub is closer to hub, away is farther from, 1 is closest to node, 3 is farthest, populates 1->3

volatile message_id_history message1, message2, message3, message4, message5, message6, message7, message8, message9, message10;

volatile sent_message_buffer sent_buffer;

volatile sending_buffer_type sending_buffer;
uint8_t message_1 [64];

volatile uint8_t send_buffer_add_index = 1; //is where to add the next message (counts up)
volatile uint8_t send_buffer_send_index = 1; //is where the next message to be sent is (counts up)

void mesh_init(){
	define_addr_any_direction();
	define_addr_right_direction();
    if (isHub) {
        // Initialize as a hub
        // Set up necessary data structures for a hub
    } else {
        // Initialize as a node
        // Set up necessary data structures for a node
    }

}  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics.
//void mesh_set_hello_interval(uint32_t seconds){
//    interval =
//}


uint32_t random_number_gen(void){
	uint32_t random_number;
	if (HAL_RNG_GenerateRandomNumber(&hrng, &random_number) != HAL_OK) {
	    // Error handling
	    Error_Handler();
	}
	return random_number;
}

bool calc_crc(volatile uint8_t * data, uint8_t length){
	uint8_t  i = 0; //length is in bytes
	uint8_t sum = 0;
	while( i < length){
		int j = 0;
		while(j < 8){
			if(data[i] & (0x1 << j)){
				sum += 1;
			}
			j += 1;
		}
		i += 1;
	}
	return ((sum % 2) == 0);
}

bool calc_even_with_crc(uint8_t * data, uint8_t length, uint8_t crc_bit){
	bool even = calc_crc(data, length);
	bool out;
	if(crc_bit){
		out = !even;
	}
	else{
		out = even;
	}
	return out;
}
//void handle_sending_own_data(void){
//	uint8_t dest_addr[ADDR_LENGTH];
//	find_dest_addr_to_hub(dest_addr, 1);
//
//	uint32_t message_id;
//	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)
//	uint8_t message_id_actual [4];
//	memcpy(message_id_actual, &message_id, sizeof(uint32_t));
//
//	//TODO:figure out battery
//	uint8_t battery [BATTERY_LENGTH];
//	HAL_ADC_Start(&hadc);
//	if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
//		adc_val = HAL_ADC_GetValue(&hadc);
//	}
//	HAL_ADC_Stop(&hadc);
//
//	battery[0] = (uint8_t)(((uint32_t)(adc_val - ADC_MIN) * 255) / (ADC_MAX - ADC_MIN));
//	if (adc_val <= ADC_MIN){
//		battery[0] = 0;
//	}
//	if (adc_val >= ADC_MAX){
//		battery[0] = 255;
//	}
//
//	//TODO: figure out water height
//	uint8_t water_height [WATER_LENGTH];
//	uint32_t data = ultrasonic_update();
//	ultrasonic_init();
//	water_height[0] = (data) & 0xFF;
//
//	uint8_t time [6];
//	get_timestamp();//get current time
//	time_t current_time_s = get_time_in_seconds(&current_time, &current_date);
//	convert_time_t_to_dead_since(current_time_s, time);
//
//	mesh_send_data(message_id_actual, dest_addr, water_height, battery, self_addr, time, 1, &hdma_usart2_tx, &huart2);
//
//}

bool send_item_off_send_buffer(void){
	//send_buffer_send_index
	volatile sending_buffer_entry * entry = get_sending_buffer_entry(send_buffer_send_index);
	if(entry->valid == false){
		return false;
	}
	uint8_t id [4];
	uint8_t dest_addr [ADDR_LENGTH];
	int i = 0;
	while(i < ADDR_LENGTH){
		dest_addr[i] = entry->message[i];
		i += 1;
	}
	int j = 0;
	int k = i;
	while(i < k + 4){ //message_id
		id[j] = entry->message[i];
		i += 1;
		j += 1;
		}
	bool good;
	good = lora_send(entry->message, entry->length, &hdma_usart2_tx, &huart2);
	//send_usb_ttl_message(true, entry->type, id, entry->attempt, dest_addr, &huart1);
	send_buffer_send_index += 1;
	entry->valid = false;
	if(send_buffer_send_index > 10){
		send_buffer_send_index = 0;
	}
	//add to sent message buffer
	if((entry->type != MESH_MSG_ACK) & (entry->type != MESH_MSG_HELLO)){
		add_to_sent_message_buffer(entry);
	}
	return good;
}

void add_to_sent_message_buffer(volatile sending_buffer_entry * entry){
	//find empty buffer slot
	if(sent_buffer.entry1.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry1);
	}
	else if(sent_buffer.entry2.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry2);
	}
	else if(sent_buffer.entry3.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry3);
	}
	else if(sent_buffer.entry4.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry4);
	}
	else if(sent_buffer.entry5.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry5);
	}
	else if(sent_buffer.entry6.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry6);
	}
	else if(sent_buffer.entry7.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry7);
	}
	else if(sent_buffer.entry8.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry8);
	}
	else if(sent_buffer.entry9.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry9);
	}
	else if(sent_buffer.entry10.entry.valid == false){
		replace_one_sent_buffer_entry(entry, &sent_buffer.entry10);
	}
}

void replace_one_sent_buffer_entry(volatile sending_buffer_entry * new_entry, volatile sent_message_buff_entry * old_entry){
	old_entry->entry.valid = true;
	old_entry->entry.type = new_entry->type;
	old_entry->entry.attempt = new_entry->attempt;
	old_entry->entry.length = new_entry->length;
	int i = 0;
	while(i < MESH_MAX_MESSAGE_LENGTH){
		old_entry->entry.message[i] = new_entry->message[i];
		i += 1;
	}
	time_t current_time_s = get_time_in_seconds(&current_time, &current_date);
	old_entry->last_sent_time = current_time_s;
}
//
//void add_to_sent_message_ids(uint8_t * message_id, uint8_t attempt){
//	message_id_history_sent values;
//	values.attempt = attempt;
//	values.valid = true;
//	int i = 0;
//	while(i < 4){
//		values.message_id[i] = message_id[i];
//		i += 1;
//	}
//	if(sent_message1.valid == false){
//		replace_one_message_id_struct_sent(sent_message1, values);//replace with new message
//	}
//	else if(sent_message2.valid == false){
//		replace_one_message_id_struct_sent(sent_message2, values);//replace with new message
//	}
//	else if(sent_message3.valid == false){
//		replace_one_message_id_struct_sent(sent_message3, values);//replace with new message
//	}
//	else if(sent_message4.valid == false){
//		replace_one_message_id_struct_sent(sent_message4, values);//replace with new message
//	}
//	else if(sent_message5.valid == false){
//		replace_one_message_id_struct_sent(sent_message5, values);//replace with new message
//	}
//	else{
//
//	}
//}

//TODO: handle message ID
//TODO: check resending
//TODO: check crc
//TODO: stop sends after too many attempts

void handle_send_hello(void){
	//TODO:figure out battery
	uint8_t battery [BATTERY_LENGTH];
	mesh_send_hello(battery, &hdma_usart2_tx, &huart2);
}

//use the buffer for sent messages and have a timer to add a sent message to the send buffer if the message is still in the sent buffer and is still valid
//TODO: need a timer to resend a message that was not received, the timer calls the below function
void handle_resending(DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//go through each sent message buffer and checks if it is valid and if its time is past the current time plus a threshold

	get_timestamp();//get current time
	time_t current_time_s = get_time_in_seconds(&current_time, &current_date);

	handle_one_resending(current_time_s, &sent_buffer.entry1);
	handle_one_resending(current_time_s, &sent_buffer.entry2);
	handle_one_resending(current_time_s, &sent_buffer.entry3);
	handle_one_resending(current_time_s, &sent_buffer.entry4);
	handle_one_resending(current_time_s, &sent_buffer.entry5);
	handle_one_resending(current_time_s, &sent_buffer.entry6);
	handle_one_resending(current_time_s, &sent_buffer.entry7);
	handle_one_resending(current_time_s, &sent_buffer.entry8);
	handle_one_resending(current_time_s, &sent_buffer.entry9);
	handle_one_resending(current_time_s, &sent_buffer.entry10);

	//also check if any node_dead messages need to be sent
	check_node_deads(current_time_s);
}

void check_node_deads(time_t current_time_s){
	//goes through each neighbor node and figures out if it is past the time to send a node dead message for it
	double diff_seconds = difftime(current_time_s, neighbor_to_hub1.last_seen);
	if((diff_seconds > DEAD_MESSAGE_THRESHOLD) & neighbor_to_hub1.valid){
		handle_node_dead_send(neighbor_to_hub1);//send node dead message for it
	}
	double diff_seconds2 = difftime(current_time_s, neighbor_to_hub2.last_seen);
	if((diff_seconds2 > DEAD_MESSAGE_THRESHOLD) & neighbor_to_hub2.valid){
		handle_node_dead_send(neighbor_to_hub2);//send node dead message for it
	}
	double diff_seconds3 = difftime(current_time_s, neighbor_to_hub3.last_seen);
	if((diff_seconds3 > DEAD_MESSAGE_THRESHOLD) & neighbor_to_hub3.valid){
		handle_node_dead_send(neighbor_to_hub3);//send node dead message for it
	}
	double diff_seconds4 = difftime(current_time_s, neighbor_away_hub1.last_seen);
	if((diff_seconds4 > DEAD_MESSAGE_THRESHOLD) & neighbor_away_hub1.valid){
		handle_node_dead_send(neighbor_away_hub1);//send node dead message for it
	}
	double diff_seconds5 = difftime(current_time_s, neighbor_away_hub2.last_seen);
	if((diff_seconds5 > DEAD_MESSAGE_THRESHOLD) & neighbor_away_hub2.valid){
		handle_node_dead_send(neighbor_away_hub2);//send node dead message for it
	}
	double diff_seconds6 = difftime(current_time_s, neighbor_away_hub3.last_seen);
	if((diff_seconds6 > DEAD_MESSAGE_THRESHOLD) & neighbor_away_hub3.valid){
		handle_node_dead_send(neighbor_away_hub3);//send node dead message for it
	}
}

void convert_time_t_to_dead_since(time_t time, uint8_t *dead_since) {
    struct tm *timeinfo = gmtime(&time);

    // 2. Map structure members to your 6-byte array
    dead_since[0] = (uint8_t)timeinfo->tm_min;          // Minutes
    dead_since[1] = (uint8_t)timeinfo->tm_sec;          // Seconds
    dead_since[2] = (uint8_t)timeinfo->tm_hour;         // Hours (0-23)
    dead_since[3] = (uint8_t)timeinfo->tm_mday;         // Day
    dead_since[4] = (uint8_t)(timeinfo->tm_mon + 1);    // Month
    dead_since[5] = (uint8_t)(timeinfo->tm_year % 100); // Year
}

void handle_node_dead_send(mesh_neighbor neighbor){
	uint8_t dest_addr[ADDR_LENGTH];
	find_dest_addr_to_hub(dest_addr, 1);

	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)
	uint8_t message_id_actual [4];
	memcpy(message_id_actual, &message_id, sizeof(uint32_t));
	//TODO:figure out battery
	uint8_t battery [BATTERY_LENGTH];

	uint8_t dead_since [6];
	convert_time_t_to_dead_since(neighbor.last_seen, dead_since);

	mesh_send_dead(dest_addr, neighbor.addr, dead_since, battery,  message_id_actual, 1, &hdma_usart2_tx,  &huart2);
}

void handle_one_resending(time_t current_time, volatile sent_message_buff_entry * sent_message){
	if(sent_message->entry.valid == false){
		return;
	}

	bool past_time = decide_if_past_time(current_time, sent_message->last_sent_time);
	if(past_time){
		if(sent_message->entry.attempt > 8){
			return;
		}
		sent_message->entry.attempt = sent_message->entry.attempt + 1;
		//update addr
		uint8_t dest_addr [ADDR_LENGTH];
		if(sent_message->entry.type == MESH_MSG_POLL){//only one that goes away from hub (besides ack, but that doesn't count)
			find_dest_addr_away_hub(dest_addr, sent_message->entry.attempt);
		}
		else{
			find_dest_addr_to_hub(dest_addr, sent_message->entry.attempt);
		}
		int i = 0;
		while(i < ADDR_LENGTH){
			sent_message->entry.message[i] = dest_addr[i]; //since dest addr is always the first bytes
			i += 1;
		}
		add_one_send_to_sending_buffer(sent_message->entry);
		sent_message->entry.valid = false;
	}
}

bool add_one_send_to_sending_buffer(sending_buffer_entry new_entry){
	//uint8_t send_buffer_add_index; //is where to add the next message (counts up)
	//uint8_t send_buffer_send_index;

	//check if add_index is free
	volatile sending_buffer_entry * sending_buffer_at_add_index = get_sending_buffer_entry(send_buffer_add_index);
	if(sending_buffer_at_add_index->valid == true){
		//issue, buffer overflowed
		return false;
	}
	else{
		sending_buffer_at_add_index->valid = true;
		int i = 0;
		while (i < MESH_MAX_MESSAGE_LENGTH){
			sending_buffer_at_add_index->message[i] = new_entry.message[i];
			i += 1;
		}
		sending_buffer_at_add_index->length = new_entry.length;
		sending_buffer_at_add_index->attempt = new_entry.attempt;
		sending_buffer_at_add_index->type = new_entry.type;


		send_buffer_add_index += 1;
		if(send_buffer_add_index > 10){
			send_buffer_add_index = 0;
		}
		return true;
	}
	return false;
}

volatile sending_buffer_entry * get_sending_buffer_entry(uint8_t index){
	switch(index){
		case 10:
			return &sending_buffer.entry10;
			break;
		case 9:
			return &sending_buffer.entry9;
			break;
		case 8:
			return &sending_buffer.entry8;
			break;
		case 7:
			return &sending_buffer.entry7;
			break;
		case 6:
			return &sending_buffer.entry6;
			break;
		case 5:
			return &sending_buffer.entry5;
			break;
		case 4:
			return &sending_buffer.entry4;
			break;
		case 3:
			return &sending_buffer.entry3;
			break;
		case 2:
			return &sending_buffer.entry2;
			break;
		case 1:
			return &sending_buffer.entry1;
			break;
		}
	return &sending_buffer.entry10;
}

bool decide_if_past_time(time_t current_time, time_t stored_time){
	double diff_seconds = difftime(current_time, stored_time);
	if(diff_seconds > RESEND_THRESHOLD){
		return true;
	}
	else{
		return false;
	}
}
time_t get_time_in_seconds(volatile RTC_TimeTypeDef *time, volatile RTC_DateTypeDef *date){
	//gets time in seconds since epoch
	struct tm tim = {0};
	tim.tm_sec = time->Seconds;
	tim.tm_min = time->Minutes;
	tim.tm_hour = time->Hours;
	tim.tm_mday = date->Date;
	tim.tm_mon = date->Month - 1;
	tim.tm_year = date->Year + 100;
	tim.tm_isdst = -1;
	return mktime(&tim);
}

void init_one_neighbor(mesh_neighbor node){
	node.addr[0] = 0;
	node.addr[1] = 0;
	node.battery[0] = 0;
	node.last_seen = 0;
	node.valid = false;
}
// 1 is closest to this node, 3 is farthest from this node
void init_neighbors(){
	init_one_neighbor(neighbor_to_hub1);
	init_one_neighbor(neighbor_to_hub2);
	init_one_neighbor(neighbor_to_hub3);
	init_one_neighbor(neighbor_away_hub1);
	init_one_neighbor(neighbor_away_hub2);
	init_one_neighbor(neighbor_away_hub3);
}

bool check_addr_closer_to_hub(volatile uint8_t * first_addr,volatile uint8_t * second_addr){
	//returns true if the first addr is closer to the hub than the second addr
	uint16_t first = (uint16_t)(first_addr[1] << 8) | first_addr[0];
	uint16_t second = (uint16_t)(second_addr[1] << 8) | second_addr[0];
	if(abs(first) > abs(second)){
		return false;
	}
	else{
		return true;
	}
}

bool check_addr_farther_from_hub(volatile uint8_t * first_addr,volatile uint8_t * second_addr){
	//returns true if the first addr is farther from the hub than the second addr
	uint16_t first = (uint16_t)(first_addr[1] << 8) | first_addr[0];
	uint16_t second = (uint16_t)(second_addr[1] << 8) | second_addr[0];
	if(abs(first) < abs(second)){
			return false;
	}
	else{
		return true;
	}
}

bool check_addr_correct_dir(uint8_t * sending_addr, volatile mesh_msg_type * type){
	//check if addr is in the correct direction, if so return true, else false
	if((type[0] == MESH_MSG_POLL) | (type[0] == MESH_MSG_ACK)){//away from hub
		return check_addr_farther_from_hub(self_addr, sending_addr);
	}
	else{//towards hub: data, dead, add
		return check_addr_closer_to_hub(self_addr, sending_addr);
	}
    //hello is neither
}

bool check_addr_any_dir(uint8_t * sending_addr, volatile mesh_msg_type * type){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//check if this node knows any nodes in the direction closer to the hub than the sender's addr
	//or father from hub than sender's addr

	bool cor_dir = check_addr_correct_dir(sending_addr, type);//check correct direction first
	if(cor_dir){
		return true;
	}
	//else check if this node knows and node farther in the correct direction than the sending addr
	bool match;
	if((type[0] == MESH_MSG_POLL) | (type[0] == MESH_MSG_ACK)){//away from hub
		match = (check_addr_farther_from_hub(neighbor_away_hub3.addr, sending_addr) & neighbor_away_hub3.valid);
		return (match);
	}
	else{//towards hub: data, dead, add
		match = (check_addr_closer_to_hub(neighbor_to_hub1.addr, sending_addr) & neighbor_to_hub1.valid);
		return (match);
	}
	//hello is neither

}
//to hub is closer to hub, away is farther from, 1 is closest to node, 3 is farthest, populates 1->3


void find_dest_addr_to_hub(uint8_t * dest_addr, uint8_t attempt){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//finds dest addr for a message going towards the hub
	//looks in neighbor node structs
	int sum = neighbor_to_hub1.valid + neighbor_to_hub2.valid + neighbor_to_hub3.valid;
	if (sum == 0){
		if(attempt < 7){
			dest_addr[0] = addr_right_direction[0];
			dest_addr[1] = addr_right_direction[1];
			return;
		}
		else{
			dest_addr[0] = addr_any_direction[0];
			dest_addr[1] = addr_any_direction[1];
			return;
		}
	}

	if((attempt == 1) | (attempt == 2)){ //farthest towards hub
		switch(sum){
		case 3:
			dest_addr[0] = neighbor_to_hub3.addr[0];
			dest_addr[1] = neighbor_to_hub3.addr[1];
			break;
		case 2:
			dest_addr[0] = neighbor_to_hub2.addr[0];
			dest_addr[1] = neighbor_to_hub2.addr[1];
			break;
		case 1:
			dest_addr[0] = neighbor_to_hub1.addr[0];
			dest_addr[1] = neighbor_to_hub1.addr[1];
			break;
		}
	}
	else if((attempt == 3) | (attempt == 4)){//2nd farthest towards hub
		switch(sum){
			case 3:
				dest_addr[0] = neighbor_to_hub2.addr[0];
				dest_addr[1] = neighbor_to_hub2.addr[1];
				break;
			case 2:
				dest_addr[0] = neighbor_to_hub2.addr[0];
				dest_addr[1] = neighbor_to_hub2.addr[1];
				break;
			case 1:
				dest_addr[0] = neighbor_to_hub1.addr[0];
				dest_addr[1] = neighbor_to_hub1.addr[1];
				break;
				}
	}
	else if((attempt == 5) | (attempt == 6)){//3rd farthest towards hub
		dest_addr[0] = neighbor_to_hub1.addr[0];
		dest_addr[1] = neighbor_to_hub1.addr[1];
	}
	else if((attempt == 7) | (attempt == 8)){//correct direction (towards hub)
		dest_addr[0] = addr_right_direction[0];
		dest_addr[1] = addr_right_direction[1];
	}
	else{ //any direction
		dest_addr[0] = addr_any_direction[0];
		dest_addr[1] = addr_any_direction[1];
	}
}

void find_dest_addr_away_hub(uint8_t * dest_addr, uint8_t attempt){
	//finds dest addr for a message going away from the hub
	//looks in neighbor node structs
	int sum = neighbor_away_hub1.valid + neighbor_away_hub2.valid + neighbor_away_hub3.valid;
	if (sum == 0){
		if(attempt < 7){
			dest_addr[0] = addr_right_direction[0];
			dest_addr[1] = addr_right_direction[1];
			return;
		}
		else{
			dest_addr[0] = addr_any_direction[0];
			dest_addr[1] = addr_any_direction[1];
			return;
		}
	}

	if((attempt == 1) | (attempt == 2)){ //farthest away from hub
		switch(sum){
		case 3:
			dest_addr[0] = neighbor_away_hub3.addr[0];
			dest_addr[1] = neighbor_away_hub3.addr[1];
			break;
		case 2:
			dest_addr[0] = neighbor_away_hub2.addr[0];
			dest_addr[1] = neighbor_away_hub2.addr[1];
			break;
		case 1:
			dest_addr[0] = neighbor_away_hub1.addr[0];
			dest_addr[1] = neighbor_away_hub1.addr[1];
			break;
		}
	}
	else if((attempt == 3) | (attempt == 4)){//2nd farthest towards hub
		switch(sum){
			case 3:
				dest_addr[0] = neighbor_away_hub2.addr[0];
				dest_addr[1] = neighbor_away_hub2.addr[1];
				break;
			case 2:
				dest_addr[0] = neighbor_away_hub2.addr[0];
				dest_addr[1] = neighbor_away_hub2.addr[1];
				break;
			case 1:
				dest_addr[0] = neighbor_away_hub1.addr[0];
				dest_addr[1] = neighbor_away_hub1.addr[1];
				break;
				}
	}
	else if((attempt == 5) | (attempt == 6)){//3rd farthest towards hub
		dest_addr[0] = neighbor_away_hub1.addr[0];
		dest_addr[1] = neighbor_away_hub1.addr[1];
	}
	else if((attempt == 7) | (attempt == 8)){//correct direction (towards hub)
		dest_addr[0] = addr_right_direction[0];
		dest_addr[1] = addr_right_direction[1];
	}
	else{ //any direction
		dest_addr[0] = addr_any_direction[0];
		dest_addr[1] = addr_any_direction[1];
	}
}

void set_self_addr(uint8_t * addr){
	//updates a global uint8_t * called self_addr
	int i = 0;
	while (i < ADDR_LENGTH){
		self_addr[i] = addr[i];
		i += 1;
	}
}

void define_addr_any_direction(){
	//define a special addr that means any direction
	addr_any_direction[0] = 0xFF;
	addr_any_direction[1] = 0x7F; //int max (signed)
}

void define_addr_right_direction(){
	//define a special addr that means correct direction
	addr_right_direction[0] = 0x00;
	addr_right_direction[1] = 0x80; //int min (signed)
}


sending_buffer_entry make_sending_buffer_entry(uint8_t * message, uint8_t attempt, uint8_t length, mesh_msg_type type){
	sending_buffer_entry entry;
	entry.valid = true;
	entry.length = length;
	entry.attempt = attempt;
	entry.type = type;
	int i = 0;
	while (i < MESH_MAX_MESSAGE_LENGTH){
		entry.message[i] = message[i];
		i += 1;
	}
	return entry;
}

bool mesh_send_hello(uint8_t * battery, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//message is dest_addr, message_id, message_type, sending_addr, battery

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH + 1]; //10 + 1 crc for just battery and sending addr

	//message ID is 32 bit random number
	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)
	uint8_t message_id_actual [4];
	memcpy(message_id_actual, &message_id, sizeof(uint32_t));

	uint8_t crc_byte = 0;
	bool even;

	//dest addr is any direction
	uint8_t dest_addr [2];
	dest_addr [0] = addr_any_direction[0];
	dest_addr [1] = addr_any_direction[1];

	int i = 0;

	i = mesh_send_add_header(message, message_id_actual, dest_addr, MESH_MSG_HELLO);
	even = calc_crc(&self_addr[0], 1);
	crc_byte |= (!even) << 0;
	even = calc_crc(&self_addr[1], 1);
	crc_byte |= (!even) << 1;

	int k = i;
	int j = 0;
	while(i < k+BATTERY_LENGTH){
		message[i] = battery[j];
		even = calc_crc(&battery[j], 1);
		crc_byte |= (!even) << (2 + j);
		i += 1;
		j += 1;
	}

	message[i] = crc_byte;

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, 1, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH + 1), MESH_MSG_HELLO);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH), hdma_usart_tx, huart);
	////send_usb_ttl_message(true, MESH_MSG_HELLO, message_id_actual, 1, dest_addr, huart1);
	return good;
}

bool check_and_handle_neighbor_match(uint8_t * addr, uint8_t * battery, volatile mesh_neighbor * neighbor){
	int i = 0;
	while(i < ADDR_LENGTH){
		if(addr[i] != neighbor->addr[i]){
			return false;
		}
		i += 1;
	}
	//got this far means a match to an existing addr, so just update battery, first battery is most recent, 2nd is least recent
	i = 0;
	//while(i < BATTERY_LENGTH){
	neighbor->battery[1] = neighbor->battery[0];
	//	i += 1;
	//}
	i = 0;
	while(i < BATTERY_LENGTH){
		neighbor->battery[i] = battery[i];
		i += 1;
	}
	//update last seen
	get_timestamp();//updates current time and date
	neighbor->last_seen = 0;
	neighbor->last_seen |= (uint64_t) current_time.Seconds;
	neighbor->last_seen |= (uint64_t) current_time.Minutes << (8*1);
	neighbor->last_seen |= (uint64_t) current_time.Hours << (8*2);
	neighbor->last_seen |= (uint64_t) current_date.Date << (8*3);
	neighbor->last_seen |= (uint64_t) current_date.Month << (8*4);
	neighbor->last_seen |= (uint64_t) current_date.Year << (8*5);
	return true;
}

void replace_neighbor_node(volatile mesh_neighbor * replaced,volatile mesh_neighbor * replacing){
	replaced->valid = replacing->valid;
	replaced->last_seen = replacing->last_seen;
	int i = 0;
	while(i < ADDR_LENGTH){
		replaced->addr[i] = replacing->addr[i];
		i += 1;
	}
	i = 0;
	while(i < BATTERY_LENGTH * 2){
		replaced->battery[i] = replacing->battery[i];
		i += 1;
	}
}

bool add_new_neighbor_node(uint8_t * sending_addr, uint8_t * battery){
	//figure out direction
	bool closer_to_hub = check_addr_closer_to_hub(sending_addr,self_addr);
	uint8_t addr1 [ADDR_LENGTH];
	uint8_t addr2 [ADDR_LENGTH];
	uint8_t addr3 [ADDR_LENGTH];
	bool addr1_valid;
	bool addr2_valid;
	bool addr3_valid;

	mesh_neighbor new[1]; //make new mesh_neighbor struct
	new->valid = true;
	int i = 0;
	while(i < ADDR_LENGTH){
		new->addr[i] = sending_addr[i];
		i += 1;
	}
	i = 0;
	while(i < BATTERY_LENGTH * 2){
		new->battery[i] = battery[i];
		i += 1;
	}
	get_timestamp();//updates current time and date
	new->last_seen = 0;
	new->last_seen |= (uint64_t) current_time.Seconds;
	new->last_seen |= (uint64_t) current_time.Minutes << (8*1);
	new->last_seen |= (uint64_t) current_time.Hours << (8*2);
	new->last_seen |= (uint64_t) current_date.Date << (8*3);
	new->last_seen |= (uint64_t) current_date.Month << (8*4);
	new->last_seen |= (uint64_t) current_date.Year << (8*5);//end make new mesh_neighbor struct

	i = 0;
	if(closer_to_hub){//handle if closer to hub
		while(i < ADDR_LENGTH){
			addr1[i] = neighbor_to_hub1.addr[i];
			i += 1;
		}
		addr1_valid = neighbor_to_hub1.valid;
		i = 0;
		while(i < ADDR_LENGTH){
			addr2[i] = neighbor_to_hub2.addr[i];
			i += 1;
		}
		addr2_valid = neighbor_to_hub2.valid;
		i = 0;
		while(i < ADDR_LENGTH){
			addr3[i] = neighbor_to_hub3.addr[i];
			i += 1;
		}
		addr3_valid = neighbor_to_hub3.valid;
		uint8_t num_valid = addr3_valid + addr1_valid + addr2_valid;
		if(num_valid == 0){//replace 1
			replace_neighbor_node(&neighbor_to_hub1,new);
			return true;
		}
		else if(num_valid == 1){//figure out whether to place at 1 or 2
			if(check_addr_closer_to_hub(new->addr, addr1)){//if new is closer to hub than neighbor_to_hub1, place at 2
				replace_neighbor_node(&neighbor_to_hub2,new);//place in 2nd spot
				return true;
			}//was not closer to hub (so closer to node), move 1st to 2nd
			replace_neighbor_node(&neighbor_to_hub2,&neighbor_to_hub1);//move 1st spot to 2nd
			replace_neighbor_node(&neighbor_to_hub1,new);//move new to 1st
			return true;

		}
		else if(num_valid == 2){//figure out whether to place at 1 or 2 or 3
			if(!check_addr_closer_to_hub(new->addr, addr1)){//if new is not closer to hub than neighbor_to_hub1, place at 1 and shift
				replace_neighbor_node(&neighbor_to_hub3,&neighbor_to_hub2);
				replace_neighbor_node(&neighbor_to_hub2,&neighbor_to_hub1);
				replace_neighbor_node(&neighbor_to_hub1,new);
				return true;
			}
			else if(!check_addr_closer_to_hub(new->addr, addr2)){//new is not closer to hub than 2, so shift and place at 2
				replace_neighbor_node(&neighbor_to_hub3,&neighbor_to_hub2);
				replace_neighbor_node(&neighbor_to_hub2,new);
				return true;
			}
			else{
				replace_neighbor_node(&neighbor_to_hub3,new);//place in 3rd spot
				return true;
			}
		}
		else{//all 3 valid
			if(check_addr_closer_to_hub(new->addr, addr3)){//if new is closer to hub than 3, so shift all and place at 3
				replace_neighbor_node(&neighbor_to_hub1,&neighbor_to_hub2);//move 2nd to 1st
				replace_neighbor_node(&neighbor_to_hub2,&neighbor_to_hub3);//move 3rd to 2nd
				replace_neighbor_node(&neighbor_to_hub3,new);//move new to 3
				return true;
			}
			else if(check_addr_closer_to_hub(new->addr, addr2)){//if new is closer to hub than 2, so shift and place at 2
				replace_neighbor_node(&neighbor_to_hub1,&neighbor_to_hub2);//move 2nd to 1st
				replace_neighbor_node(&neighbor_to_hub2,new);//move new to 2
				return true;
			}
			else if(check_addr_closer_to_hub(new->addr, addr1)){//if new is closer to hub than 1, place at 1
				replace_neighbor_node(&neighbor_to_hub1,new);//move new to 1
				return true;
			}
			else{//closest to node, so don't add
				return true;
			}
		}
	}
	else{//handle if farther from hub
		while(i < ADDR_LENGTH){
			addr1[i] = neighbor_away_hub1.addr[i];
			i += 1;
		}
		addr1_valid = neighbor_away_hub1.valid;
		i = 0;
		while(i < ADDR_LENGTH){
			addr2[i] = neighbor_away_hub2.addr[i];
			i += 1;
		}
		addr2_valid = neighbor_away_hub2.valid;
		i = 0;
		while(i < ADDR_LENGTH){
			addr3[i] = neighbor_away_hub3.addr[i];
			i += 1;
		}
		addr3_valid = neighbor_away_hub3.valid;
		uint8_t num_valid = addr1_valid + addr2_valid + addr3_valid;
		if(num_valid == 0){//replace 1
			replace_neighbor_node(&neighbor_away_hub1,new);
			return true;
		}
		else if(num_valid == 1){//figure out whether to place at 1 or 2
			if(check_addr_farther_from_hub(new->addr, addr1)){//if new is closer to hub than neighbor_to_hub1, place at 2
				replace_neighbor_node(&neighbor_away_hub2,new);//place in 2nd spot
				return true;
			}//was not closer to hub (so closer to node), move 1st to 2nd
			replace_neighbor_node(&neighbor_away_hub2,&neighbor_away_hub1);//move 1st spot to 2nd
			replace_neighbor_node(&neighbor_away_hub1,new);//move new to 1st
			return true;

		}
		else if(num_valid == 2){//figure out whether to place at 1 or 2 or 3
			if(!check_addr_farther_from_hub(new->addr, addr1)){//if new is not closer to hub than neighbor_to_hub1, place at 1 and shift
				replace_neighbor_node(&neighbor_away_hub3,&neighbor_away_hub2);
				replace_neighbor_node(&neighbor_away_hub2,&neighbor_away_hub1);
				replace_neighbor_node(&neighbor_away_hub1,new);
				return true;
			}
			else if(!check_addr_farther_from_hub(new->addr, addr2)){//new is not closer to hub than 2, so shift and place at 2
				replace_neighbor_node(&neighbor_away_hub3,&neighbor_away_hub2);
				replace_neighbor_node(&neighbor_away_hub2,new);
				return true;
			}
			else{
				replace_neighbor_node(&neighbor_away_hub3,new);//place in 3rd spot
				return true;
			}
		}
		else{//all 3 valid
			if(check_addr_farther_from_hub(new->addr, addr3)){//if new is closer to hub than 3, so shift all and place at 3
				replace_neighbor_node(&neighbor_away_hub1,&neighbor_away_hub2);//move 2nd to 1st
				replace_neighbor_node(&neighbor_away_hub2,&neighbor_away_hub3);//move 3rd to 2nd
				replace_neighbor_node(&neighbor_away_hub3,new);//move new to 3
				return true;
			}
			else if(check_addr_farther_from_hub(new->addr, addr2)){//if new is closer to hub than 2, so shift and place at 2
				replace_neighbor_node(&neighbor_away_hub1,&neighbor_away_hub2);//move 2nd to 1st
				replace_neighbor_node(&neighbor_away_hub2,new);//move new to 2
				return true;
			}
			else if(check_addr_farther_from_hub(new->addr, addr1)){//if new is closer to hub than 1, place at 1
				replace_neighbor_node(&neighbor_away_hub1,new);//move new to 1
				return true;
			}
			else{//closest to node, so don't add
				return true;
			}
		}
	}
	return false; ///error, should not go here
}


bool update_neighbor_nodes(uint8_t * sending_addr, uint8_t * battery){
	//update neighbor node list
	//mesh_neighbor neighbor_to_hub1, neighbor_to_hub2, neighbor_to_hub3, neighbor_away_hub1, neighbor_away_hub2, neighbor_away_hub3;
	//to hub is closer to hub, away is farther from, 1 is closest to node, 3 is farthest, populates 1->3
	bool good1 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_to_hub1);
	bool good2 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_to_hub2);
	bool good3 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_to_hub3);
	bool good4 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_away_hub1);
	bool good5 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_away_hub2);
	bool good6 = check_and_handle_neighbor_match(sending_addr, battery, &neighbor_away_hub3);
	//if all are false, go back and add the new neighbor node
	if(!(good1 | good2 | good3 | good4 | good5 | good6)){
		add_new_neighbor_node(sending_addr, battery);
		return false;
	}
	else{
		return true;
	}
}

bool mesh_rec_hello(volatile uint8_t * data, uint8_t * message_id, uint8_t * sending_addr, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//update node addr list if needed
	//message is dest_addr, message_id, message_type, sending_addr, battery

	uint8_t battery [BATTERY_LENGTH];
	//lora_read_fifo_all(battery, BATTERY_LENGTH, false, hdma_usart_tx, huart);
	uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
	uint8_t k = 0;
	while(k < BATTERY_LENGTH){
		battery[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t crc [1];
	//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
	crc[0] = data[i];

	//check crc
	bool good1 = calc_even_with_crc(&sending_addr[0], 1, (crc[0] & (0x1 << 0)));
	bool good2 = calc_even_with_crc(&sending_addr[1], 1, (crc[0] & (0x1 << 1)));
	bool good3 = calc_even_with_crc(&battery[0], 1, (crc[0] & (0x1 << 2)));
	if(!(good1 & good2 & good3)){
		//crc error, send a special ack: attempt = 0
		uint32_t new_id;
		memcpy(&new_id, message_id, sizeof(uint32_t));
		mesh_send_ack(sending_addr, new_id, 0, hdma_usart_tx, huart);
		return false;
	}

	bool good = update_neighbor_nodes(sending_addr, battery);
	if(isHub){
		//update mem
	}
	//rest is node
	if(!good){
		handle_send_hello();//response to a hello is own hello if new node
	}

	//update mem
	return  good;
}

bool mesh_send_dead(uint8_t * dest_addr, volatile uint8_t * dead_addr, uint8_t * dead_since, uint8_t * battery, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//message is dest_addr, message_id, message_type, dead_addr, dead_since, battery
	//dead since is 6 bytes, battery is last 2 battery, so BATTERY_LENGTH * 2

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2 + 1]; //19 + 1 crc
	//crc is 2 bytes of dead_addr, 6 bytes of dead_since

	uint8_t crc_byte = 0;
	bool even;
	int i = 0;
	int j = 0;
	int k = 0;

	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_DEAD);

	k = i;
	j = 0;
	while(i < k+ADDR_LENGTH){ //dead_addr
		message[i] = dead_addr[j];
		even = calc_crc(&dead_addr[j], 1);
		crc_byte |= (!even) << j;
		i += 1;
		j += 1;
	}
	k = i;
	j = 0;
	while(i < k+6){ //dead_since
		message[i] = dead_since[j];
		even = calc_crc(&dead_since[j], 1);
		crc_byte |= (!even) << (j + 2);
		i += 1;
		j += 1;
	}
	k = i;
	j = 0;
	while(i < k+BATTERY_LENGTH * 2){ //battery
		message[i] = battery[j];
		i += 1;
		j += 1;
	}
	message[i] = crc_byte;

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2 + 1), MESH_MSG_DEAD);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2), hdma_usart_tx, huart);
	////send_usb_ttl_message(true, MESH_MSG_DEAD, message_id, attempt, dest_addr, huart1);
	return good;
}
//dead since could be the actual time (I think 6 bytes)
bool mesh_rec_dead(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//message is dest_addr, message_id, message_type, dead_addr, dead_since, battery
	//dead since is 6 bytes, battery is last 2 battery, so BATTERY_LENGTH * 2

	if(isHub){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		//update mem

		//dest addr is node in any direction??????
		 //mesh_send_ack(uint8_t * dest_addr, message_id, hdma_usart_tx, huart);//send ack
	}
	//rest is node
	//pass on towards hub

	//get dead_addr
	uint8_t dead_addr [ADDR_LENGTH];
	//lora_read_fifo_all(dead_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);
	uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
	uint8_t k = 0;
	while(k < ADDR_LENGTH){
		dead_addr[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t dead_since [6];
	//lora_read_fifo_all(dead_since, 6, false, hdma_usart_tx, huart);//get dead_since
	k = 0;
	while(k < 6){
		dead_since[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t battery [BATTERY_LENGTH * 2];
	//lora_read_fifo_all(battery, BATTERY_LENGTH * 2, false, hdma_usart_tx, huart);//get battery
	k = 0;
	while(k < (BATTERY_LENGTH * 2)){
		battery[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_to_hub(dest_addr, 1);//find addr closer to hub, 1st attempt

	uint8_t crc [1];
	//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);//get battery
	crc[0] = data[i];

	//check crc
	bool good1 = calc_even_with_crc(&dead_addr[0], 1, (crc[0] & (0x1 << 0)));
	bool good2 = calc_even_with_crc(&dead_addr[1], 1, (crc[0] & (0x1 << 1)));
	bool good3 = calc_even_with_crc(&dead_since[0], 1, (crc[0] & (0x1 << 2)));
	bool good4 = calc_even_with_crc(&dead_since[1], 1, (crc[0] & (0x1 << 3)));
	bool good5 = calc_even_with_crc(&dead_since[2], 1, (crc[0] & (0x1 << 4)));
	bool good6 = calc_even_with_crc(&dead_since[3], 1, (crc[0] & (0x1 << 5)));
	bool good7 = calc_even_with_crc(&dead_since[0], 1, (crc[0] & (0x1 << 6)));
	bool good8 = calc_even_with_crc(&dead_since[1], 1, (crc[0] & (0x1 << 7)));
	if(!(good1 & good2 & good3 & good4 & good5 & good6 & good7 & good8 )){
		//crc error, send a special ack: attempt = 0
		uint32_t new_id;
		memcpy(&new_id, message_id, sizeof(uint32_t));
		mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		return false;
	}

	bool good;
	good = mesh_send_dead(dest_addr, dead_addr,  dead_since, battery, message_id, 1, hdma_usart_tx, huart);
	return good;//send towards hub
}


bool mesh_send_add(uint8_t * dest_addr,uint8_t * new_addr,uint8_t * coords, uint8_t * distance, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){//done
	//forget what distance was supposed to be, for node data, are we sending actual water height or just the measured distance?
	//message is dest_addr, message_id, message_type, new node_addr, node coordinates (4 bytes), distance (2)

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2 + 1]; //17 + 1 for CRC

	int i = 0;
	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_ADD);

	uint8_t crc_byte = 0;
	bool even;

	int k = i;
	int j = 0;
	while(i < k + ADDR_LENGTH){ //new_node_addr
		message[i] = new_addr[j];
		even = calc_crc(&new_addr[j], 1);
		crc_byte |= (!even) << j;
		i += 1;//even
		j += 1;//odd
	}

	k = i;
	j = 0;
	while(i < k + 4){ //coords
		message[i] = coords[j];
		even = calc_crc(&coords[j], 1);//odd
		crc_byte |= (!even) << (j + 2);//even
		i += 1;//even
		j += 1;//odd
	}

	k = i;
	j = 0;
	while(i < k + 2){ //distance
		message[i] = distance[j];
		even = calc_crc(&distance[j], 1);//even
		crc_byte |= (!even) << (j + 6);//odd
		i += 1;
		j += 1;
	}

	//calc crc and add to message, use even, so bits should sum up to an even number
	//8 bit crc, first 2 are new node addr, next 4 are for coords, last 2 are for distance
	message[i] = crc_byte;

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2 + 1), MESH_MSG_ADD);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2), hdma_usart_tx, huart);
	////send_usb_ttl_message(true, MESH_MSG_ADD, message_id, attempt, dest_addr, huart1);
	return good;
}

bool mesh_rec_add(volatile uint8_t * data, uint8_t * message_id, uint8_t * send_addr, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//forget what distance was supposed to be, for node data, are we sending actual water height or just the measured distance?

	if(isHub){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		 uint8_t new_node_addr [ADDR_LENGTH];
		    uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
		    uint8_t k = 0;

		    while(k < ADDR_LENGTH){
		        new_node_addr[k] = data[i++];
		        k++;
		    }

		    uint8_t node_coordinates [4];
		    for(k = 0; k < 4; k++){
		        node_coordinates[k] = data[i++];
		    }

		    uint8_t distance [2];
		    for(k = 0; k < 2; k++){
		        distance[k] = data[i++];
		    }

		    uint8_t crc = data[i];

		    // CRC check (same as yours)
		    bool good1 = calc_even_with_crc(&new_node_addr[0], 1, (crc & (0x1 << 0)));
		    bool good2 = calc_even_with_crc(&new_node_addr[1], 1, (crc & (0x1 << 1)));
		    bool good3 = calc_even_with_crc(&node_coordinates[0], 1, (crc & (0x1 << 2)));
		    bool good4 = calc_even_with_crc(&node_coordinates[1], 1, (crc & (0x1 << 3)));
		    bool good5 = calc_even_with_crc(&node_coordinates[2], 1, (crc & (0x1 << 4)));
		    bool good6 = calc_even_with_crc(&node_coordinates[3], 1, (crc & (0x1 << 5)));
		    bool good7 = calc_even_with_crc(&distance[0], 1, (crc & (0x1 << 6)));
		    bool good8 = calc_even_with_crc(&distance[1], 1, (crc & (0x1 << 7)));

		    if(!(good1 & good2 & good3 & good4 & good5 & good6 & good7 & good8)){
		        uint32_t new_id;
		        memcpy(&new_id, message_id, sizeof(uint32_t));
		        mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		        return false;
		    }

		    // ---- convert node id ----
		    uint16_t node_id = (new_node_addr[0] << 8) | new_node_addr[1];

		    static char nameBuf[10][10];
		    static uint8_t nameIndex = 0;

		    char *name = nameBuf[nameIndex];
		    nameIndex = (nameIndex + 1) % 10;

		    name[0] = 'N'; name[1] = 'o'; name[2] = 'd'; name[3] = 'e'; name[4] = ' ';
		    name[5] = '0' + ((node_id / 10) % 10);
		    name[6] = '0' + (node_id % 10);
		    name[7] = '\0';

		    // ADDS NODE TO UI
		    addNode(node_id, name, "Active");


		    uint32_t new_id;
		    memcpy(&new_id, message_id, sizeof(uint32_t));
		    mesh_send_ack(send_addr, new_id, 1, hdma_usart_tx, huart);

		    return true;
	}
	//rest is node

	//get new node_addr, node coordinates, distance
	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_to_hub(dest_addr, 1);

	//get new node addr
	uint8_t new_node_addr [ADDR_LENGTH];
	//lora_read_fifo_all(new_node_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);
	uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
	uint8_t k = 0;
	while(k < ADDR_LENGTH){
		new_node_addr[k] = data[i];
		i += 1;
		k += 1;
	}


	//get node_coordinates
	uint8_t node_coordinates [4];
	//lora_read_fifo_all(node_coordinates, 4, false, hdma_usart_tx, huart);
	k = 0;
	while(k < 4){
		node_coordinates[k] = data[i];
		i += 1;
		k += 1;
	}

	//get distance
	uint8_t distance [2];
	//lora_read_fifo_all(distance, 2, false, hdma_usart_tx, huart);
	k = 0;
	while(k < 2){
		distance[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t crc [1];
	//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
	crc[0] = data[i];
	//check crc

	bool good1 = calc_even_with_crc(&new_node_addr[0], 1, (crc[0] & (0x1 << 0)));
	bool good2 = calc_even_with_crc(&new_node_addr[1], 1, (crc[0] & (0x1 << 1)));
	bool good3 = calc_even_with_crc(&node_coordinates[0], 1, (crc[0] & (0x1 << 2)));
	bool good4 = calc_even_with_crc(&node_coordinates[1], 1, (crc[0] & (0x1 << 3)));
	bool good5 = calc_even_with_crc(&node_coordinates[2], 1, (crc[0] & (0x1 << 4)));
	bool good6 = calc_even_with_crc(&node_coordinates[3], 1, (crc[0] & (0x1 << 5)));
	bool good7 = calc_even_with_crc(&distance[0], 1, (crc[0] & (0x1 << 6)));
	bool good8 = calc_even_with_crc(&distance[1], 1, (crc[0] & (0x1 << 7)));
	if(!(good1 & good2 & good3 & good4 & good5 & good6 & good7 & good8 )){
		//crc error, send a special ack: attempt = 0
		uint32_t new_id;
		memcpy(&new_id, message_id, sizeof(uint32_t));
		mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		return false;
	}

	//pass on message
	bool good = mesh_send_add(dest_addr, new_node_addr, node_coordinates, distance, message_id, 1, hdma_usart_tx, huart);
	return good;
}

bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){ //done
	//message is dest_addr, message_id, message_type, sending_addr, new_frequency

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4 + 1]; //13 + 1 for crc
	//crc is new frequency

	uint8_t crc_byte = 0;
	bool even;

	int i = 0;
	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_POLL);

	int k = i;
	int j = 0;
	while(i < k + 4){ //new frequency
		message[i] = (uint8_t) (new_frequency >> (8 * j) );
		even = calc_crc(&message[i], 1);
		crc_byte |= (!even) << j;
		i += 1;
		j += 1;
	}

	message[i] = crc_byte;

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4 + 1), MESH_MSG_POLL);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4), hdma_usart_tx, huart);
	////send_usb_ttl_message(true, MESH_MSG_POLL, message_id, attempt, dest_addr, huart1);
	return good;
}

bool mesh_rec_poll(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//pass on message away from hub
	if(isHub){
		return true; //should never happen, but just in case
	}

	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_away_hub(dest_addr, 1);


	//get new frequency
	uint8_t freq_pre [4];
	//lora_read_fifo_all(freq_pre, 4, false, hdma_usart_tx, huart);
	uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
	uint8_t k = 0;
	while(k < 4){
		freq_pre[k] = data[i];
		i += 1;
		k += 1;
	}

	uint8_t crc [1];
	//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
	crc[0] = data[i];
	//check crc

	bool good1 = calc_even_with_crc(&freq_pre[0], 1, (crc[0] & (0x1 << 0)));
	bool good2 = calc_even_with_crc(&freq_pre[1], 1, (crc[0] & (0x1 << 1)));
	bool good3 = calc_even_with_crc(&freq_pre[0], 1, (crc[0] & (0x1 << 2)));
	bool good4 = calc_even_with_crc(&freq_pre[1], 1, (crc[0] & (0x1 << 3)));
	if(!(good1 & good2 & good3 & good4)){
		//crc error, send a special ack: attempt = 0
		uint32_t new_id;
		memcpy(&new_id, message_id, sizeof(uint32_t));
		mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		return false;
	}

	uint32_t new_frequency;
	memcpy(&new_frequency, freq_pre, sizeof(uint32_t));
	setup_lora_send_timer(&htim6, new_frequency);


	//send a new polling frequency message
	bool good = mesh_send_poll(dest_addr, message_id, new_frequency, 1, hdma_usart_tx, huart);
	//update own polling frequency
	setup_lora_send_timer(&htim6, new_frequency);
	return good;
}

bool mesh_rec_ack(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	if(isHub){
//		while(1){
//			//something went wrong
//		}
	}

	uint8_t acked_msg_id [4];
	//lora_read_fifo_all(acked_msg_id, 4, false, hdma_usart_tx, huart);

	uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
	uint8_t k = 0;
	while(k < 4){
		acked_msg_id[k] = data[i];
		i += 1;
		k += 1;
	}
	//check crc
	uint8_t crc [1];
	//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
	crc[0] = data[i];

	bool good1 = calc_even_with_crc(&acked_msg_id[0], 1, (crc[0] & (0x1 << 0)));
	bool good2 = calc_even_with_crc(&acked_msg_id[1], 1, (crc[0] & (0x1 << 1)));
	bool good3 = calc_even_with_crc(&acked_msg_id[2], 1, (crc[0] & (0x1 << 2)));
	bool good4 = calc_even_with_crc(&acked_msg_id[3], 1, (crc[0] & (0x1 << 3)));
	if(!(good1 & good2 & good3 & good4)){
		//crc error, send a special ack: attempt = 0
		uint32_t new_id;
		memcpy(&new_id, message_id, sizeof(uint32_t));
		mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		return false;
	}
	if((crc[0] >> 7) & 0x1){//CRC error
		//send the message again without incrementing attempt
		bool matchd1, matchd2,  matchd3,  matchd4,  matchd5, matchd6, matchd7,  matchd8,  matchd9,  matchd10;
		matchd1 = check_message_id_sent(sent_buffer.entry1, message_id);
		if(matchd1){
			add_one_send_to_sending_buffer(sent_buffer.entry1.entry);
			sent_buffer.entry1.entry.valid = false;
			return false;
		}
		matchd2 = check_message_id_sent(sent_buffer.entry2, message_id);
		if(matchd2){
			add_one_send_to_sending_buffer(sent_buffer.entry2.entry);
			sent_buffer.entry2.entry.valid = false;
			return false;
		}
		matchd3 = check_message_id_sent(sent_buffer.entry3, message_id);
		if(matchd3){
			add_one_send_to_sending_buffer(sent_buffer.entry3.entry);
			sent_buffer.entry3.entry.valid = false;
			return false;
		}
		matchd4 = check_message_id_sent(sent_buffer.entry4, message_id);
		if(matchd4){
			add_one_send_to_sending_buffer(sent_buffer.entry4.entry);
			sent_buffer.entry4.entry.valid = false;
			return false;
		}
		matchd5 = check_message_id_sent(sent_buffer.entry5, message_id);
		if(matchd5){
			add_one_send_to_sending_buffer(sent_buffer.entry5.entry);
			sent_buffer.entry5.entry.valid = false;
			return false;
		}
		matchd6 = check_message_id_sent(sent_buffer.entry6, message_id);
		if(matchd6){
			add_one_send_to_sending_buffer(sent_buffer.entry6.entry);
			sent_buffer.entry6.entry.valid = false;
			return false;
		}
		matchd7 = check_message_id_sent(sent_buffer.entry7, message_id);
		if(matchd7){
			add_one_send_to_sending_buffer(sent_buffer.entry7.entry);
			sent_buffer.entry7.entry.valid = false;
			return false;
		}
		matchd8 = check_message_id_sent(sent_buffer.entry8, message_id);
		if(matchd8){
			add_one_send_to_sending_buffer(sent_buffer.entry8.entry);
			sent_buffer.entry8.entry.valid = false;
			return false;
		}
		matchd9 = check_message_id_sent(sent_buffer.entry9, message_id);
		if(matchd9){
			add_one_send_to_sending_buffer(sent_buffer.entry9.entry);
			sent_buffer.entry9.entry.valid = false;
			return false;
		}
		matchd10 = check_message_id_sent(sent_buffer.entry10, message_id);
		if(matchd10){
			add_one_send_to_sending_buffer(sent_buffer.entry10.entry);
			sent_buffer.entry10.entry.valid = false;
			return false;
		}
	}
	clear_sent_message_buffer(acked_msg_id); //this clears the sent data struct if it is a match and does nothing if it is not

	return true;
}

////////////////////////message id is the same for the same message through the chain, should not change along chain!!

bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){ //done
	//message is dest_addr, message_id, message_type, sending_addr, Message ID that it is acking
	uint8_t message [ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4 + 1];//13 + 1 crc
	//crc is message_id (that is being acked)
	//message ID is 32 bit random number
	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)
	uint8_t crc_byte = 0;
	bool even;

	int i = 0;
	int j = 0;
	int k = 0;
	uint8_t message_id_actual [4];
	memcpy(message_id_actual, &message_id, sizeof(uint32_t));

	i = mesh_send_add_header(message, message_id_actual, dest_addr, MESH_MSG_ACK);

	k = i;
	j = 0;
	while(i < k + 4){ //acking message_id
		message[i] = (uint8_t) (acked_msg_id >> (8 * j) );
		even = calc_crc(&message[i], 1);
		crc_byte |= (!even) << j;
		i += 1;
		j += 1;
	}
	if(attempt == 0){
		crc_byte |= 1 << 7;
	}
	message[i] = crc_byte;
	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4 + 1), MESH_MSG_ACK);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4), hdma_usart_tx, huart);
	////send_usb_ttl_message(true, MESH_MSG_ACK, message_id_actual, attempt, dest_addr, huart1);
	return good;
}

int mesh_send_add_header(uint8_t *message, uint8_t * message_id, uint8_t * dest_addr, mesh_msg_type type){
	//adds dest_addr, message_id, message_type, and sending_node's addr to message
	int i = 0;
	while(i < ADDR_LENGTH){ //dest_addr
		message[i] = dest_addr[i];
		i += 1;
	}
	int j = 0;
	int k = i;
	while(i < k + 4){ //message_id
		message[i] = message_id[j];
		i += 1;
		j += 1;
	}
	message[i] = type; //message type
	i += 1;
	j = 0;
	k = i;
	while(i < k + ADDR_LENGTH){ //sending_addr
		message[i] = self_addr[j];
		i += 1;
		j += 1;
	}

	return i;
}
bool mesh_send_data(uint8_t * message_id, uint8_t * dest_addr, uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, uint8_t * time, uint8_t attempt, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){ //done
	//sending time, water distance, node addr that took data, battery level/status

	//message is dest_addr, message_id, message_type, sending_addr, node addr that took data, time, water distance, battery status/level
	int i = 0;
	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH + 1]; //19 + 1 crc
	//crc is 6 time, water, battery
	uint8_t crc_byte = 0;
	bool even;

	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_DATA);
	int j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH){ //addr that took data
		message[i] = node_addr[j];
		i += 1;
		j += 1;
	}

	//time
	message[i] = time[0];//current_time.Hours;
	even = calc_crc(&time[0], 1);
	crc_byte |= (!even) << 0;
	i += 1;
	message[i] = time[1];//current_time.Minutes;
	even = calc_crc(&time[1], 1);
	crc_byte |= (!even) << 1;
	i += 1;
	message[i] = time[2];//current_time.Seconds;
	even = calc_crc(&time[2], 1);
	crc_byte |= (!even) << 2;
	i += 1;
	message[i] = time[3];//current_date.Month;
	even = calc_crc(&time[3], 1);
	crc_byte |= (!even) << 3;
	i += 1;
	message[i] = time[4];//current_date.Date;
	even = calc_crc(&time[4], 1);
	crc_byte |= (!even) << 4;
	i += 1;
	message[i] = time[5];//current_date.Year;
	even = calc_crc(&time[5], 1);
	crc_byte |= (!even) << 5;
	i += 1;

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH){ //water distance
		message[i] = water_height[j];
		even = calc_crc(&water_height[j], 1);
		crc_byte |= (!even) << (6 + j);
		i += 1;
		j += 1;
	}

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH){ //battery status
		message[i] = battery_status[j];
		even = calc_crc(&battery_status[j], 1);
		crc_byte |= (!even) << (7 + j);
		i += 1;
		j += 1;
	}

	message[i] = crc_byte;

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH + 1), MESH_MSG_DATA);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH), hdma_usart_tx, huart);
	return good;
}

void message_id_init(message_id_history message){
	//this makes a message_id struct have default values
	message.valid = false;
	message.message_id[0] = 0;
	message.message_id[1] = 0;
	message.message_id[2] = 0;
	message.message_id[3] = 0;
}



void message_id_struct_init(){
	//Initializes 10 message id structs
	//1 is most recent, 10 is least recent
	message_id_init(message1);
	message_id_init(message2);
	message_id_init(message3);
	message_id_init(message4);
	message_id_init(message5);
	message_id_init(message6);
	message_id_init(message7);
	message_id_init(message8);
	message_id_init(message9);
	message_id_init(message10);
	//also inits 5 sent messages (the last 5 messages that this node has sent and may still need to send again)
//	message_id_init_sent(sent_message1);
//	message_id_init_sent(sent_message2);
//	message_id_init_sent(sent_message3);
//	message_id_init_sent(sent_message4);
//	message_id_init_sent(sent_message5);

}

uint8_t check_message_id(message_id_history past_message, uint8_t * message_id){
	if(past_message.valid == false){
		return 0;
	}
	bool match = check_message_struct_match(message_id, past_message.message_id);
	return match;
}

bool check_message_id_sent(sent_message_buff_entry entry,uint8_t * message_id){
	uint8_t message_id_entry [4];
	int i = 0;
	int k = ADDR_LENGTH;
	while (i < 4){
		message_id_entry[i] = entry.entry.message[k];
		i += 1;
		k += 1;
	}
	return (check_message_struct_match(message_id_entry, message_id));//& entry.entry.valid, adding valid does not work since the message could be invalid, but in the sending buffer
}


void check_message_id_all(uint8_t * message_id, bool * match){
	//checks if the message id is one that this node has received recently (in last 10 messages)
	bool matchd1, matchd2,  matchd3,  matchd4,  matchd5, matchd6, matchd7,  matchd8,  matchd9,  matchd10;
	matchd1 = check_message_id_sent(sent_buffer.entry1, message_id);
	matchd2 = check_message_id_sent(sent_buffer.entry2, message_id);
	matchd3 = check_message_id_sent(sent_buffer.entry3, message_id);
	matchd4 = check_message_id_sent(sent_buffer.entry4, message_id);
	matchd5 = check_message_id_sent(sent_buffer.entry5, message_id);
	matchd6 = check_message_id_sent(sent_buffer.entry6, message_id);
	matchd7 = check_message_id_sent(sent_buffer.entry7, message_id);
	matchd8 = check_message_id_sent(sent_buffer.entry8, message_id);
	matchd9 = check_message_id_sent(sent_buffer.entry9, message_id);
	matchd10 = check_message_id_sent(sent_buffer.entry10, message_id);
	if(matchd1 | matchd2| matchd3| matchd4| matchd5 | matchd6 | matchd7| matchd8| matchd9| matchd10){
		match[0] = true; //it was a match
		match[1] = true; //this node sent it
		return;
	}

	matchd1 = check_message_id(message1, message_id);
	matchd2 = check_message_id(message2, message_id);
	matchd3 = check_message_id(message3, message_id);
	matchd4 = check_message_id(message4, message_id);
	matchd5 = check_message_id(message5, message_id);
	matchd6 = check_message_id(message6, message_id);
	matchd7 = check_message_id(message7, message_id);
	matchd8 = check_message_id(message8, message_id);
	matchd9 = check_message_id(message9, message_id);
	matchd10 = check_message_id(message10, message_id);
	if(matchd1 | matchd2| matchd3| matchd4| matchd5 | matchd6 | matchd7| matchd8| matchd9| matchd10){
		match[0] = true; //it was a match
		match[1] = false; //this node did not send it
		return;
	}

	match[0] = false;
	match[1] = false;
	return;
}
void replace_one_message_id_struct(message_id_history changing, message_id_history values){
	changing.valid = values.valid;
	changing.message_id[0] = values.message_id[0];
	changing.message_id[1] = values.message_id[1];
	changing.message_id[2] = values.message_id[2];
	changing.message_id[3] = values.message_id[3];
}


void shift_all_messages(uint8_t * message_id, bool this_node_sent){
	//this shifts all messages
	//10 becomes 9, 9 becomes 8, ...
	replace_one_message_id_struct(message10, message9);//first is one changing to be the same as the second one
	replace_one_message_id_struct(message9, message8);
	replace_one_message_id_struct(message8, message7);
	replace_one_message_id_struct(message7, message6);
	replace_one_message_id_struct(message6, message5);
	replace_one_message_id_struct(message5, message4);
	replace_one_message_id_struct(message4, message3);
	replace_one_message_id_struct(message3, message2);
	replace_one_message_id_struct(message2, message1);
	message_id_history message0;
	message0.message_id[0] = message_id[0];
	message0.message_id[1] = message_id[1];
	message0.message_id[2] = message_id[2];
	message0.message_id[3] = message_id[3];
	message0.valid = true;
	replace_one_message_id_struct(message1, message0);
}

bool check_message_struct_match(uint8_t * message_id, volatile uint8_t * message_id2){
	//this checks if the two message ids are equal, returns true if they are
	int i = 0;
	while (i < 4){
		if(message_id[i] != message_id2[i]){
			return false;
		}
		i += 1;
	}
	return true;

}

bool sent_message_buffer_clear(uint8_t * message_id, volatile sent_message_buff_entry * entry){
	if(entry->entry.valid == false){
		return false;
	}
	int i = 0;
	int k = ADDR_LENGTH;//since message_id is not the first couple bytes
	while(i < 4){
		if(entry->entry.message[k] != message_id[i]){
			return false;
		}
		i += 1;
		k += 1;
	}
	entry->entry.valid = false;
	return true;
}

void clear_sending_buffer(uint8_t * message_id, volatile sending_buffer_entry * buffer){
	uint8_t i = 0;
	uint8_t k = ADDR_LENGTH;
	while(i < 4){
		if(message_id[i] != buffer->message[k]){
			return;
		}
		i += 1;
		k += 1;
	}
	buffer->valid = false;
}

void clear_sent_message_buffer(uint8_t * message_id){
	//this removes the message from a sent_message struct
	sent_message_buffer_clear(message_id, &sent_buffer.entry1);
	sent_message_buffer_clear(message_id, &sent_buffer.entry2);
	sent_message_buffer_clear(message_id, &sent_buffer.entry3);
	sent_message_buffer_clear(message_id, &sent_buffer.entry4);
	sent_message_buffer_clear(message_id, &sent_buffer.entry5);
	sent_message_buffer_clear(message_id, &sent_buffer.entry6);
	sent_message_buffer_clear(message_id, &sent_buffer.entry7);
	sent_message_buffer_clear(message_id, &sent_buffer.entry8);
	sent_message_buffer_clear(message_id, &sent_buffer.entry9);
	sent_message_buffer_clear(message_id, &sent_buffer.entry10);
	//also needs to clear the value from send buffer
	clear_sending_buffer(message_id, &sending_buffer.entry1);
	clear_sending_buffer(message_id, &sending_buffer.entry2);
	clear_sending_buffer(message_id, &sending_buffer.entry3);
	clear_sending_buffer(message_id, &sending_buffer.entry4);
	clear_sending_buffer(message_id, &sending_buffer.entry5);
	clear_sending_buffer(message_id, &sending_buffer.entry6);
	clear_sending_buffer(message_id, &sending_buffer.entry7);
	clear_sending_buffer(message_id, &sending_buffer.entry8);
	clear_sending_buffer(message_id, &sending_buffer.entry9);
	clear_sending_buffer(message_id, &sending_buffer.entry10);
}

bool mesh_handle_id_and_message_type(volatile uint8_t * z,volatile uint8_t * data, volatile mesh_msg_type * type, uint8_t * message_id){
	//read message_id
	uint8_t message_id_pre [4];
	int i = z[0];
	//lora_read_fifo_all(message_id_pre, 4, false, hdma_usart2_tx, huart2);
	int j = 0;
	while(j < 4){
		message_id_pre[j] = data[i];
		i += 1;
		j += 1;
	}

	bool match[2]; //0 is match or no match, 1 is true if this node sent it
	check_message_id_all(message_id_pre, match);
	int q = 0;
	while(q < 4){
		message_id[q] = message_id_pre[q];
		q += 1;
	}

	//read message_type
	uint8_t message_type [1];
	//lora_read_fifo_all(message_type, 1, false, hdma_usart2_tx, huart2);
	message_type[0] = data[i];
	i += 1;
	z[0] = i;

	type[0] = (mesh_msg_type) message_type[0]; //might be a typecasting error/warning
	if(match[0] == true){
		//matched
		if(match[1] == true){//this node sent the message
			//clear_sent_message_buffer(message_id);//don't send this message again
			return false;
		}
		else{//this node did not send the message
			return true; //was false, but false means send an ack
			//this would cause an issue if a node received a message, but ignored it because it wasn't the correct addr,
			//but later was the correct addr
		}
	}
	else{
		shift_all_messages(message_id, false); //shift all messages, adding new one to top
	}
	return true;
}

bool mesh_message_type_helper(volatile uint8_t * data, mesh_msg_type type, uint8_t * message_id, uint8_t * sending_addr,DMA_HandleTypeDef *hdma_usart_tx, UART_HandleTypeDef *huart){
	//this calls the correct sending message
	bool good;
	if(type == MESH_MSG_DATA){
		good = mesh_rec_data(data, sending_addr, message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_POLL){
		good = mesh_rec_poll(data, sending_addr, message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_ACK){
		good = mesh_rec_ack(data, sending_addr, message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_DEAD){
		good = mesh_rec_dead(data, sending_addr, message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_HELLO){
		good = mesh_rec_hello(data, message_id, sending_addr, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_ADD){
		good = mesh_rec_add(data, message_id, sending_addr, hdma_usart_tx, huart);
	}
	else{
		while(1){
			//error
		}
	}
	return good;
}

bool check_ban_addr(uint8_t * dest_addr){
	uint8_t i = 0;
	bool match1 = banned_addr1_valid;
	bool match2 = banned_addr2_valid;
	while(i < ADDR_LENGTH){
		if(banned_addr1[i] != dest_addr[i]){
			match1 = false;
		}
		if(banned_addr2[i] != dest_addr[i]){
			match2 = false;
		}
		i += 1;
	}
	return (match1 | match2);
}


bool mesh_main_rec(volatile uint8_t * data,DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){//////////////////////////done
	//this is the function that is called whenever a receive is successful in CAD mode
	//this figure out what to do with tshe message
	//return value is true if everything worked, false if something went wrong

	//read buffer to get dest_addr
	uint8_t dest_addr [ADDR_LENGTH];
	//lora_read_fifo_all(dest_addr, (uint8_t)ADDR_LENGTH, true, hdma_usart_tx, huart);
	uint8_t z = 0;
	int q = 0;
	while(z < ADDR_LENGTH){
		dest_addr[z] = ((uint8_t *)data)[z];
		z += 1;
	}

	int i = 0;
	bool addr_match = true;
	bool addr_match_right_direction = true;
	bool addr_match_any_direction = true;
	while (i < ADDR_LENGTH){
		if(dest_addr[i] != self_addr[i]){
			addr_match = false;
		}
		if(dest_addr[i] != addr_right_direction[i]){
			addr_match_right_direction = false;
		}
		if(dest_addr[i] != addr_any_direction[i]){
			addr_match_any_direction = false;
		}
		i += 1;
	}

	volatile mesh_msg_type type [1];
	uint8_t message_id [4];
	bool id_valid = mesh_handle_id_and_message_type(&z, data, type, message_id);

	uint8_t sending_addr [ADDR_LENGTH];
	//lora_read_fifo_all(sending_addr, (uint8_t)ADDR_LENGTH, false, hdma_usart_tx, huart);//get sending addr
	q = 0;
	while(q < ADDR_LENGTH){
		sending_addr[q] = data[z];
		z += 1;
		q += 1;
	}
	//ban logic
	bool banned = check_ban_addr(sending_addr);
	if(banned){
		//uint8_t message [64];
		char* str1 ="\r\n\r\nIgnored message due to banned addr";
		while(usb_ttl_done == false);
		memcpy(&message_1[0], str1, strlen(str1));
		//send_usb_ttl(message_1, strlen(str1), &huart1);
		return true;
	}
	//end ban logic
	if(id_valid == false){
			clear_sent_message_buffer(message_id);
	}

	if(id_valid == false){//this means received a message with the same id as previously seen, send an ack to stop the node from sending it again
		//send_usb_ttl_message(false, type[0], message_id, 1, sending_addr, &huart1);//usb ttl debug print
		uint32_t new_message_id;
		memcpy(&new_message_id, message_id, sizeof(uint32_t));
		//only send ack if sending addr is farther away from the destination than the self_addr
		if(type[0] == MESH_MSG_POLL){
			//only send if sender is closer to hub than self_addr
			if(check_addr_closer_to_hub(sending_addr,self_addr)){
				mesh_send_ack(sending_addr, new_message_id, 1, hdma_usart_tx, huart);
			}
		}
		else if((type[0] == MESH_MSG_DATA) | (type[0] == MESH_MSG_DEAD) | (type[0] == MESH_MSG_ADD)){
			//only send if sender is farther from hub than self addr
			if(!check_addr_closer_to_hub(sending_addr,self_addr)){
				mesh_send_ack(sending_addr, new_message_id, 1, hdma_usart_tx, huart);
			}
		}
		return true; //don't have to do anything else, false is either message that this node already sent or a message that has already been seen
	}

	if(addr_match == false){
		if(addr_match_right_direction == true){
			bool valid = check_addr_correct_dir(sending_addr, type);
			if(valid){
				//send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, &huart1);//usb ttl debug print
				bool good = mesh_message_type_helper(data, type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
				return good;
			}
			//send_usb_ttl_message(false, type[0], message_id, 2, sending_addr, &huart1);//usb ttl debug print
			return true; //don't pass on
		}
		else if(addr_match_any_direction == true){
			bool good_pre = check_addr_any_dir(sending_addr, type);
			if(good_pre == true){
				//send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, &huart1);//usb ttl debug print
				bool good = mesh_message_type_helper(data, type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
				return good;
			}
			//send_usb_ttl_message(false, type[0], message_id, 3, sending_addr, &huart1);//usb ttl debug print
			return true; //don't pass on
		}
		else{
			//do nothing
			return true;
		}
	}

	if(addr_match == true){
		//send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, &huart1);//usb ttl debug print
		bool good = mesh_message_type_helper(data, type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
		return good;
	}
	return false;//should never reach here
}

bool mesh_rec_data(volatile uint8_t * data, uint8_t * send_addr, uint8_t * message_id, DMA_HandleTypeDef * hdma_usart_tx, UART_HandleTypeDef * huart){
	//got a node_data message, figure out what to do with it

	if(isHub){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		uint8_t node_addr [ADDR_LENGTH];
		    uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
		    uint8_t k = 0;

		    while(k < ADDR_LENGTH){
		        node_addr[k] = data[i];
		        k++;
		        i++;
		    }

		    uint8_t time [6];
		    		//lora_read_fifo_all(time, 6, false, hdma_usart_tx, huart);
		    k = 0;
		    while(k < 6){
		    	time[k] = data[i];
		    	i += 1;
		    	k += 1;
		    }

		    uint8_t water_height [WATER_LENGTH];
		    		//lora_read_fifo_all(water_height, WATER_LENGTH, false, hdma_usart_tx, huart);
		    k = 0;
		    while(k < WATER_LENGTH){
		    	water_height[k] = data[i];
		    	i += 1;
		    	k += 1;
		    }

		    uint8_t battery_status [BATTERY_LENGTH];
		    		//lora_read_fifo_all(battery_status, BATTERY_LENGTH, false, hdma_usart_tx, huart);
		    k = 0;
		    while(k < BATTERY_LENGTH){
		    	battery_status[k] = data[i];
		    	i += 1;
		    	k += 1;
		    }
		    uint8_t crc [1];
		    		//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
		    crc[0] = data[i];
		    		//check crc

		    bool good1 = calc_even_with_crc(&time[0], 1, (crc[0] & (0x1 << 0)));
		    bool good2 = calc_even_with_crc(&time[1], 1, (crc[0] & (0x1 << 1)));
		    bool good3 = calc_even_with_crc(&time[2], 1, (crc[0] & (0x1 << 2)));
		    bool good4 = calc_even_with_crc(&time[3], 1, (crc[0] & (0x1 << 3)));
		    bool good5 = calc_even_with_crc(&time[4], 1, (crc[0] & (0x1 << 4)));
		    bool good6 = calc_even_with_crc(&time[5], 1, (crc[0] & (0x1 << 5)));
		    bool good7 = calc_even_with_crc(&water_height[0], 1, (crc[0] & (0x1 << 6)));
		    bool good8 = calc_even_with_crc(&battery_status[0], 1, (crc[0] & (0x1 << 7)));
		    if(!(good1 & good2 & good3 & good4 & good5 & good6 & good7 & good8)){
		    			//crc error, send a special ack: attempt = 0
		    	uint32_t new_id;
		    	memcpy(&new_id, message_id, sizeof(uint32_t));
		    	mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
		    	return false;
		    }

		    uint16_t node_id = (node_addr[0] << 8) | node_addr[1];

		    float water = (float)water_height[0];
		    float battery = (float)battery_status[0];


		    updateNodeData(node_id, water, time, battery);
		    uint32_t new_id;
		    memcpy(&new_id, message_id, sizeof(uint32_t));
		    mesh_send_ack(send_addr, new_id, 1, hdma_usart_tx, huart);

		    return true;
	}
	else{ //is a node
		uint8_t dest_addr [ADDR_LENGTH];
		find_dest_addr_to_hub(dest_addr, 1);//figure out which node to send it to, first attempt

		//get water height, battery_status, and node_addr
		uint8_t node_addr [ADDR_LENGTH];
		//lora_read_fifo_all(node_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);
		uint8_t i = ADDR_LENGTH + 4 + 1 + ADDR_LENGTH;
		uint8_t k = 0;
		while(k < ADDR_LENGTH){
			node_addr[k] = data[i];
			i += 1;
			k += 1;
		}

		uint8_t time [6];
		//lora_read_fifo_all(time, 6, false, hdma_usart_tx, huart);
		k = 0;
		while(k < 6){
			time[k] = data[i];
			i += 1;
			k += 1;
		}

		uint8_t water_height [WATER_LENGTH];
		//lora_read_fifo_all(water_height, WATER_LENGTH, false, hdma_usart_tx, huart);
		k = 0;
		while(k < WATER_LENGTH){
			water_height[k] = data[i];
			i += 1;
			k += 1;
		}

		uint8_t battery_status [BATTERY_LENGTH];
		//lora_read_fifo_all(battery_status, BATTERY_LENGTH, false, hdma_usart_tx, huart);
		k = 0;
		while(k < BATTERY_LENGTH){
			battery_status[k] = data[i];
			i += 1;
			k += 1;
		}

		uint8_t crc [1];
		//lora_read_fifo_all(crc, 1, false, hdma_usart_tx, huart);
		crc[0] = data[i];
		//check crc

		bool good1 = calc_even_with_crc(&time[0], 1, (crc[0] & (0x1 << 0)));
		bool good2 = calc_even_with_crc(&time[1], 1, (crc[0] & (0x1 << 1)));
		bool good3 = calc_even_with_crc(&time[2], 1, (crc[0] & (0x1 << 2)));
		bool good4 = calc_even_with_crc(&time[3], 1, (crc[0] & (0x1 << 3)));
		bool good5 = calc_even_with_crc(&time[4], 1, (crc[0] & (0x1 << 4)));
		bool good6 = calc_even_with_crc(&time[5], 1, (crc[0] & (0x1 << 5)));
		bool good7 = calc_even_with_crc(&water_height[0], 1, (crc[0] & (0x1 << 6)));
		bool good8 = calc_even_with_crc(&battery_status[0], 1, (crc[0] & (0x1 << 7)));
		if(!(good1 & good2 & good3 & good4 & good5 & good6 & good7 & good8)){
			//crc error, send a special ack: attempt = 0
			uint32_t new_id;
			memcpy(&new_id, message_id, sizeof(uint32_t));
			mesh_send_ack(send_addr, new_id, 0, hdma_usart_tx, huart);
			return false;
		}

		//pass on data
		bool good = mesh_send_data(message_id, dest_addr, water_height, battery_status, node_addr, time,1, hdma_usart_tx, huart);
		//send_usb_ttl_message(true, MESH_MSG_DATA, message_id, 1, dest_addr, &huart1);
		return good;

	}
	return false;
}

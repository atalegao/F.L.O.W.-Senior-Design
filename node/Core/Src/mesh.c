#include "main.h"
#include "mesh.h"
#include <string.h>

extern RTC_TimeTypeDef current_time;
extern RTC_DateTypeDef current_date;
extern bool isHub;
extern bool usb_ttl_done;
extern uint8_t last_sent_msg_id [4];

extern uint8_t self_addr [ADDR_LENGTH];
extern uint8_t addr_any_direction [ADDR_LENGTH];
extern uint8_t addr_right_direction [ADDR_LENGTH];
extern RNG_HandleTypeDef hrng;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart2_tx;

mesh_neighbor neighbor_to_hub1, neighbor_to_hub2, neighbor_to_hub3, neighbor_away_hub1, neighbor_away_hub2, neighbor_away_hub3;
//to hub is closer to hub, away is farther from, 1 is closest to node, 3 is farthest, populates 1->3

message_id_history message1, message2, message3, message4, message5, message6, message7, message8, message9, message10;

sent_message_buffer sent_buffer;

sending_buffer_type sending_buffer;

uint8_t send_buffer_add_index = 1; //is where to add the next message (counts up)
uint8_t send_buffer_send_index = 1; //is where the next message to be sent is (counts up)

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


bool send_item_off_send_buffer(void){
	//send_buffer_send_index
	sending_buffer_entry * entry = get_sending_buffer_entry(send_buffer_send_index);
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
	good = lora_send(entry->message, entry->length, hdma_usart2_tx, huart2);
	send_usb_ttl_message(true, entry->type, id, entry->attempt, dest_addr, huart1);
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

void add_to_sent_message_buffer(sending_buffer_entry * entry){
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

void replace_one_sent_buffer_entry(sending_buffer_entry * new_entry, sent_message_buff_entry * old_entry){
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
//TODO: if a message is received that the node has already seen, ack it to stop the sending one from sending over and over again
//hub handles this, just need for nodes
//what if it is sent to a different addr?, ack anyway: yes

//TODO: need a timer to send hello
//TODO:need a timer to handle resending
//TODO: need a timer to send own data (could be the same one to do ultrasonic)

//use the buffer for sent messages and have a timer to add a sent message to the send buffer if the message is still in the sent buffer and is still valid
//TODO: need a timer to resend a message that was not received, the timer calls the below function
void handle_resending(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
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
}

void handle_one_resending(time_t current_time, sent_message_buff_entry * sent_message){
	if(sent_message->entry.valid == false){
		return;
	}

	bool past_time = decide_if_past_time(current_time, sent_message->last_sent_time);
	if(past_time){
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
	sending_buffer_entry * sending_buffer_at_add_index = get_sending_buffer_entry(send_buffer_add_index);
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

sending_buffer_entry * get_sending_buffer_entry(uint8_t index){
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
time_t get_time_in_seconds(RTC_TimeTypeDef *time, RTC_DateTypeDef *date){
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

bool check_addr_closer_to_hub(uint8_t * first_addr,uint8_t * second_addr){
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

bool check_addr_farther_from_hub(uint8_t * first_addr,uint8_t * second_addr){
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

bool check_addr_correct_dir(uint8_t * sending_addr, mesh_msg_type * type){
	//check if addr is in the correct direction, if so return true, else false
	if((type[0] == MESH_MSG_POLL) | (type[0] == MESH_MSG_ACK)){//away from hub
		return check_addr_farther_from_hub(self_addr, sending_addr);
	}
	else{//towards hub: data, dead, add
		return check_addr_closer_to_hub(self_addr, sending_addr);
	}
    //hello is neither
}

bool check_addr_any_dir(uint8_t * sending_addr, mesh_msg_type * type){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
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

bool mesh_send_hello(uint8_t * battery, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//message is dest_addr, message_id, message_type, sending_addr, battery

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH];

	//message ID is 32 bit random number
	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)
	uint8_t message_id_actual [4];
	memcpy(message_id_actual, &message_id, sizeof(uint32_t));

	//dest addr is any direction
	uint8_t dest_addr [2];
	dest_addr [0] = addr_any_direction[0];
	dest_addr [1] = addr_any_direction[1];

	int i = 0;

	i = mesh_send_add_header(message, message_id_actual, dest_addr, MESH_MSG_HELLO);

	int k = i;
	int j = 0;
	while(i < k+BATTERY_LENGTH){
		message[i] = battery[j];
		i += 1;
		j += 1;
	}

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, 1, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH), MESH_MSG_HELLO);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + BATTERY_LENGTH), hdma_usart_tx, huart);
	//send_usb_ttl_message(true, MESH_MSG_HELLO, message_id_actual, 1, dest_addr, huart1);
	return good;
}

bool check_and_handle_neighbor_match(uint8_t * addr, uint8_t * battery, mesh_neighbor * neighbor){
	int i = 0;
	while(i < ADDR_LENGTH){
		if(addr[i] != neighbor->addr[i]){
			return false;
		}
		i += 1;
	}
	//got this far means a match to an existing addr, so just update battery, first battery is most recent, 2nd is least recent
	i = 0;
	while(i < BATTERY_LENGTH){
		neighbor->battery[i + ADDR_LENGTH] = neighbor->battery[i];
		i += 1;
	}
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

void replace_neighbor_node(mesh_neighbor * replaced,mesh_neighbor * replacing){
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
		bool good = add_new_neighbor_node(sending_addr, battery);
		return good;
	}
	else{
		return true;
	}
}

bool mesh_rec_hello(uint8_t * sending_addr, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//update node addr list if needed
	//message is dest_addr, message_id, message_type, sending_addr, battery

	uint8_t battery [BATTERY_LENGTH];
	lora_read_fifo_all(battery, BATTERY_LENGTH, false, hdma_usart_tx, huart);
	bool good = update_neighbor_nodes(sending_addr, battery);
	if(isHub){
		//update mem
	}
	//rest is node

	//update mem
	return  good;
}

bool mesh_send_dead(uint8_t * dest_addr, uint8_t * dead_addr, uint8_t * dead_since, uint8_t * battery, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//message is dest_addr, message_id, message_type, dead_addr, dead_since, battery
	//dead since is 6 bytes, battery is last 2 battery, so BATTERY_LENGTH * 2

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2];

	int i = 0;
	int j = 0;
	int k = 0;

	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_DEAD);

	k = i;
	j = 0;
	while(i < k+ADDR_LENGTH){ //dead_addr
		message[i] = dead_addr[j];
		i += 1;
		j += 1;
	}
	k = i;
	j = 0;
	while(i < k+6){ //dead_since
		message[i] = dead_since[j];
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
	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2), MESH_MSG_DEAD);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 6 + BATTERY_LENGTH * 2), hdma_usart_tx, huart);
	//send_usb_ttl_message(true, MESH_MSG_DEAD, message_id, attempt, dest_addr, huart1);
	return good;
}
//dead since could be the actual time (I think 6 bytes)
bool mesh_rec_dead(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
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
	lora_read_fifo_all(dead_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);

	uint8_t dead_since [6];
	lora_read_fifo_all(dead_since, 6, false, hdma_usart_tx, huart);//get dead_since

	uint8_t battery [BATTERY_LENGTH * 2];
	lora_read_fifo_all(battery, BATTERY_LENGTH * 2, false, hdma_usart_tx, huart);//get battery

	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_to_hub(dest_addr, 1);//find addr closer to hub, 1st attempt

	bool good;
	good = mesh_send_dead(dest_addr, dead_addr,  dead_since, battery, message_id, 1, hdma_usart_tx, huart);
	return good;//send towards hub
}


bool mesh_send_add(uint8_t * dest_addr,uint8_t * new_addr,uint8_t * coords, uint8_t * distance, uint8_t * message_id, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done
	//forget what distance was supposed to be, for node data, are we sending actual water height or just the measured distance?
	//message is dest_addr, message_id, message_type, new node_addr, node coordinates (4 bytes), distance (2)

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2];

	int i = 0;
	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_ADD);

	int k = i;
	int j = 0;
	while(i < k + ADDR_LENGTH){ //new_node_addr
		message[i] = new_addr[j];
		i += 1;
		j += 1;
	}

	k = i;
	j = 0;
	while(i < k + 4){ //coords
		message[i] = coords[j];
		i += 1;
		j += 1;
	}

	k = i;
	j = 0;
	while(i < k + 2){ //distance
		message[i] = distance[j];
		i += 1;
		j += 1;
	}

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2), MESH_MSG_ADD);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH + 4 + 2), hdma_usart_tx, huart);
	//send_usb_ttl_message(true, MESH_MSG_ADD, message_id, attempt, dest_addr, huart1);
	return good;
}

bool mesh_rec_add(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//forget what distance was supposed to be, for node data, are we sending actual water height or just the measured distance?

	if(isHub){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		//add to mem
		//send ack
	}
	//rest is node

	//get new node_addr, node coordinates, distance
	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_to_hub(dest_addr, 1);

	//get new node addr
	uint8_t new_node_addr [ADDR_LENGTH];
	lora_read_fifo_all(new_node_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);

	//get node_coordinates
	uint8_t node_coordinates [4];
	lora_read_fifo_all(node_coordinates, 4, false, hdma_usart_tx, huart);

	//get distance
	uint8_t distance [2];
	lora_read_fifo_all(distance, 2, false, hdma_usart_tx, huart);

	//pass on message
	bool good = mesh_send_add(dest_addr, new_node_addr, node_coordinates, distance, message_id, 1, hdma_usart_tx, huart);
	return good;
}

bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done
	//message is dest_addr, message_id, message_type, sending_addr, new_frequency

	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4];

	int i = 0;
	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_POLL);

	int k = i;
	int j = 0;
	while(i < k + 4){ //new frequency
		message[i] = (uint8_t) (new_frequency >> (8 * j) );
		i += 1;
		j += 1;
	}

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4), MESH_MSG_POLL);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 4), hdma_usart_tx, huart);
	//send_usb_ttl_message(true, MESH_MSG_POLL, message_id, attempt, dest_addr, huart1);
	return good;
}

bool mesh_rec_poll(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//pass on message away from hub
	if(isHub){
		return true; //should never happen, but just in case
	}

	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_away_hub(dest_addr, 1);

	//get new frequency
	uint8_t freq_pre [4];
	lora_read_fifo_all(freq_pre, 4, false, hdma_usart_tx, huart);

	uint32_t new_frequency;
	memcpy(&new_frequency, freq_pre, sizeof(uint32_t));
	setup_lora_send_timer(&htim6, new_frequency);


	//send a new polling frequency message
	bool good = mesh_send_poll(dest_addr, message_id, new_frequency, 1, hdma_usart_tx, huart);
	//update own polling frequency
	setup_lora_send_timer(&htim6, new_frequency);
	return good;
}

bool mesh_rec_ack(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	if(isHub){
		while(1){
			//something went wrong
		}
	}

	uint8_t acked_msg_id [4];
	lora_read_fifo_all(acked_msg_id, 4, false, hdma_usart_tx, huart);
	clear_sent_message_buffer(acked_msg_id); //this clears the sent data struct if it is a match and does nothing if it is not

	return true;
}

////////////////////////message id is the same for the same message through the chain, should not change along chain!!

bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done
	//message is dest_addr, message_id, message_type, sending_addr, Message ID that it is acking
	uint8_t message [ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4];

	//message ID is 32 bit random number
	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)

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
		i += 1;
		j += 1;
	}
	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4), MESH_MSG_ACK);
	good = add_one_send_to_sending_buffer(entry);
	//good = lora_send(message, (ADDR_LENGTH + ADDR_LENGTH+ 4 + 1 + 4), hdma_usart_tx, huart);
	//send_usb_ttl_message(true, MESH_MSG_ACK, message_id_actual, attempt, dest_addr, huart1);
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
bool mesh_send_data(uint8_t * message_id, uint8_t * dest_addr, uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, uint8_t * time, uint8_t attempt, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done
	//sending time, water distance, node addr that took data, battery level/status

	//message is dest_addr, message_id, message_type, sending_addr, node addr that took data, time, water distance, battery status/level
	int i = 0;
	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH];

	i = mesh_send_add_header(message, message_id, dest_addr, MESH_MSG_DATA);
	int j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH){ //addr that took data
		message[i] = node_addr[j];
		i += 1;
		j += 1;
	}

	//time
	message[i] = time[0];//current_time.Hours;
	i += 1;
	message[i] = time[1];//current_time.Minutes;
	i += 1;
	message[i] = time[2];//current_time.Seconds;
	i += 1;
	message[i] = time[3];//current_date.Month;
	i += 1;
	message[i] = time[4];//current_date.Date;
	i += 1;
	message[i] = time[5];//current_date.Year;
	i += 1;

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH){ //water distance
		message[i] = water_height[j];
		i += 1;
		j += 1;
	}

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH){ //battery status
		message[i] = battery_status[j];
		i += 1;
		j += 1;
	}

	bool good;
	sending_buffer_entry entry = make_sending_buffer_entry(message, attempt, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH), MESH_MSG_DATA);
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

bool check_message_struct_match(uint8_t * message_id, uint8_t * message_id2){
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

bool sent_message_buffer_clear(uint8_t * message_id, sent_message_buff_entry * entry){
	if(entry->entry.valid == false){
		return false;
	}
	bool id_match = true;
	int i = 0;
	int k = ADDR_LENGTH;//since message_id is not the first couple bytes
	while(i < 4){
		if(entry->entry.message[k] != message_id[i]){
			id_match = false;
			return false;
		}
		i += 1;
	}
	entry->entry.valid = false;
	return true;
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
}

bool mesh_handle_id_and_message_type(mesh_msg_type * type, uint8_t * message_id){
	//read message_id
	uint8_t message_id_pre [4];
	lora_read_fifo_all(message_id_pre, 4, false, hdma_usart2_tx, huart2);


	bool match[2]; //0 is match or no match, 1 is true if this node sent it
	check_message_id_all(message_id_pre, match);
	int i = 0;
	while(i < 4){
		message_id[i] = message_id_pre[i];
		i += 1;
	}
	if(match[0] == true){
		//matched
		if(match[1] == true){//this node sent the message
			clear_sent_message_buffer(message_id);//don't send this message again
			return false;
		}
		else{//this node did not send the message
			return false;
		}
	}
	else{
		shift_all_messages(message_id, false); //shift all messages, adding new one to top
	}

	//read message_type
	uint8_t message_type [1];
	lora_read_fifo_all(message_type, 1, false, hdma_usart2_tx, huart2);

	type[0] = (mesh_msg_type) message_type[0]; //might be a typecasting error/warning
	return true;
}

bool mesh_message_type_helper(mesh_msg_type type, uint8_t * message_id, uint8_t * sending_addr,DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//this calls the correct sending message
	bool good;
	if(type == MESH_MSG_DATA){
		good = mesh_rec_data(message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_POLL){
		good = mesh_rec_poll(message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_ACK){
		good = mesh_rec_ack(hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_DEAD){
		good = mesh_rec_dead(message_id, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_HELLO){
		good = mesh_rec_hello(sending_addr, hdma_usart_tx, huart);
	}
	else if(type == MESH_MSG_ADD){
		good = mesh_rec_add(message_id, hdma_usart_tx, huart);
	}
	else{
		while(1){
			//error
		}
	}
	return good;
}




bool mesh_main_rec(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//////////////////////////done
	//this is the function that is called whenever a receive is successful in CAD mode
	//this figure out what to do with the message
	//return value is true if everything worked, false if something went wrong

	//read buffer to get dest_addr
	uint8_t dest_addr [ADDR_LENGTH];
	lora_read_fifo_all(dest_addr, (uint8_t)ADDR_LENGTH, true, hdma_usart_tx, huart);

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
	mesh_msg_type type [1];
	uint8_t message_id [4];
	bool id_valid = mesh_handle_id_and_message_type(type, message_id);

	uint8_t sending_addr [ADDR_LENGTH];
	lora_read_fifo_all(sending_addr, (uint8_t)ADDR_LENGTH, false, hdma_usart_tx, huart);//get sending addr

	if(id_valid == false){//this means received a message with the same id as previously seen, send an ack to stop the node from sending it again
		send_usb_ttl_message(false, type[0], message_id, 1, sending_addr, huart1);//usb ttl debug print
		uint32_t new_message_id;
		memcpy(&new_message_id, message_id, sizeof(uint32_t));
		mesh_send_ack(sending_addr, new_message_id, 1, hdma_usart_tx, huart);
		return true; //don't have to do anything else, false is either message that this node already sent or a message that has already been seen
	}

	if(addr_match == false){
		if(addr_match_right_direction == true){
			bool valid = check_addr_correct_dir(sending_addr, type);
			if(valid){
				send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, huart1);//usb ttl debug print
				bool good = mesh_message_type_helper(type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
				return good;
			}
			send_usb_ttl_message(false, type[0], message_id, 2, sending_addr, huart1);//usb ttl debug print
			return true; //don't pass on
		}
		else if(addr_match_any_direction == true){
			bool good_pre = check_addr_any_dir(sending_addr, type);
			if(good_pre == true){
				send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, huart1);//usb ttl debug print
				bool good = mesh_message_type_helper(type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
				return good;
			}
			send_usb_ttl_message(false, type[0], message_id, 3, sending_addr, huart1);//usb ttl debug print
			return true; //don't pass on
		}
		else{
			//do nothing
			return true;
		}
	}

	if(addr_match == true){
		send_usb_ttl_message(false, type[0], message_id, 0, sending_addr, huart1);//usb ttl debug print
		bool good = mesh_message_type_helper(type[0], message_id, sending_addr,hdma_usart_tx, huart);//pass on
		return good;
	}
	return false;//should never reach here
}

bool mesh_rec_data(uint8_t * message_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//got a node_data message, figure out what to do with it

	if(isHub){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		//add data to mem
		//send ack
	}
	else{ //is a node
		uint8_t dest_addr [ADDR_LENGTH];
		find_dest_addr_to_hub(dest_addr, 1);//figure out which node to send it to, first attempt

		//get water height, battery_status, and node_addr
		uint8_t node_addr [ADDR_LENGTH];
		lora_read_fifo_all(node_addr, ADDR_LENGTH, false, hdma_usart_tx, huart);

		uint8_t time [6];
		lora_read_fifo_all(time, 6, false, hdma_usart_tx, huart);

		uint8_t water_height [WATER_LENGTH];
		lora_read_fifo_all(water_height, WATER_LENGTH, false, hdma_usart_tx, huart);

		uint8_t battery_status [BATTERY_LENGTH];
		lora_read_fifo_all(battery_status, BATTERY_LENGTH, false, hdma_usart_tx, huart);

		//pass on data
		bool good = mesh_send_data(message_id, dest_addr, water_height, battery_status, node_addr, time,1, hdma_usart_tx, huart);
		send_usb_ttl_message(true, MESH_MSG_DATA, message_id, 1, dest_addr, huart1);
		return good;

	}
	return false;
}

void send_usb_ttl_message(bool sent, mesh_msg_type type, uint8_t * message_id, uint8_t time_or_ignore_reason, uint8_t * send_or_rec_addr, UART_HandleTypeDef huart){
	//this prints a message to the usb-ttl about sent and received messages
	uint8_t message [64];
	if(sent){
		char* str1 ="\r\n\r\nSent a message with id: ";
		while(usb_ttl_done == false);
		memcpy(&message[0], str1, strlen(str1));
		send_usb_ttl(message, strlen(str1), huart);
	}
	else {
		char* str1 ="\r\n\r\nReceived a message with id: ";
		while(usb_ttl_done == false);
		memcpy(&message[0], str1, strlen(str1));
		send_usb_ttl(message, strlen(str1), huart);
	}
	while(usb_ttl_done == false);
	send_usb_ttl(message_id, 4, huart);
	char * str2;
	switch(type){
	case MESH_MSG_DATA:
		str2 ="\r\nof type data";
		break;
	case MESH_MSG_POLL:
		str2 ="\r\nof type poll";
		break;
	case MESH_MSG_ACK:
		str2 ="\r\nof type ack";
		break;
	case MESH_MSG_DEAD:
		str2 ="\r\nof type dead";
		break;
	case MESH_MSG_HELLO:
		str2 ="\r\nof type hello";
		break;
	case MESH_MSG_ADD:
		str2 ="\r\nof type add";
		break;
	}
	while(usb_ttl_done == false);
	memcpy(&message[0], str2, strlen(str2));
	send_usb_ttl(message, strlen(str2), huart);
	if(sent){
		char* str3 =" for the";
		while(usb_ttl_done == false);
		memcpy(&message[0], str3, strlen(str3));
		send_usb_ttl(message, strlen(str3), huart);
		while(usb_ttl_done == false);
		send_usb_ttl(&time_or_ignore_reason, 1, huart);
		char* str5 =" time, rec addr is";
		while(usb_ttl_done == false);
		memcpy(&message[0], str5, strlen(str5));
		send_usb_ttl(message, strlen(str5), huart);
		while(usb_ttl_done == false);
		send_usb_ttl(send_or_rec_addr, ADDR_LENGTH, huart);
	}
	else{
		char* str4;
		switch(time_or_ignore_reason){
		case 0:
			str4 =" and accepted the message from addr ";
			break;
		case 1:
			str4 =" and ignored due to invalid id from addr";
			break;
		case 2:
			str4 =" and ignored due to right direction fail from addr";
			break;
		case 3:
			str4 =" and ignored due to any direction fail from addr";
			break;
		}
		while(usb_ttl_done == false);
		memcpy(&message[0], str4, strlen(str4));
		send_usb_ttl(message, strlen(str4), huart);
		while(usb_ttl_done == false);
		send_usb_ttl(send_or_rec_addr, ADDR_LENGTH, huart);
	}
}

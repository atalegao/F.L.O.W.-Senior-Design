extern RTC_TimeTypeDef *current_time;
extern RTC_DateTypeDef *current_date;
extern bool isHub;
extern uint8_t last_sent_msg_id [4];

extern uint8_t [ADDR_LENGTH] self_addr;

void mesh_init(bool isHub, uint8_t ownAddress){
    if (isHub) {
        // Initialize as a hub
        // Set up necessary data structures for a hub
    } else {
        // Initialize as a node
        // Set up necessary data structures for a node
    }

}  //isHub is just a bool which indicates whether a module is a node or hub, since they have different characteristics. 
void mesh_set_hello_interval(uint32_t seconds){
    interval = 
}
//random number function, put somewhere else
uint32_t random_number_gen(void){
	uint32_t random_number;
	if (HAL_RNG_GenerateRandomNumber(&RngHandle, &random_number) != HAL_OK) {
	    // Error handling
	    Error_Handler();
	}
	return random_number;
}

bool mesh_send_hello(uint8_t battery);//update node addr list in rec version
bool mesh_send_dead(uint8_t dest, uint8_t dead_addr, uint32_t dead_since, uint8_t battery); 
bool mesh_send_add(uint8_t dest,uint8_t new_addr,uint32_t coords, uint16_t distance);

bool mesh_send_poll(uint8_t * dest_addr, uint8_t * message_id, uint32_t new_frequency, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//message is dest_addr, message_id, message_type, new_frequency

	uint8_t message [ADDR_LENGTH + 4 + 1 + 4];

	int i = 0;
	int j = 0;
	int k = 0;

	while(i < ADDR_LENGTH){ //dest_addr
			message[i] = dest_addr[i];
			i += 1;
	}

	k = i;
	j = 0;
	while(i < k + 4){ //message_id
		message[i] = message_id[j];
		i += 1;
		j += 1;
	}

	message[i] = MESH_MSG_POLL; //message type
	i += 1;

	k = i;
	j = 0;
	while(i < k + 4){ //new frequency
		message[i] = (uint8_t) (new_frequency >> (8 * j) );
		i += 1;
		j += 1;
	}

	bool good;
	good = lora_send(message, (ADDR_LENGTH + 4 + 1 + 4), hdma_usart_tx, huart);
	return good;
}

bool mesh_rec_poll(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//pass on message away from hub
	if(isHub){
		return true; //should never happen, but just in case
	}

	uint8_t dest_addr [ADDR_LENGTH];
	find_dest_addr_away_hub(uint8_t * dest_addr);

	//get new frequency
	uint8_t freq_pre [4];
	lora_read_fifo_all(freq_pre, 4, hdma_usart_tx, huart);
	//change polling frequency //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

	uint32_t new_frequency = {freq_pre[3], freq_pre[2], freq_pre[1], freq_pre[0]};


	//send a new polling frequency message
	bool good = mesh_send_poll(dest_addr, new_frequency,hdma_usart_tx, huart);
	return good;
}

bool mesh_rec_ack(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	if(isHub){
		while(1){
			//something went wrong
		}
	}

	uint8_t acked_msg_id [4];
	lora_read_fifo_all(acked_msg_id, 4, hdma_usart_tx, huart);

	bool same_ack = true;
	while (i < 4){
		if(acked_msg_id[i] != last_sent_msg_id[i]){
			same_ack = false;
			break;
		}
		i += 1;
	}

	if(same_ack){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		//own message was acked, so don't send again
		//change some global variable to not send the message again
	}
	else{
		//not own message that was acked, so ignore
		return true;
	}
}

////////////////////////message id is the same for the same message through the chain, should not change along chain!!

bool mesh_send_ack(uint8_t * dest_addr, uint32_t acked_msg_id, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//message is dest_addr, message_id, message_type, Message ID that it is acking
	uint8_t message [ADDR_LENGTH + 4 + 1 + 4];

	//message ID is 32 bit random number
	uint32_t message_id;
	message_id = random_number_gen(); //new since this will always be a new message (does not get passed on)

	int i = 0;
	int j = 0;
	int k = 0;

	while(i < ADDR_LENGTH){ //dest_addr
			message[i] = dest_addr[i];
			i += 1;
	}

	k = i;
	j = 0;
	while(i < k + 4){ //message_id
		message[i] = (uint8_t) (message_id >> (8 * j) );
		i += 1;
		j += 1;
	}

	message[i] = MESH_MSG_ACK; //message type
	i += 1;

	k = i;
	j = 0;
	while(i < k + 4){ //acking message_id
		message[i] = (uint8_t) (acked_msg_id >> (8 * j) );
		i += 1;
		j += 1;
	}
	bool good;
	good = lora_send(message, (ADDR_LENGTH + 4 + 1 + 4), hdma_usart1_tx, huart1);
	return good;
}

bool mesh_send_data(uint8_t * message_id, uint8_t * dest_addr, uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, uint8_t * time, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//sending time, water distance, node addr that took data, battery level/status

	//message is dest_addr, message_id, message_type, node addr that took data, time, water distance, battery status/level
	int i = 0;
	uint8_t message [ADDR_LENGTH + 4 + 1 + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH];
	while(i < ADDR_LENGTH){ //dest_addr
		message[i] = dest_addr[i];
		i += 1;
	}
	int j = 0;
	while(i < ADDR_LENGTH + 4){ //message_id
		message[i] = message_id[j];
		i += 1;
		j += 1;
	}
	message[i] = MESH_MSG_DATA; //message type
	i += 1;
	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH){ //addr that took data
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

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 + ADDR_LENGTH + 6 + WATER_LENGTH){ //water distance
		message[i] = water_height[j];
		i += 1;
		j += 1;
	}

	j = 0;
	while(i < ADDR_LENGTH + 4 + 1 +  ADDR_LENGTH + 6 + WATER_LENGTH + BATTERY_LENGTH){ //battery status
		message[i] = battery_status[j];
		i += 1;
		j += 1;
	}

	bool good;
	good = lora_send(message, (ADDR_LENGTH + 4 + 1 + ADDR_LENGTH +  6 + WATER_LENGTH + BATTERY_LENGTH), hdma_usart1_tx, huart1);
	return good;
}
bool mesh_handle_id_and_message_type(mesh_msg_type * type){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//read message_id
	uint8_t message_id [4];
	lora_read_fifo_all(message_id, 4, hdma_usart_tx, huart);

	TODO: check if message id was already seen
	: if so, ignore and return false
	: else continue

	  TODO: check if message id was one this node already sent
	  	: if so, dont send it again and return false
	  	: else continue

	//read message_type
	uint8_t message_type [1];
	lora_read_fifo_all(message_type, 1, hdma_usart_tx, huart);

	TODO: will not work since read_fifo currently just resets fifo pointer to 0,
			it needs to actually make it go to the correct one, else use a new function that does not modify the fifo pointer and just reads

	type[0] = (mesh_msg_type) message_type[0]; //might be a typecasting error/warning
	return true;
}

bool mesh_message_type_helper(mesh_msg_type type){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//this calls the correct sending message
	bool good;
	if(type == MESH_MSG_DATA){
		good = mesh_send_data();
	}
	else if(type == MESH_MSG_POLL){
		good = mesh_send_poll();
	}
	else if(type == MESH_MSG_ACK){
		good = mesh_send_ack();
	}
	else if(type == MESH_MSG_DEAD){
		good = mesh_send_dead();
	}
	else if(type == MESH_MSG_HELLO){
		good = mesh_send_hello();
	}
	else if(type == MESH_MSG_ADD){
		good = mesh_send_add();
	}
	else{
		while(1){
			//error
		}
	}
}
bool check_addr_correct_dir(uint8_t * dest_addr){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//TODO: check if addr is in the correct direction, if so return true, else false
}

bool check_addr_any_dir(uint8_t * dest_addr, mesh_msg_type * type){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//TODO:check if this node knows any nodes in the direction closer to the hub than the sender's addr
	//or father from hub than sender's addr
}

void find_dest_addr_to_hub(uint8_t * dest_addr){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//finds dest addr for a message going towards the hub
}

void find_dest_addr_away_hub(uint8_t * dest_addr){//TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	//finds dest addr for a message going away from the hub
}

bool mesh_main_rec(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//////////////////////////done
	//this is the function that is called whenever a receive is successful in CAD mode
	//this figure out what to do with the message
	//return value is true if everything worked, false if something went wrong

	//read buffer to get dest_addr
	uint8_t dest_addr [ADDR_LENGTH];
	lora_read_fifo_all(dest_addr, (uint8_t)ADDR_LENGTH, hdma_usart_tx, huart);

	int i = 0;
	bool addr_match = true;
	bool addr_match_right_direction = true;
	bool addr_match_any_direction = true;
	while (i < ADDR_LENGTH){
		if(dest_addr[i] != self_addr[i]){
			addr_match = false;
			break;
		}
		i += 1;
	}
	mesh_msg_type type [1];
	bool id_valid = mesh_handle_id_and_message_type(type);

	if(id_valid == false){
		return true; //don't have to do anything else, false is either message that this node already sent or a message that has already been seen
	}

	if(addr_match == false){
		if(addr_match_right_direction == true){
			bool valid = check_addr_correct_dir(dest_addr);
			if(valid){
				bool good = mesh_message_type_helper(type);//pass on
				return good;
			}
			return true; //don't pass on
		}
		else if(addr_match_any_direction == true){
			bool good_pre = check_addr_any_dir(dest_addr, type);
			if(good_pre == true){
				bool good = mesh_message_type_helper(type);//pass on
				return good;
			}
			return true; //don't pass on
		}
		else{
			//do nothing
			return true;
		}
	}

	if(addr_match == true){
		bool good = mesh_message_type_helper(type);//pass on
		return good;
	}
}

bool mesh_rec_data(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//got a node_data message, figure out what to do with it

	if(isHub){ //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		//add data to mem
		//send ack
	}
	else{ //is a node
		uint8_t dest_addr [ADDR_LENGTH];
		find_dest_addr_to_hub(dest_addr);//figure out which node to send it to

		//get water height, battery_status, and node_addr
		uint8_t node_addr [ADDR_LENGTH];
		lora_read_fifo_all(node_addr, ADDR_LENGTH, hdma_usart_tx, huart);

		uint8_t time [6];
		lora_read_fifo_all(time, 6, hdma_usart_tx, huart);

		uint8_t water_height [WATER_LENGTH];
		lora_read_fifo_all(water_height, WATER_LENGTH, hdma_usart_tx, huart);

		uint8_t battery_status [BATTERY_LENGTH];
		lora_read_fifo_all(battery_status, BATTERY_LENGTH, hdma_usart_tx, huart);

		//pass on data
		bool good = mesh_send_data(dest_addr, water_height, battery_status, node_addr, time,hdma_usart_tx, huart);
		return good;

	}
}

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


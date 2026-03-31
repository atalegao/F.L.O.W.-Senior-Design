bool lora_node_data_send(uint8_t* water_height, uint8_t *battery_status, uint8_t * node_addr, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//sending time, water distance, node addr that took data, battery level/status

	//get sending time from RTC

	//put all data into one buffer and give to sending function

}

bool lora_node_data_rec(uint8_t* data, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	//
}

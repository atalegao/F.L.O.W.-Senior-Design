#include <lora.h>
#include <mesh.h>
extern bool sendfifo_ready_norm;
extern int sendfifo_offset_norm;
extern bool rx_ready;
extern uint8_t sendfifo_norm[FIFOSIZE_TX_NORM]; //array of data sending to LoRa module
extern uint8_t sendfifo_send_message[FIFOSIZE_TX_SEND]; //array of data sending to LoRa module for an actual message send
extern uint8_t sendfifo_rec_message[FIFOSIZE_TX_REC]; //array of data sending to LoRa module for reading FIFO buffer
extern uint8_t receivefifo[FIFOSIZE_RX]; //array of data read from LoRa module
extern uint8_t global_receive_mode_from_cad;
extern int sendfifo_offset_send;
extern bool sendfifo_ready_send;
extern int sendfifo_offset_rec;
extern bool sendfifo_ready_rec;
extern bool send_normal;
extern bool send_send;
extern bool send_rec;
extern bool read_lora_fifo;


bool doing_connected_test = false;
bool doing_send = false;


 bool lora_init(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//not done, not tested
        //sets preamble length, center frequency, Tx power, and modem config
        // ALSO NEED TO SET ADDRESS of the node (needed depending on AddressFiltering register, but reg is 34)
        // (default is off)

        //write formula: 57, reg | 0x80, 01, value (2 hex)
        // //read formula: 52, reg & ~0x80, 01

        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE, 1); // 57 81 01 80 //normal
        //lora_read_single(0x01);//testing

        //setup FIFO
        lora_write_single(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0, 1); //57 8E 01 00//normal
        lora_write_single(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0, 1); //57 8F 01 00//normal

        //set mode to IDLE
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY, 1); // 57 81 01 01//normal
        //lora_read_single(0x01);//testing

        //setPreambleLength Default is 8 bytes
        // 57, reg | 80, 01, value (2 hex)
        lora_write_single(RH_RF95_REG_20_PREAMBLE_MSB, PREAMBLE_LENGTH >> 8, 1); // 57 A0 01 00 //normal
        lora_write_single(RH_RF95_REG_21_PREAMBLE_LSB, PREAMBLE_LENGTH & 0xff, 1); // 57 A1 01 08 //normal

        //setFrequency(868.0);
        uint32_t frf = (CENTER_FREQUENCY * 1000000.0) / RH_RF95_FSTEP; //14,221,312, D9_00_00 //normal
        lora_write_single(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff, 1); // 57 86 01 D9 //normal
        lora_write_single(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff, 1); // 57 87 01 00 //normal
        lora_write_single(RH_RF95_REG_08_FRF_LSB, frf & 0xff, 1); // 57 88 01 00 //normal

        //setTxPower(13);
        int8_t power = TXPOWER;
        if (power > 23) {
            power = 23;
        }
        if (power < 5) {
            power = 5;
        }
        if (power > 20) {
            lora_write_single(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE, 1);//normal
            power -= 3;
        } else {
            lora_write_single(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE, 1); // 57 CD 01 04//normal
        }
        lora_write_single(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power - 5), 1); // 57 89 01 88//normal

        #ifdef USE_CUSTOM_SETTINGS

        //lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, BANDWIDTH | CRC_ON | CODING_RATE | RH_RF95_IMPLICIT_HEADER_MODE_ON); // 57 9D 01 1E updated for implicit header
        //lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, SPREADING_FACTOR | RH_RF95_AGC_AUTO_ON); //last 57 9E 01 C4

        //new config based on what should happen according to the manual
        lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, 0x70 | 0x08 | 0x01, 1); // 57 9D 01 79 updated for implicit header
        lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, 0xC0  | 0x04, 1); //last 57 9E 01 C4
        //end
        lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, MESH_MAX_MESSAGE_LENGTH, 1); //57 A2 01 06      update regpayload length (for implicit header mode only)
//        change the above line to the max length, it should be fine if it is longer than the actual message length since
//		the sending one will just send garbage and the rec one just won't read it'
        lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //norm
        return true;
        #endif

        #ifdef USE_DEFAULT_SETTINGS
        //setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
        if (MODEM_CONFIG_CHOICE > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig))) {
            return false;
        }

        ModemConfig cfg;
        memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
        setModemRegisters(&cfg);
        return true;
        #endif
        lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //norm
    }


uint8_t lora_read_fifo_single(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING ONE BYTE OF A LORA MESSAGE,
    uint8_t val = 0;
    uart_write_rx('R'); //0x52
    uart_write_rx(0X00 & ~RH_WRITE_MASK); //0x00 & ~0x80, so 0x00
    uart_write_rx(1); //0x01
    lora_dma_write_send(sendfifo_offset_rec, hdma_usart_tx, huart, 2, 1); //3, rec
    val = uart_read();
    return val;
    // 52, 00, 01
}

void lora_dma_write_send(int length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t send_type, bool re_enable){
    //This enables the message send for the LoRa's DMA
	//send type is 1 for normal, 2 for rx, 3 for tx

	while((send_normal == true) | (send_send == true) | (send_rec == true)){
		//wait until all sends are complete before doing another one
	}

	HAL_NVIC_DisableIRQ(TIM21_IRQn); //disable all interrupts that could use the Lora DMA
	HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); //disable all interrupts that could use the Lora DMA
	HAL_NVIC_DisableIRQ(USART2_IRQn); //disable all interrupts that could use the Lora DMA
	HAL_StatusTypeDef status;


	if(send_type == 1){ //normal
		status = HAL_UART_Transmit_DMA(&huart, sendfifo_norm, length);
		sendfifo_ready_norm = false;
//		if(global_receive_mode_from_cad == 1){//remove, this is for testing
//			//do nothing
//			global_receive_mode_from_cad = 1;
//		}
		//HAL_DMA_GetState(&hdma_usart_tx);
		sendfifo_offset_norm = 0;
		send_normal = true;
	}
	else if(send_type == 2){//rx
		HAL_UART_Transmit_DMA(&huart, sendfifo_rec_message, length);
		sendfifo_ready_rec = false;
//		if(global_receive_mode_from_cad == 1){//remove, this is for testing
//			//do nothing
//			global_receive_mode_from_cad = 1;
//		}
		//HAL_DMA_GetState(&hdma_usart_tx);
		sendfifo_offset_rec = 0;
		send_rec = true;
	}
	else if (send_type == 3){//tx
		HAL_UART_Transmit_DMA(&huart, sendfifo_send_message, length);
		sendfifo_ready_send = false;
//		if(global_receive_mode_from_cad == 1){//remove, this is for testing
//			//do nothing
//			global_receive_mode_from_cad = 1;
//		}
		//HAL_DMA_GetState(&hdma_usart_tx);
		sendfifo_offset_send = 0;
		send_send = true;
	}
	else{
		while (true){
			//error
		}
	}
	HAL_NVIC_EnableIRQ(USART2_IRQn); //enable all interrupts that could use the Lora DMA
	if(re_enable){
		HAL_NVIC_EnableIRQ(TIM21_IRQn); //enable all interrupts that could use the Lora DMA
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); //enable all interrupts that could use the Lora DMA
	}
}

void set_mode_continuous_receive(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS, 1); // 57, 81, 01, 05
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00, 1); // 57 C0 01 00
    lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal
}
void set_mode_sleep(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){/////////////////////////////////////maybe or values | RH_RF95_LONG_RANGE_MODE to put in LoRa mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP, 1);
    lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal
}

void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length, uint8_t message_type){//done, not tested
    //THIS IS FOR WRTING TO REGISTERS IN THE LORA MICRO, NOT SENDING A LORA MESSAGE
    //writes value to the address specified in reg
    //reg is in the LoRa microcontroller
    //length is the number of bytes written
	//message type: 1: norm, 2: rx, 3: tx, has to be one of them
	if(message_type == 1){
		uart_write_normal('W');
		uart_write_normal(reg | RH_WRITE_MASK);
		uart_write_normal(length);
		for (int i = 0; i < length; i ++) {
			uart_write_normal(*(value + i));
		}
	}
	else if(message_type == 2){
		uart_write_rx('W');
		uart_write_rx(reg | RH_WRITE_MASK);
		uart_write_rx(length);
		for (int i = 0; i < length; i ++) {
			uart_write_rx(*(value + i));
		}
	}
	else if(message_type == 3){
		uart_write_tx('W');
		uart_write_tx(reg | RH_WRITE_MASK);
		uart_write_tx(length);
		for (int i = 0; i < length; i ++) {
			uart_write_tx(*(value + i));
		}
	}
}


void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t send_type){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg and places it in result
    //reg is in the LoRa microcontroller
    //length is the number of bytes to read
	//send type is 1 for normal, 2 for rx, 3 for tx
    if(send_type == 1){
    	uart_write_normal('R');
    	uart_write_normal(reg & ~RH_WRITE_MASK);
    	uart_write_normal(length);
    	lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //3
    }
    else if(send_type == 2){
    	uart_write_rx('R');
    	uart_write_rx(reg & ~RH_WRITE_MASK);
    	uart_write_rx(length);
    	lora_dma_write_send(sendfifo_offset_rec, hdma_usart_tx, huart, 2, 1); //3
    }
    else if(send_type == 3){
    	uart_write_tx('R');
    	uart_write_tx(reg & ~RH_WRITE_MASK);
    	uart_write_tx(length);
    	lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, 1); //3
    }

    int i = 0;
    while (1) {
        //available means uart is not currently reading a message, figure out how to do this
        *(result + i) = uart_read();
        i ++;
        if (i >= length) {
            break;
        }
    }
}

void lora_write_single(uint8_t reg, uint8_t value, uint8_t message_type){//done, not tested
    //THIS IS FOR WRTING TO REGISTERS IN THE LORA MICRO, NOT SENDING A LORA MESSAGE
    //writes value to the address specified in reg
    //reg is in the LoRa microcontroller
	//1 is normal, 2 is rx, 3 is tx
	if(message_type == 1){
		uart_write_normal('W'); //0x57
		uart_write_normal(reg | RH_WRITE_MASK); // try 00 | 80 = 80
		uart_write_normal(1);
		uart_write_normal(value);
	}
	else if(message_type == 2){
		uart_write_rx('W'); //0x57
		uart_write_rx(reg | RH_WRITE_MASK); // try 00 | 80 = 80
		uart_write_rx(1);
		uart_write_rx(value);
	}
	else if(message_type == 3){
		uart_write_tx('W'); //0x57
		uart_write_tx(reg | RH_WRITE_MASK); // try 00 | 80 = 80
		uart_write_tx(1);
		uart_write_tx(value);
	}
}

bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done, not tested
    //outputs rxdone, valid_header, and crc_error flags after reading them
    //THIS ALSO CLEARS THE FLAG REGISTER if clear == 1

    //read irq flag reg (12)
    uint8_t value = 0;
    //value = lora_read_single(0x01);
    //
    while ((value == 0x0) | (value == 0x80)){
        value = lora_read_single(0x12, hdma_usart_tx, huart, 1, 1); //norm
        //set
        *(rxdone) = (value >> 6) & 0x1;
        *(valid_header) = (value >> 4) & 0x1;
        *(crc_error) = (value >> 5) & 0x1;
        HAL_Delay(100); //in ms
    }
    //clear
    if(clear){
            lora_write_single(0x12, 0xFF, 1);
            lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal
        }
    return true;
}

void lora_read_fifo_all(uint8_t* data, uint8_t length, bool clear_header, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING THE ENTIRE LORA MESSAGE, NO HEADERS
    //LENGTH IS WITHOUT HEADERS
    uint8_t start_addr = 0;
    uint8_t datab = 0;
    //start_addr = lora_read_single(0x10);//read start addr of last packet received
    //for some reason 0x10 does not have the correct addr, receive correct values when this is commented out
    datab = 0;

    if(clear_header){
    	lora_write_single(0x0D, start_addr, 2);//set FIFO pointer to addr of last packet received
    	lora_dma_write_send(sendfifo_offset_rec, hdma_usart_tx, huart, 2, 1); //rec only

    	for (int i = 0; i < 4; i ++) {
    		datab = lora_read_fifo_single(hdma_usart_tx, huart); //read the headers, but don't store them
    	}
    }

    for (int i = 0; i < length; i ++) {
        datab = lora_read_fifo_single(hdma_usart_tx, huart);
        *(data + i) = datab; //read one byte of the message
        //start_addr = 0; //for testing
    }
}

bool connected_test(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    //returns true if LoRa module is connected and false if not

    uint8_t counter = 0;
    uint8_t value = 0;
    bool done = false;
    doing_connected_test = true;
    while(done == false){
        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE), 1); // 57 81 01 80
        lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal

        value = lora_read_single(0x01, hdma_usart_tx, huart, 1, 1);//check irq register for done, norm
        if(value == 0x80){//==0x80
            //GPIOC->ODR = 0;//testing
        	doing_connected_test = false;
            return true;
        }
        else{
        	HAL_Delay(10);
            counter += 1;
            //GPIOC->ODR = 1;//testing
            // if(counter > 10){ //5 seconds
            //     return false;
            // }
        }
    }
    return false;
 }


uint8_t uart_read(){ //not done (add timeout logic), not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR READING DATA SENT FROM THE LORA MICRO USING UART
    //for reading received lora messages
    //USE lora_receive instead
    uint8_t c = 0;
    // int counter = 0;
    // //UART_READ has to have timeout logic like in uartRx in RHUartDriver.cpp
    // while (!(USART5->ISR & USART_ISR_RXNE)) {
    //     c = USART5->RDR;
    //     nano_wait(1000000); //wait 1/1000 second
    //     counter += 1;
    //     if(counter >= 10000){
    //         return 0x0;
    //     }
    // }
    // c = USART5->RDR;

    //changes for DMA
//    HAL_Delay(1000);
//    //nano_wait(500000000000); //wait 0.5 seconds
//    c = receivefifo[receivefifo_offset];
//    receivefifo[receivefifo_offset] = 0;

    //changes for HAL
//    HAL_Delay(100);
//    if(rx_ready == true){
//    	c = receivefifo[0];
//    	receivefifo[0] = 0;
//    }
//    else{
//    	rx_ready = false;
//    	//
//    }
//    rx_ready = false;

    //HAL_Delay(100);dma
    while(rx_ready == false){
    	if((receivefifo[0] == 0x49) & doing_send){ //automatic I response
    		c = receivefifo[0];
    		receivefifo[0] = 0;
    		rx_ready = false;
    		return c;
    	}
//    	if(receivefifo[0] == 0x00){
//    		rx_ready = true;
//    	}
    	if(doing_connected_test){
    		if(receivefifo[0] == 0x00){
    		    rx_ready = true;
    		}
    	}
    }
    HAL_Delay(1);
    //HAL priority needs to be set higher to be used in an interrupt
    c = receivefifo[0];
    receivefifo[0] = 0;
    rx_ready = false;
    return c;
}

void uart_write_normal(uint8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead
	//this is for normal writes only (no message sending or FIFO reads)

    int counter = 0;

    while(sendfifo_ready_norm == false) {
        //nano_wait(1000000); //wait 1/1000 second
    	HAL_Delay(1);
        counter += 1;
        if(counter >= 10000){
            break;
        }
    	if(sendfifo_norm[sendfifo_offset_norm] == 0x0){
    		//if sendfifo[offset] == 0, then most likely it is available to be modified
    		sendfifo_ready_norm = true;
    	}
    }

    sendfifo_norm[sendfifo_offset_norm] = data;
    sendfifo_offset_norm += 1;
    if(sendfifo_offset_norm > FIFOSIZE_TX_NORM){
        sendfifo_offset_norm = 0;
        while(1){
        	//
        }
    }
}

void uart_write_rx(uint8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead
	//this is for FIFO reads only (no message sending or normal messages)

    int counter = 0;

    while(sendfifo_ready_rec == false) {
        //nano_wait(1000000); //wait 1/1000 second
    	HAL_Delay(1);
        counter += 1;
        if(counter >= 10000){
            break;
        }
    	if(sendfifo_rec_message[sendfifo_offset_rec] == 0x0){
    		//if sendfifo[offset] == 0, then most likely it is available to be modified
    		sendfifo_ready_rec = true;
    	}
    }

    sendfifo_rec_message[sendfifo_offset_rec] = data;
    sendfifo_offset_rec += 1;
    if(sendfifo_offset_rec > FIFOSIZE_TX_REC){
    	sendfifo_offset_rec = 0;
    	while(1){
    	        	//
    	        }
    }
}


void uart_write_tx(uint8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead
	//this is for send writes only (no normal messages or FIFO reads)

    int counter = 0;

    while(sendfifo_ready_send == false) {
        //nano_wait(1000000); //wait 1/1000 second
    	HAL_Delay(1);
        counter += 1;
        if(counter >= 10000){
            break;
        }
    	if(sendfifo_send_message[sendfifo_offset_send] == 0x0){
    		//if sendfifo[offset] == 0, then most likely it is available to be modified
    		sendfifo_ready_send = true;
    	}
    }

    sendfifo_send_message[sendfifo_offset_send] = data;
    sendfifo_offset_send += 1;
    if(sendfifo_offset_send > FIFOSIZE_TX_SEND){
        sendfifo_offset_send = 0;
        while(1){
                	//
                }
    }
}

uint8_t lora_read_single(uint8_t reg, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart, uint8_t message_type, bool re_enable){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg
    //reg is in the LoRa microcontroller
    uint8_t val = 0;
    if(message_type == 1){ //norm
    	uart_write_normal('R'); //0x52
    	uart_write_normal(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    	uart_write_normal(1); //0x01
    	lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, re_enable); //1
    }
    else if(message_type == 2){ //rx
    	uart_write_rx('R'); //0x52
    	uart_write_rx(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    	uart_write_rx(1); //0x01
    	lora_dma_write_send(sendfifo_offset_rec, hdma_usart_tx, huart, 2, re_enable); //2
    }
    else if(message_type == 3){ //tx
    	uart_write_tx('R'); //0x52
    	uart_write_tx(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    	uart_write_tx(1); //0x01
    	lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, re_enable); //3
    }

    val = uart_read();
    return val; //baud is 57600
    //worked with 52 0F 01
    // 52 00 01 read vale written by write
}

bool lora_send(uint8_t* data, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart) { //not done, not tested
    //THIS IS FOR SENDING A LORA MESSAGE, NOT WRTING TO REGISTERS IN THE LORA MICRO
    //this handles sending a lora message
    //length is the length of the message in bytes
    //data is the payload data being sent
    bool done = false;
    uint8_t counter = 0;
    uint8_t value = 0;
    doing_send = true;

    //this function will need to be updated
    if (length > RH_RF95_MAX_MESSAGE_LEN) {
    	doing_send = false;
        return false;
    }

    //this->waitPacketSent(); // Make sure we dont interrupt an outgoing message
    //setModeIdle();

//    HAL_Delay(100);
//    value = lora_read_single(0x12, hdma_usart_tx, huart, 3);
//    if(value != 0x0){
//    	lora_write_single(0x12, 0xff,3); // Clear all IRQ flags//tx
//    	lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3); //send
//    }
//    HAL_Delay(100);

    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY, 3); //new 57, 81, 01, 01 //tx
    //added
    lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, 1); //send
    //HAL_Delay(100);
    //end added
    //value = lora_read_single(RH_RF95_REG_01_OP_MODE);
    // while(value != 0x81){
    //     value = lora_read_single(RH_RF95_REG_01_OP_MODE);
    // }

    // Position at the beginning of the FIFO
    // 57, reg | 80, 01, value (2 hex)
    lora_write_single(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0, 3);// 57, 8d, 01, 00 (2 hex)//tx
    // while(value != 0){
    //     value = lora_read_single(RH_RF95_REG_0D_FIFO_ADDR_PTR);
    // }
    // //reg, value

    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x40, 3); // Interrupt on TxDone // 57, C0, 01, 40//tx
    // while(value != 0x40){
    //     value = lora_read_single(RH_RF95_REG_40_DIO_MAPPING1);
    // }

    // The headers
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRTO,3); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRFROM,3); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERID,3); // 57, 80, 01, 00
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERFLAGS,3); // 57, 80, 01, 00//tx

    //lora_write_multiple(RH_RF95_REG_00_FIFO, data, length); //57, 80, 02, F0, 0F //sends F0 0F
    for (int i = 0; i < length; i ++) {
        lora_write_single(RH_RF95_REG_00_FIFO, data[i], 3);//tx
    }


    // while(value != 99){
    //     value = lora_read_single(0x0E);
    // }

    //this apparently doesn't do anything/////////////////////////////////////////////////////////////////////////////////////////////TODO
    lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, (length + 4),3); //57 , A2, 01, 06//tx
    // while(value != (length + 4)){
    //     value = lora_read_single(RH_RF95_REG_22_PAYLOAD_LENGTH);
    // }

    // for (int i = 0;  i < 0x27; i++){
    //     value = lora_read_single(i);
    //     value = 0;
    //     //mode is standby
    //     //6:D9, 0, 0, 9:88, 10:09, 2B, 20, 13:7, pointer = 0, 0, 0, 17:0, 0, 0
    // }

    //change module to send mode
    // while (((USART5->ISR >> 4) & 0x1) == 0){ //wait until usart is idle
    //     nano_wait(500000000000000);
    //     value = uart_read();
    // }
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX,3);  // 57, 81, 01, 03//tx

    while(!((!global_receive_mode_from_cad) & (!read_lora_fifo))){
    	//do nothing
    }
    HAL_NVIC_DisableIRQ(TIM21_IRQn); //disable all interrupts that could use the Lora DMA
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); //disable all interrupts that could use the Lora DMA

    lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, 0); //send

    //logic to clear irq flags
    value = uart_read(); //this should wait until the I response and then let the code move to the actual read

    while(value != 0x49){
    	while(1){
    		//error
    	}
    }
    while(done == false){
        value = lora_read_single(0x12, hdma_usart_tx, huart, 3, 0);//check irq register for done (12), send
        //value = lora_read_single(RH_RF95_REG_01_OP_MODE);//check irq register for done (12) //remove, checking if in tx state
        if((value >> 3) & 0x1){//(value >> 3) & 0x1
        // value = lora_read_single(0x01);//check mode register for idle
        // if(value == 0x80){//
            done = true;
            lora_write_single(0x12, 0xff,3); // Clear all IRQ flags//tx
            lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, 1); //send
            doing_send = false;
            return true;
        }
        else{
        	if(value == 0x15){
        		while(true){
        			//
        		}
        	}
        	HAL_Delay(1000);
            counter += 1;
            if(counter > 50){ //5 seconds
                lora_write_single(0x12, 0xff,3); // Clear all IRQ flags//tx
                lora_dma_write_send(sendfifo_offset_send, hdma_usart_tx, huart, 3, 1); //send
                doing_send = false;
                return false;
            }
        }
    }
    doing_send = false;
    return true;
}

void set_mode_standby(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
	lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY,1);
	lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00,1);
	lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal
}

void set_mode_cad(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD | RH_RF95_LONG_RANGE_MODE,1);
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0xA0,1);
    lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //normal
}

bool cad_cycle(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done, not tested
    //THIS IS THE CAD CYCLE OF RECEIVING
    //returning true means go to continuous receive mode
    //returning false means go to sleep mode
	set_mode_standby(hdma_usart_tx, huart);
	HAL_Delay(10);

    set_mode_cad(hdma_usart_tx, huart); //go to cad mode (111)
    uint8_t done = 0;

    uart_read(); //this should wait until the I response and then let the code move to the actual read
    while(1){
    	//HAL_Delay(60); //this is used to prevent the first read from occurring before CAD is done //removed for the above uart_read
        done = lora_read_single(0x12, hdma_usart_tx, huart, 1, 1); //wait until reg 12-2 is high (CAD is done) //norm
//        if(done == 0x49){ //removed for the above uart_read
//        	//I response
//        	//done = lora_read_single(0x12, hdma_usart_tx, huart, 1); //get actual value //norm
//        	done = uart_read();
//        }
        if(((done >> 2) & 0x1)){
            if((done & 0x1)){//if 12-0 is high, return true
                lora_write_single(0x12, 0xFF,1); //clear irq flags
                lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //norm
                return true;
            }
            else{
                lora_write_single(0x12, 0xFF,1); //clear irq flags
                lora_dma_write_send(sendfifo_offset_norm, hdma_usart_tx, huart, 1, 1); //norm
                return false; //else return 0
            }
        }
        else{
            //maybe add a very small wait here to reduce power consumption
        }
    }
}

//one timer for LoRa mode changes
//calls cad_cycle with one length
//in interrupt, checks value and calls receive or sleep and resets timer
//cad_cycle sets mode to sleep and resets the timer

void change_lora_timer_period(int cause, TIM_HandleTypeDef * htim){
	//this is for changing the timer period for the lora cad cycle
	//if cause == 0, then it is the timer for the cad cycle
	//if cause == 1, then it is the timer for receive mode timeout

	//reset timer
	HAL_TIM_Base_DeInit(htim);
	//change period
	//input clock is APB2Tim_clock (currently 32 MHz)
	//period is 1/ (APB2Tim_clock / (Prescaler + 1) / (Period + 1))
	if(cause == 1){
		htim->Init.Prescaler = 10000;
		htim->Init.Period = 65535;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	}
	else if(cause == 0){
		htim->Init.Prescaler = 1000;//added an extra 0 for 2second period
		htim->Init.Period = 65535;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	}
	//update timer
	HAL_TIM_Base_Init(htim); //init
	htim->Instance->EGR |= TIM_EGR_UG; //manually trigger update event (loads new Prescaler and ARR values)
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE); //clears interrupt flag, htim.Instance->CR1 |= TIM_CR1_URS; // Interrupts only on overflow
	__HAL_TIM_SET_COUNTER(htim, 0); //set counter to 0
}

void setup_lora_send_timer(TIM_HandleTypeDef * htim, uint32_t lora_send_time){
	//this is for setting up the lora_send timer
	uint16_t prescaler;
	uint16_t period;
	uint32_t val;

	HAL_TIM_Base_Stop_IT(htim);
	HAL_TIM_Base_DeInit(htim);
	//input clock is APB2Tim_clock (currently 32 MHz)
	//use lora_send_time (in ms)
	//time * (APB2Tim_clock / (Prescaler + 1) / (Period + 1)) = 1
	// (time * APB2Tim_clock) = ((Prescaler + 1) * (Period + 1))

	val = lora_send_time / 100 * 32 * 1000000; // /100 is for ms conversion 10^6 is M
	period = 64 * 1000; //64000, almost max value
	prescaler = val / period;

	htim->Init.Prescaler = prescaler;
	htim->Init.Period = period;
	HAL_TIM_Base_Init(htim); //init
	htim->Instance->EGR |= TIM_EGR_UG; //manually trigger update event (loads new Prescaler and ARR values)
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE); //clears interrupt flag, htim.Instance->CR1 |= TIM_CR1_URS; // Interrupts only on overflow
	__HAL_TIM_SET_COUNTER(htim, 0); //set counter to 0
	HAL_TIM_Base_Start_IT(htim);
}

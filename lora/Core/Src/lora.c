#include <lora.h>
extern bool sendfifo_ready;
extern int sendfifo_offset;
extern bool rx_ready;
extern uint8_t sendfifo[FIFOSIZE_TX]; //array of data read from LoRa module
extern uint8_t receivefifo[FIFOSIZE_RX]; //array of data read from LoRa module
extern uint8_t global_receive_mode_from_cad;


 bool lora_init(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//not done, not tested
        //sets preamble length, center frequency, Tx power, and modem config
        // ALSO NEED TO SET ADDRESS of the node (needed depending on AddressFiltering register, but reg is 34)
        // (default is off)

        //write formula: 57, reg | 0x80, 01, value (2 hex)
        // //read formula: 52, reg & ~0x80, 01

        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE); // 57 81 01 80
        //lora_read_single(0x01);//testing

        //setup FIFO
        lora_write_single(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0); //57 8E 01 00
        lora_write_single(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0); //57 8F 01 00

        //set mode to IDLE
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY); // 57 81 01 01
        //lora_read_single(0x01);//testing

        //setPreambleLength Default is 8 bytes
        // 57, reg | 80, 01, value (2 hex)
        lora_write_single(RH_RF95_REG_20_PREAMBLE_MSB, PREAMBLE_LENGTH >> 8); // 57 A0 01 00
        lora_write_single(RH_RF95_REG_21_PREAMBLE_LSB, PREAMBLE_LENGTH & 0xff); // 57 A1 01 08 /////////////////////////////changed now

        //setFrequency(868.0);
        uint32_t frf = (CENTER_FREQUENCY * 1000000.0) / RH_RF95_FSTEP; //14,221,312, D9_00_00
        lora_write_single(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff); // 57 86 01 D9
        lora_write_single(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff); // 57 87 01 00 ///////////////////////////////changed now
        lora_write_single(RH_RF95_REG_08_FRF_LSB, frf & 0xff); // 57 88 01 00 /////////////////////////changed now

        //setTxPower(13);
        int8_t power = TXPOWER;
        if (power > 23) {
            power = 23;
        }
        if (power < 5) {
            power = 5;
        }
        if (power > 20) {
            lora_write_single(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
            power -= 3;
        } else {
            lora_write_single(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE); // 57 CD 01 04
        }
        lora_write_single(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power - 5)); // 57 89 01 88

        #ifdef USE_CUSTOM_SETTINGS

        //lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, BANDWIDTH | CRC_ON | CODING_RATE | RH_RF95_IMPLICIT_HEADER_MODE_ON); // 57 9D 01 1E updated for implicit header
        //lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, SPREADING_FACTOR | RH_RF95_AGC_AUTO_ON); //last 57 9E 01 C4

        //new config based on what should happen according to the manual
        lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, 0x70 | 0x08 | 0x01); // 57 9D 01 79 updated for implicit header
        lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, 0xC0  | 0x04); //last 57 9E 01 C4
        //end
        lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, 6); //57 A2 01 06      update regpayload length (for implicit header mode only)
        lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
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
        lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
    }


uint8_t lora_read_fifo_single(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING ONE BYTE OF A LORA MESSAGE,
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(0X00 & ~RH_WRITE_MASK); //0x00 & ~0x80, so 0x00
    uart_write(1); //0x01
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart); //3
    val = uart_read();
    return val;
    // 52, 00, 01
}

void lora_dma_write_send(int length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    //This enables the message send for the LoRa's DMA
    //set EN bit to send writes
	//old
//    DMA1_Channel2->CCR &= ~DMA_CCR_EN; //turn off DMA sending
//    DMA1_Channel2->CNDTR = length;//set CNDTR
//    while(DMA1_Channel2->CNDTR != (length)){
//        //do nothing while data is sending over DMA
//    }
//    DMA1_Channel2->CCR |= DMA_CCR_EN;
//    while(DMA1_Channel2->CNDTR != 0){
//        //do nothing while data is sending over DMA
//    }
//    //wait for transfer complete flag
//    while(((USART1->ISR >> 6) & 0x1) == 0){ //TC(bit 6)
//        //do nothing while data is sending
//    }
//    USART1->ICR |= (1 << 6); //clear TC
//    //clear sendfifo and reset offset
	//end old
	HAL_StatusTypeDef status;
	HAL_DMA_StateTypeDef dma_status;
	//uint32_t dma_estatus;
	//dma_status = HAL_DMA_GetState(&hdma_usart1_tx);
	status = HAL_UART_Transmit_DMA(&huart, sendfifo, length);
	sendfifo_ready = false;
	if(global_receive_mode_from_cad == 1){//remove, this is for testing
		//do nothing
		global_receive_mode_from_cad = 1;
	}
	dma_status = HAL_DMA_GetState(&hdma_usart_tx);
	sendfifo_offset = 0;
//	while(((USART1->ISR >> 6) & 0x1) == 0){ //TC(bit 6)
//	        //do nothing while data is sending
//	    }
	//dma_estatus = HAL_DMA_GetError(&hdma_usart1_tx);

//    for (int i = 0;  i < FIFOSIZE_TX; i++){
//        sendfifo[i] = 0;
//    }
//    sendfifo_offset = 0;
    //DMA1_Channel7->CNDTR = FIFOSIZE_TX;//set CNDTR
    //DMA1_Channel2->CCR &= ~DMA_CCR_EN; //turn off DMA sending
}

void set_mode_continuous_receive(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS); // 57, 81, 01, 05
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // 57 C0 01 00
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
}
void set_mode_sleep(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){/////////////////////////////////////maybe or values | RH_RF95_LONG_RANGE_MODE to put in LoRa mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
}

void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length){//done, not tested
    //THIS IS FOR WRTING TO REGISTERS IN THE LORA MICRO, NOT SENDING A LORA MESSAGE
    //writes value to the address specified in reg
    //reg is in the LoRa microcontroller
    //length is the number of bytes written
    uart_write('W');
    uart_write(reg | RH_WRITE_MASK);
    uart_write(length);
    for (int i = 0; i < length; i ++) {
        uart_write(*(value + i));
    }
    //lora_dma_write_send(sendfifo_offset); //3+length
}


void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg and places it in result
    //reg is in the LoRa microcontroller
    //length is the number of bytes to read
    uart_write('R');
    uart_write(reg & ~RH_WRITE_MASK);
    uart_write(length);
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart); //3

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

void lora_write_single(uint8_t reg, uint8_t value){//done, not tested
    //THIS IS FOR WRTING TO REGISTERS IN THE LORA MICRO, NOT SENDING A LORA MESSAGE
    //writes value to the address specified in reg
    //reg is in the LoRa microcontroller
    uart_write('W'); //0x57
    uart_write(reg | RH_WRITE_MASK); // try 00 | 80 = 80
    uart_write(1);
    uart_write(value);
    //57 80 01 FF //writes FF to addr 00 worked
    //lora_dma_write_send(sendfifo_offset); //4
}

bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done, not tested
    //outputs rxdone, valid_header, and crc_error flags after reading them
    //THIS ALSO CLEARS THE FLAG REGISTER if clear == 1

    //read irq flag reg (12)
    uint8_t value = 0;
    //value = lora_read_single(0x01);
    //
    while ((value == 0x0) | (value == 0x80)){
        value = lora_read_single(0x12, hdma_usart_tx, huart);
        //set
        *(rxdone) = (value >> 6) & 0x1;
        *(valid_header) = (value >> 4) & 0x1;
        *(crc_error) = (value >> 5) & 0x1;
        HAL_Delay(100); //in ms
    }
    //clear
    if(clear){
            lora_write_single(0x12, 0xFF);
            lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
        }
    return true;
}

void lora_read_fifo_all(uint8_t* data, uint8_t length, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING THE ENTIRE LORA MESSAGE, NO HEADERS
    //LENGTH IS WITHOUT HEADERS
    uint8_t start_addr = 0;
    uint8_t datab = 0;
    //start_addr = lora_read_single(0x10);//read start addr of last packet received
    //for some reason 0x10 does not have the correct addr, receive correct values when this is commented out
    lora_write_single(0x0D, start_addr);//set FIFO pointer to addr of last packet received
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
    datab = 0;

    for (int i = 0; i < 4; i ++) {
        datab = lora_read_fifo_single(hdma_usart_tx, huart); //read the headers, but don't store them
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
    while(done == false){
        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)); // 57 81 01 80
        lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);

        value = lora_read_single(0x01, hdma_usart_tx, huart);//check irq register for done
        if(value == 0x80){//==0x80
            //GPIOC->ODR = 0;//testing
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

    //HAL_Delay(100);
    while(rx_ready == false){
    	if(receivefifo[0] == 0x49){ //automatic I response
    		rx_ready = true;
    	}
    	else if(receivefifo[0] == 0x00){
    		rx_ready = true;
    	}
    }
    //HAL_Delay(1); had to remove since called in interrupt?
    //HAL priority needs to be set higher to be used in an interrupt
    c = receivefifo[0];
    receivefifo[0] = 0;
    rx_ready = false;
    return c;
}

void uart_write(uint8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead

    //non DMA
    int counter = 0;
    // while(!(USART5->ISR & USART_ISR_TXE)) {
    //     nano_wait(1000000); //wait 1/1000 second
    //     counter += 1;
    //     if(counter >= 10000){
    //         break;
    //     }
    // }
    // USART5->TDR = data;
    //wait and then set back to 0
    // nano_wait(1000000000); //wait 1/1000 second
    // USART5->TDR = 0x0;
    //end of non-DMA

    //changes for DMA
    //nano_wait(500000000000); //wait 0.5 seconds

    //commented the below out for CAD, should add something like it back later (has been added back now)
    while(sendfifo_ready == false) {
        //nano_wait(1000000); //wait 1/1000 second
    	HAL_Delay(1);
        counter += 1;
        if(counter >= 10000){
            break;
        }
    	if(sendfifo[sendfifo_offset] == 0x0){
    		//if sendfifo[offset] == 0, then most likely it is available to be modified
    		sendfifo_ready = true;
    	}
    }

    sendfifo[sendfifo_offset] = data;
    sendfifo_offset += 1;
    if(sendfifo_offset > FIFOSIZE_TX){
        sendfifo_offset = 0;
    }
}

uint8_t lora_read_single(uint8_t reg, DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg
    //reg is in the LoRa microcontroller
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    uart_write(1); //0x01
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart); //3
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

    //this function will need to be updated
    if (length > RH_RF95_MAX_MESSAGE_LEN) {
        return false;
    }

    //this->waitPacketSent(); // Make sure we dont interrupt an outgoing message
    //setModeIdle();
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY); //new 57, 81, 01, 01
    //value = lora_read_single(RH_RF95_REG_01_OP_MODE);
    // while(value != 0x81){
    //     value = lora_read_single(RH_RF95_REG_01_OP_MODE);
    // }

    // Position at the beginning of the FIFO
    // 57, reg | 80, 01, value (2 hex)
    lora_write_single(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);// 57, 8d, 01, 00 (2 hex)
    // while(value != 0){
    //     value = lora_read_single(RH_RF95_REG_0D_FIFO_ADDR_PTR);
    // }
    // //reg, value

    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone // 57, C0, 01, 40
    // while(value != 0x40){
    //     value = lora_read_single(RH_RF95_REG_40_DIO_MAPPING1);
    // }

    // The headers
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRTO); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRFROM); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERID); // 57, 80, 01, 00
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERFLAGS); // 57, 80, 01, 00

    //lora_write_multiple(RH_RF95_REG_00_FIFO, data, length); //57, 80, 02, F0, 0F //sends F0 0F
    for (int i = 0; i < length; i ++) {
        lora_write_single(RH_RF95_REG_00_FIFO, data[i]);
    }


    // while(value != 99){
    //     value = lora_read_single(0x0E);
    // }


    lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, (length + 4)); //57 , A2, 01, 06
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
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);  // 57, 81, 01, 03
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
    HAL_Delay(1000);
    // value = 0;
    // while((value == 0) | (value == 0x80)){
    //     value = lora_read_single(RH_RF95_REG_01_OP_MODE);//check irq register for done (12) //remove, checking if in tx state
    //     lora_read_single(0x12);
    //     nano_wait(500000000000000); //wait 0.5 seconds
    //     lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);  // 57, 81, 01, 03
    //     nano_wait(500000000000000); //wait 0.5 seconds
    // }
    // value = uart_read();

    // for (int i = 0;  i < 0x27; i++){
    //     value = lora_read_single(i);
    //     value = 0;
    //     //mode is sleep
    //     //6:D9, 0, 0, 9:88, 10:09, 2B, 20, 13:7, pointer = 0, 0, 0, 17:0, 0, 0
    //     //ONLY CHANGE AFTER SETTING MODE TO TX (OR TRYING TO) IS MODE CHANGES TO SLEEP
    // }

    // value = 0;
    // while (value == 0){
    //     nano_wait(500000000000000);
    //     value = uart_read();
    // }

    //logic to clear irq flags
    while(done == false){
        value = lora_read_single(0x12, hdma_usart_tx, huart);//check irq register for done (12)
        //value = lora_read_single(RH_RF95_REG_01_OP_MODE);//check irq register for done (12) //remove, checking if in tx state
        if((value >> 3) & 0x1){//(value >> 3) & 0x1
        // value = lora_read_single(0x01);//check mode register for idle
        // if(value == 0x80){//
            done = true;
            lora_write_single(0x12, 0xff); // Clear all IRQ flags
            lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
            return true;
        }
        else{
        	HAL_Delay(1000);
            counter += 1;
            if(counter > 50){ //5 seconds
                lora_write_single(0x12, 0xff); // Clear all IRQ flags
                lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
                return false;
            }
        }
    }
    return true;
}

void set_mode_cad(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD | RH_RF95_LONG_RANGE_MODE);
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0xA0);
    lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
}

bool cad_cycle(DMA_HandleTypeDef hdma_usart_tx, UART_HandleTypeDef huart){ //done, not tested
    //THIS IS THE CAD CYCLE OF RECEIVING
    //returning true means go to continuous receive mode
    //returning false means go to sleep mode

    set_mode_cad(hdma_usart_tx, huart); //go to cad mode (111)
    uint8_t done = 0;

    while(1){
    	HAL_Delay(100); //this is used to prevent the first read from occurring before CAD is done
        done = lora_read_single(0x12, hdma_usart_tx, huart); //wait until reg 12-2 is high (CAD is done)
        if(((done >> 2) & 0x1)){
            if((done & 0x1)){//if 12-0 is high, return true
                lora_write_single(0x12, 0xFF); //clear irq flags
                lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
                return true;
            }
            else{
                lora_write_single(0x12, 0xFF); //clear irq flags
                lora_dma_write_send(sendfifo_offset, hdma_usart_tx, huart);
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
	//period is 1/ (APB2Tim_clock / Prescaler / Period)
	if(cause == 1){
		htim->Init.Prescaler = 10000;
		htim->Init.Period = 65535;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	}
	else if(cause == 0){
		htim->Init.Prescaler = 0;
		htim->Init.Period = 65535;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	}
	//update timer
	HAL_TIM_Base_Init(htim);
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>
#include <lora_receive_node.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define RECEIVE_BOL //for receive mode
#define SEND_BOL //for send mode
#define RH_WRITE_MASK 0x80
#define PREAMBLE_LENGTH 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13
#define FIFOSIZE_RX 1 //number of bytes in a received message (always 1)
#define FIFOSIZE_TX 100 //max number of bytes in a sent message (not just one write, but an actual sent LoRa message)
#define ADDRTO 0x10 //address of message that should receive any sent message
#define ADDRFROM 0x10 //address of this node (should be same as ADDRTO)
#define HEADERID 0 //this is one of the lora headers, but don't know what it is
#define HEADERFLAGS 0 //this is one of the lora headers, but don't know what it is

#define MESSAGE_LENGTH 2 //length of the message without headers

//comment out one of the 2 below
#define USE_CUSTOM_SETTINGS
//#define USE_DEFAULT_SETTINGS

#ifdef USE_CUSTOM_SETTINGS
#define BANDWIDTH           RH_RF95_BW_125KHZ //values in RH_RF95.h
#define CODING_RATE         RH_RF95_CODING_RATE_4_8 //chart on page 24, values in RH_RF95.h
#define SPREADING_FACTOR    RH_RF95_SPREADING_FACTOR_4096CPS //chart on page 24, values in RH_RF95.h
#define CRC_ON              RH_RF95_RX_PAYLOAD_CRC_ON //values in RH_RF95.h, 0 if no CRC else RH_RF95_RX_PAYLOAD_CRC_ON for CRC

#endif

#ifdef USE_DEFAULT_SETTINGS
#define MODEM_CONFIG_CHOICE Bw125Cr45Sf128 //(info page 22 of datasheet) Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on
    //is preset combo for Bandwidth, coding rate, spreading factor, CRC on/off (not using this)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t sendfifo[FIFOSIZE_TX]; //array of data read from LoRa module
int sendfifo_offset = 0;
bool sendfifo_ready = true;

 bool lora_init(){//not done, not tested
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
        lora_dma_write_send(sendfifo_offset);
    }


uint8_t lora_read_fifo_single(){//done, not tested
    //THIS IS FOR READING ONE BYTE OF A LORA MESSAGE,
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(0X00 & ~RH_WRITE_MASK); //0x00 & ~0x80, so 0x00
    uart_write(1); //0x01
    lora_dma_write_send(sendfifo_offset); //3
    val = uart_read();
    return val;
    // 52, 00, 01
}

void lora_dma_write_send(int length){
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
	status = HAL_UART_Transmit_DMA(&huart1, sendfifo, length);
	sendfifo_ready = false;
	dma_status = HAL_DMA_GetState(&hdma_usart1_tx);
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

void set_mode_continuous_receive(){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS); // 57, 81, 01, 05
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // 57 C0 01 00
    lora_dma_write_send(sendfifo_offset);
}
void set_mode_sleep(){/////////////////////////////////////maybe or values | RH_RF95_LONG_RANGE_MODE to put in LoRa mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
    lora_dma_write_send(sendfifo_offset);
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


void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg and places it in result
    //reg is in the LoRa microcontroller
    //length is the number of bytes to read
    uart_write('R');
    uart_write(reg & ~RH_WRITE_MASK);
    uart_write(length);
    lora_dma_write_send(sendfifo_offset); //3

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

bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear){ //done, not tested
    //outputs rxdone, valid_header, and crc_error flags after reading them
    //THIS ALSO CLEARS THE FLAG REGISTER if clear == 1

    //read irq flag reg (12)
    uint8_t value = 0;
    //value = lora_read_single(0x01);
    //
    while ((value == 0x0) | (value == 0x80)){
        value = lora_read_single(0x12);
        //set
        *(rxdone) = (value >> 6) & 0x1;
        *(valid_header) = (value >> 4) & 0x1;
        *(crc_error) = (value >> 5) & 0x1;
        HAL_Delay(100); //in ms
    }
    //clear
    if(clear){
            lora_write_single(0x12, 0xFF);
            lora_dma_write_send(sendfifo_offset);
        }
    return true;
}

void lora_read_fifo_all(uint8_t* data, uint8_t length){//done, not tested
    //THIS IS FOR READING THE ENTIRE LORA MESSAGE, NO HEADERS
    //LENGTH IS WITHOUT HEADERS
    uint8_t start_addr = 0;
    uint8_t datab = 0;
    //start_addr = lora_read_single(0x10);//read start addr of last packet received
    //for some reason 0x10 does not have the correct addr, receive correct values when this is commented out
    lora_write_single(0x0D, start_addr);//set FIFO pointer to addr of last packet received
    lora_dma_write_send(sendfifo_offset);
    datab = 0;

    for (int i = 0; i < 4; i ++) {
        datab = lora_read_fifo_single(); //read the headers, but don't store them
    }

    for (int i = 0; i < length; i ++) {
        datab = lora_read_fifo_single();
        *(data + i) = datab; //read one byte of the message
        //start_addr = 0; //for testing
    }
}

bool connected_test(void){
    //returns true if LoRa module is connected and false if not

    uint8_t counter = 0;
    uint8_t value = 0;
    bool done = false;
    while(done == false){
        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)); // 57 81 01 80
        lora_dma_write_send(sendfifo_offset);

        value = lora_read_single(0x01);//check irq register for done
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

uint8_t receivefifo[FIFOSIZE_RX]; //array of data read from LoRa module
bool rx_ready = false;


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
    	//
    }
    HAL_Delay(1);
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
    while(sendfifo_ready == false) {
        //nano_wait(1000000); //wait 1/1000 second
    	HAL_Delay(1000);
        counter += 1;
        if(counter >= 10000){
            break;
        }
    }
    sendfifo[sendfifo_offset] = data;
    sendfifo_offset += 1;
    if(sendfifo_offset > FIFOSIZE_TX){
        sendfifo_offset = 0;
    }
}

uint8_t lora_read_single(uint8_t reg){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg
    //reg is in the LoRa microcontroller
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    uart_write(1); //0x01
    lora_dma_write_send(sendfifo_offset); //3
    val = uart_read();
    return val; //baud is 57600
    //worked with 52 0F 01
    // 52 00 01 read vale written by write
}

bool lora_send(uint8_t* data, uint8_t length) { //not done, not tested
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
    lora_dma_write_send(sendfifo_offset);
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
        value = lora_read_single(0x12);//check irq register for done (12)
        //value = lora_read_single(RH_RF95_REG_01_OP_MODE);//check irq register for done (12) //remove, checking if in tx state
        if((value >> 3) & 0x1){//(value >> 3) & 0x1
        // value = lora_read_single(0x01);//check mode register for idle
        // if(value == 0x80){//
            done = true;
            lora_write_single(0x12, 0xff); // Clear all IRQ flags
            lora_dma_write_send(sendfifo_offset);
            return true;
        }
        else{
        	HAL_Delay(1000);
            counter += 1;
            if(counter > 50){ //5 seconds
                lora_write_single(0x12, 0xff); // Clear all IRQ flags
                lora_dma_write_send(sendfifo_offset);
                return false;
            }
        }
    }
    return true;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin (GPIOC, 8, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin (GPIOC, 8, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_UART_Receive_DMA(&huart1, receivefifo, 1);
  connected_test();
  lora_init();
  //tx = C12, rx = D2
  //rxdone LED = C0, valid_header LED = C1, crc_error LED = C2 USE RESISTORS: 150 ohm
  //C3-10 are 8 bits for data
  //if(receive_bol){ //receiving
#ifdef RECEIVE_BOL
	  uint8_t rxdone = 0;
	  uint8_t valid_header = 0;
	  uint8_t crc_error = 0;
	  bool clear = true; //this clears irq registers (need to for this, else LEDs would never reset)
	  uint8_t data [MESSAGE_LENGTH];
	  bool good = false;
  //}
#endif
  //end receiving
  //else{//sending
#ifdef SEND_BOL
	  uint8_t data [MESSAGE_LENGTH];
	  data[0] = 0xF0;
	  data[1] = 0x0F;
	  bool good = false;
#endif
  //}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef RECEIVE_BOL
		  set_mode_continuous_receive();
		  HAL_Delay(1000);
		  //every 1 second, check irq flags (and clear)
		  good = check_irq_flags_receive(&rxdone,&valid_header, &crc_error, clear);
		  //set LEDs based on flags
		  if(good){
		  	//GPIOC->ODR = rxdone | (valid_header << 1) | (crc_error << 2);
		  	lora_read_fifo_all(data, 2); //get message from FIFO
		  }
		  else{
		  	//GPIOC->ODR = 0;
		  }
		  set_mode_sleep(); //this clears FIFO
		  set_mode_continuous_receive(); //this goes back to receive mode
#endif
	  //end receiving
#ifdef SEND_BOL
		  good = lora_send(data, 2);
		  if(good){
			  GPIOC->ODR = 1 | (1 << 1) | (0 << 2) | (data[0] << 3);
		  }
		  else{
		      GPIOC->ODR = 0 | (0 << 1) | (1 << 2) | (data[0] << 3);
		   }
		  HAL_Delay(1000);
		  GPIOC->ODR = 0;
		  HAL_Delay(1000);
		  lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags (can try adding this in send module as well)
		  //like reading register until get send done and then clear it
		  if(data[0] == 0xFF){
			  data[1] += 0x1;
		  }
		  data[0] += 0x1;
		  set_mode_sleep(); //this clears FIFO
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_power_GPIO_Port, LoRa_power_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LoRa_power_Pin */
  GPIO_InitStruct.Pin = LoRa_power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_power_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	//normal code
	//receivefifo[0] = 0;
	rx_ready = true;
	//then call receive again
	HAL_UART_Receive_DMA(&huart1, receivefifo, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	for (int i = 0;  i < FIFOSIZE_TX; i++){
	        sendfifo[i] = 0;
	    }
	    sendfifo_offset = 0;
	    sendfifo_ready = true;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

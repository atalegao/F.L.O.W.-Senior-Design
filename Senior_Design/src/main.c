// Place the pins of the serial adapter into the breadboard. (red chip)
// Connect Rx (Pin 2) of the serial adapter to PC12 of the devboard.
// Connect Tx (Pin 3) of the serial adapter to PD2 of the devboard.
// Connect GND on the devboard to the breadboard’s ground rail (blue).
// Connect GND (Pin 6) of the serial adapter to the breadboard’s ground rail (blue).
// Connect CTS (Pin 5) of the serial adapter to the breadboard’s ground rail (blue).

#include "stm32f0xx.h"
void internal_clock();

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

// void setup_serial(void)
// {
//     RCC->AHBENR |= 0x00180000;
//     GPIOC->MODER  |= 0x02000000;
//     GPIOC->AFR[1] |= 0x00020000;
//     GPIOD->MODER  |= 0x00000020;
//     GPIOD->AFR[0] |= 0x00000200;
//     RCC->APB1ENR |= 0x00100000;
//     USART5->CR1 &= ~0x00000001;
//     USART5->CR1 |= 0x00008000;
//     USART5->BRR = 0x340;
//     USART5->CR1 |= 0x0000000c;
//     USART5->CR1 |= 0x00000001;
// }


//#include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <RH_RF95.h>
#include <lora_receive_node.h>

#define RH_WRITE_MASK 0x80
#define PREAMBLE_LENGTH 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13
#define FIFOSIZE 16 //number of bytes in a message
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


void lora_uart_init(){ //done, not tested
    //setup UART for lora
    //this uses usart5, tx = C12, rx = D2
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    //configure PC12 to be USART5_TX (AF2)
    GPIOC->MODER |= 0x02000000; // pin 12 to 10 (alternate) 
    GPIOC->AFR[1] |= 0x2 << ((12-8)*4); //set alternate function to AF2 (USART5_TX) AFRH, shift 0x2 (meaning AF2) by (pin number -8) * 4
    
    //configure PD2 to be USART5_RX (AF2)
    GPIOD->MODER |= 0x00000020; // pin 2 to 10 (alternate) 
    GPIOD->AFR[0] |= 0x2 << (2 * 4); //set alternate function to AF2 (USART5_RX) 0x2 means AF2 for low, shift by pin number * 4

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5->CR1 &= ~USART_CR1_UE; //turn of USART5 UE bit
    USART5->CR1 &= ~USART_CR1_M0; //change word size to 8 bits (00)
    USART5->CR1 &= ~USART_CR1_M1; //change word size to 8 bits (00)

    USART5->CR2 &= ~USART_CR2_STOP_0; //one stop bit
    USART5->CR2 &= ~USART_CR2_STOP_1; //one stop bit

    USART5->CR1 &= ~USART_CR1_PCE; //no parity control
    USART5->CR1 &= ~USART_CR1_OVER8; //16x oversampling
    USART5->BRR = 0x341; //baud rate (table96) needs to be 57600 for LoRa (THIS WAS CHANGED)
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE; //enable TE and RE 
    USART5->CR1 |= USART_CR1_UE; //enable USART

    //wait for TE and RE bits to be acknowledged
    while(((USART5->ISR & USART_ISR_TEACK) != USART_ISR_TEACK) | ((USART5->ISR & USART_ISR_REACK) != USART_ISR_REACK)){
        //nothing
    }
    // setbuf(stdin,0);
    // setbuf(stdout,0);
    // setbuf(stderr,0);
}

void uart_init_for_print(){ 
    //setup UART for printing to terminal
    //this uses usart5, tx = C12, rx = D2
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    //configure PC12 to be USART5_TX (AF2)
    GPIOC->MODER |= 0x02000000; // pin 12 to 10 (alternate) 
    GPIOC->AFR[1] |= 0x2 << ((12-8)*4); //set alternate function to AF2 (USART5_TX) AFRH, shift 0x2 (meaning AF2) by (pin number -8) * 4
    
    //configure PD2 to be USART5_RX (AF2)
    GPIOD->MODER |= 0x00000020; // pin 2 to 10 (alternate) 
    GPIOD->AFR[0] |= 0x2 << (2 * 4); //set alternate function to AF2 (USART5_RX) 0x2 means AF2 for low, shift by pin number * 4

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5->CR1 &= ~USART_CR1_UE; //turn of USART5 UE bit
    USART5->CR1 &= ~USART_CR1_M0; //change word size to 8 bits (00)
    USART5->CR1 &= ~USART_CR1_M1; //change word size to 8 bits (00)

    USART5->CR2 &= ~USART_CR2_STOP_0; //one stop bit
    USART5->CR2 &= ~USART_CR2_STOP_1; //one stop bit

    USART5->CR1 &= ~USART_CR1_PCE; //no parity control
    USART5->CR1 &= ~USART_CR1_OVER8; //16x oversampling
    USART5->BRR = 0x341; //baud rate (table96) needs to be 57600 for LoRa (THIS WAS CHANGED)
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE; //enable TE and RE 
    USART5->CR1 |= USART_CR1_UE; //enable USART

    //wait for TE and RE bits to be acknowledged
    while(((USART5->ISR & USART_ISR_TEACK) != USART_ISR_TEACK) | ((USART5->ISR & USART_ISR_REACK) != USART_ISR_REACK)){
        //nothing
    }
    // setbuf(stdin,0);
    // setbuf(stdout,0);
    // setbuf(stderr,0);
}

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

    }


    uint8_t lora_read_fifo_single(){//done, not tested
    //THIS IS FOR READING ONE BYTE OF A LORA MESSAGE, 
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(0X00 & ~RH_WRITE_MASK); //0x00 & ~0x80, so 0x00
    uart_write(1); //0x01
    val = uart_read();
    return val; 
    // 52, 00, 01
}

void set_mode_continuous_receive(){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS); // 57, 81, 01, 05
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // 57 C0 01 00
}
void set_mode_sleep(){/////////////////////////////////////maybe or values | RH_RF95_LONG_RANGE_MODE to put in LoRa mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
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
}


void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg and places it in result
    //reg is in the LoRa microcontroller
    //length is the number of bytes to read 
    uart_write('R');
    uart_write(reg & ~RH_WRITE_MASK);
    uart_write(length);

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
}

uint8_t lora_read_single(uint8_t reg){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg
    //reg is in the LoRa microcontroller
    uint8_t val = 0;
    uart_write('R'); //0x52
    uart_write(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    uart_write(1); //0x01
    val = uart_read();
    return val; //baud is 57600
    //worked with 52 0F 01
    // 52 00 01 read vale written by write
}

uint8_t uart_read(){ //not done (add timeout logic), not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR READING DATA SENT FROM THE LORA MICRO USING UART 
    //for reading received lora messages
    //USE lora_receive instead
    uint8_t c = 1;
    int counter = 0;
    //UART_READ has to have timeout logic like in uartRx in RHUartDriver.cpp
    while (!(USART5->ISR & USART_ISR_RXNE)) { 
        c = USART5->RDR;
        nano_wait(1000000); //wait 1/1000 second
        counter += 1;
        if(counter >= 10000){
            return 0x0;
        }
    }
    c = USART5->RDR;
    return c;
}


void uart_write(uint8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead
    int counter = 0;
    while(!(USART5->ISR & USART_ISR_TXE)) { 
        nano_wait(1000000); //wait 1/1000 second
        counter += 1;
        if(counter >= 10000){
            break;
        }
    }
    USART5->TDR = data;
    //wait and then set back to 0
    // nano_wait(1000000000); //wait 1/1000 second
    // USART5->TDR = 0x0;
}
//new functions//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        nano_wait(1000000000); //wait 1 second
    }
    //clear 
    if(clear){
            lora_write_single(12, 0xFF);
        }
    return true;
}

void lora_read_fifo_all(uint8_t* data, uint8_t length){//done, not tested
    //THIS IS FOR READING THE ENTIRE LORA MESSAGE, NO HEADERS
    //LENGTH IS WITHOUT HEADERS
    uint8_t start_addr = 0;
    start_addr = lora_read_single(10);//read start addr of last packet received
    lora_write_single(0x0D, start_addr);//set FIFO pointer to addr of last packet received

    for (int i = 0; i < 4; i ++) {
        lora_read_fifo_single(); //read the headers, but don't store them
    }

    for (int i = 0; i < length; i ++) {
        *(data + i) = lora_read_fifo_single(); //read one byte of the message
    }
}
 void setup_leds(void)
 {
     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
     GPIOC->MODER |= 0x00000015; //set pins 0-2 as output 01 //for 3 irq registers
     GPIOC->MODER |= 0x00155540; //set pins 3-10 as output 01 //for 8 data bits
 }

  bool connected_test(void){
    //returns true if LoRa module is connected and false if not

    uint8_t counter = 0;
    uint8_t value = 0;
    bool done = false;
    while(done == false){
        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)); // 57 81 01 80

        value = lora_read_single(0x01);//check irq register for done
        if(value == 0x80){//==0x80
            GPIOC->ODR = 0;//testing
            return true;
        }
        else{
            nano_wait(500000000); //wait 0.5 seconds
            counter += 1;
            GPIOC->ODR = 1;//testing
            // if(counter > 10){ //5 seconds
            //     return false;
            // }
        }
    }
 }
//init same hex as terminal
//continuous receive also same hex as terminal
//does go to continuous receive mode

//THIS DETECTS MESSAGES, BUT INTERRUPT REG HAS VALUE 49: RxDONE (GOOD), TxDONE (BAD), CAD_DETECTED(BAD)
int main(void){
    internal_clock(); 
    setup_leds();
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x00000001; //set pins 10 as output 01 //for 8power to LoRa module
    GPIOA->ODR = 0;
    GPIOC->ODR = 0;
    nano_wait(500000000000); //wait 0.5 seconds
    GPIOA->ODR = 1;
    nano_wait(500000000000); //wait 0.5 seconds
    lora_uart_init();
    connected_test();
    lora_init();
    //tx = C12, rx = D2
    //rxdone LED = C0, valid_header LED = C1, crc_error LED = C2 USE RESISTORS: 150 ohm
    //C3-10 are 8 bits for data
    uint8_t rxdone = 0;
    uint8_t valid_header = 0;
    uint8_t crc_error = 0;
    bool clear = true; //this clears irq registers (need to for this, else LEDs would never reset)
    uint8_t data [MESSAGE_LENGTH];
    bool good = false;
    for(;;) {
        set_mode_continuous_receive();
        nano_wait(1000000000); //wait 1 second
        //every 1 second, check irq flags (and clear)
        good = check_irq_flags_receive(&rxdone,&valid_header, &crc_error, clear);
        //set LEDs based on flags
        if(good){
            GPIOC->ODR = rxdone | (valid_header << 1) | (crc_error << 2);
            //could also use BRR and BSRR
            printf("Rxdone = %d\n", rxdone); //will need to setup another uart to get to work with USB to TTL, 
            printf("valid_header = %d\n", valid_header); // might also be a pain to setup printf by itself
            printf("crc_error = %d\n", crc_error);

            lora_read_fifo_all(data, MESSAGE_LENGTH); //get message from FIFO

            for (int i = 0; i < MESSAGE_LENGTH; i ++) {
                printf("data[%d] = %d\n", i, data[i]);
            }
            // set data LEDs based on data
            GPIOC->ODR |= (0xFF << 3);//(data[0] << 3);
        }
        else{
            GPIOC->ODR = 0;
        }
        set_mode_sleep(); //this clears FIFO
        set_mode_continuous_receive(); //this goes back to receive mode
    }
}
// int main(void)
// {
//     internal_clock();   // Never comment this out!
//     //setup_serial(); //362 function to send commands to micro from terminal
//     while(1) {
//         if ((USART5->ISR & USART_ISR_RXNE))  // if receive buffer has some data in it
//             USART5->TDR = USART5->RDR;       // copy that data into transmit buffer.
//     }
// }
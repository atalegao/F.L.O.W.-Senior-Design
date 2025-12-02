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
#include <lora_send_node.h>

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

bool lora_init(){//not done, not tested
        //sets preamble length, center frequency, Tx power, and modem config
        // ALSO NEED TO SET ADDRESS of the node (needed depending on AddressFiltering register, but reg is 34)
        // (default is off)

        //write formula: 57, reg | 0x80, 01, value (2 hex)
        // //read formula: 52, reg & ~0x80, 01

        //set mode to LORA sleep
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE); // 57 81 01 80
        lora_read_single(0x01);//testing

        //setup FIFO
        lora_write_single(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0); //57 8E 01 00
        lora_write_single(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0); //57 8F 01 00

        //set mode to IDLE
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY); // 57 81 01 01
        lora_read_single(0x01);//testing

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
    uart_write(0x57);
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
    uart_write(0x52);
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
    uart_write(0x57); //0x57
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
    uart_write(0x52); //0x52
    uart_write(reg & (~RH_WRITE_MASK)); //try 0x0F & ~0x80, so 0x0F
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
    value = lora_read_single(RH_RF95_REG_12_IRQ_FLAGS);

    // Position at the beginning of the FIFO
    // 57, reg | 80, 01, value (2 hex)
    lora_write_single(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);// 57, 8d, 01, 00 (2 hex)
    //reg, value

    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone // 57, C0, 01, 40

    // The headers
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRTO); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, ADDRFROM); // 57, 80, 01, 10
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERID); // 57, 80, 01, 00
    lora_write_single(RH_RF95_REG_00_FIFO, HEADERFLAGS); // 57, 80, 01, 00

    lora_write_multiple(RH_RF95_REG_00_FIFO, data, length); //57, 80, 02, F0, 0F //sends F0 0F
    lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, 6); //57 , A2, 01, 06

    //change module to send mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);  // 57, 81, 01, 03
    //changing mode to tx breaks reading (never get a response)
    //lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone // 57, C0, 01, 40
    // value = lora_read_single(RH_RF95_REG_01_OP_MODE);//for testing
    //value = uart_read(); //testing 
    nano_wait(50000000000000); //wait 0.5 seconds

    //logic to clear irq flags
    while(done == false){
        value = lora_read_single(RH_RF95_REG_12_IRQ_FLAGS);//check irq register for done (12)
        if(value == 0x08){//(value >> 3) & 0x1
        // value = lora_read_single(0x01);//check mode register for idle
        // if(value == 0x80){//
            done = true;
            lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
            return true;
        }
        else{
            nano_wait(5000000000); //wait 0.5 seconds
            counter += 1;
            if(counter > 50){ //5 seconds
                lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
                return false;
            }
        }
    }
    return true;
}

 void setup_leds(void)
 {
     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
     GPIOC->MODER |= 0x00000015; //set pins 0-2 as output 01 //for send notification
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

//send is exact same message in hex
//init is exact same message in hex
int main(void){
    //send a message every 5 seconds
    //set up an LED to flash whenever a message is sent and also display the number of 8 other LEDs
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
    //set pins C0-2 for send notification
    //set pins C3-9 for 8 data bits
    //tx = D2 rx = C12, (connections on LoRa module)
    //A0 is power to LoRa module
    //module needs to be unpowered to lose all settings, so set its power to a pin that only goes high after uart and clock setup
    //this could be why it is getting stuck in connected_test, but not in uart_read (getting a value, just 0 since it is already setup)
    uint8_t data [MESSAGE_LENGTH];
    data[0] = 0xF0;
    data[1] = 0x0F;
    bool good = false;
    for(;;) {
        good = lora_send(data, 2);
        printf("Message sent data = %d_%d", data[1], data[0]);
        if(good){
            GPIOC->ODR = 1 | (1 << 1) | (0 << 2) | (data[0] << 3);
        }
        else{
            GPIOC->ODR = 0 | (0 << 1) | (1 << 2) | (data[0] << 3);
        }
        nano_wait(45000000000); //wait 4.5 seconds
        GPIOC->ODR = 0;
        nano_wait(5000000000); //wait 0.5 seconds
        lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags (can try adding this in send module as well)
        //like reading register until get send done and then clear it
        if(data[0] == 0xFF){
            data[1] += 0x1;
        }
        data[0] += 0x1;
        set_mode_sleep(); //this clears FIFO
    }
}

void initb() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= 0xFFFFFFFC; //setting pin B0  to input (00) while not changing other pins
}

// int main(void){
//     //send a message every time a button is pressed
//     //set up an LED to flash whenever a message is sent and also display the number of 8 other LEDs
//     internal_clock();
//     lora_uart_init(); 
//     initb();
//     //setting pin B0  to input (00) 
//     setup_leds();
//     //set pins C0-2 for send notification
//     //set pins C3-10 for 8 data bits
//     //tx = C12, rx = D2
//     uint8_t data [MESSAGE_LENGTH];
//     data[0] = 0x1;
//     data[1] = 0x0;
//     for(;;) {
//         while(GPIOB->IDR & 0x00000001 == 0){
//             //do nothing while button is not pressed
//         }
//         lora_send(data, MESSAGE_LENGTH);
//         printf("Message sent data = %d_%d", data[1], data[0]);
//         GPIOC->ODR = 1 | (1 << 1) | (1 << 2) | (data[0] << 3);
//         nano_wait(4500000000); //wait 4.5 seconds
//         GPIOC->ODR = 0;
//         nano_wait(500000000); //wait 0.5 seconds
//         lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags (can try adding this in send module as well)
//         //like reading register until get send done and then clear it
//         if(data[0] == 0xFF){
//             data[1] += 0x1;
//         }
//         data[0] += 0x1;
//         set_mode_sleep(); //this clears FIFO
//     }
// }

// int main(void)
// {
//     internal_clock();   // Never comment this out!
//     //setup_serial(); //362 function to send commands to micro from terminal
//     while(1) {
//         if ((USART5->ISR & USART_ISR_RXNE))  // if receive buffer has some data in it
//             USART5->TDR = USART5->RDR;       // copy that data into transmit buffer.
//     }
// }
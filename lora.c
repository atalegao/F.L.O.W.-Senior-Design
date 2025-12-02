#include "stm32f0xx.h"

#define RH_WRITE_MASK 0x80
#define PREAMBLE_LENGTH 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13
#define FIFOSIZE 16 //number of bytes in a message
#define ADDRTO 10 //address of message that should receive any sent message
#define ADDRFROM 10 //address of this node (should be same as ADDRTO)
#define HEADERID 0 //this is one of the lora headers, but don't know what it is
#define HEADERFLAGS 0 //this is one of the lora headers, but don't know what it is

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

//lora chip (not module) datasheet:
// https://files.seeedstudio.com/wiki/Grove_LoRa_Radio/res/RFM95_96_97_98_DataSheet.pdf
//FIFO buffer pg 61, 30
//operation modes 31
//single receive 35
//getting data from FIFO 36, 70
//CAD mode (might be better than single receive) 39
// FIFO clearing conditions pg 62
//pg 84 is register addresses
//example code:
// https://github.com/Seeed-Studio/Grove_LoRa_433MHz_and_915MHz_RF/blob/master/RH_RF95.cpp

//LoRa module datasheet (not useful)
//https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2527/113060006_Web.pdf
//chip LoRa chip is based on datasheet:
//https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Sending is not working
//try:
    //checking if all hex conversions below are correct
        //checked, 3 were wrong, but wouldn't have caused the issue
    //checking if the tx interrupt goes high after send (make sure to not clear it by removing last line of send)
        //does not
    //mode of the module should automatically change to stand-by mode when sending is done
        //Does not go to stand-by mode, still in send mode
    //check to make sure setup/init is correct (does everything)
        //does everything in example code 
    //check the writes all work (by reading the registers that were written to) (some registers can only be written to in certain modes)
    //check that sending multiple commands at once works in hterm
    //check that writing and reading multiple bytes at a time work
    //could try writing using default settings (just add FIFO and then send?)
    //probably not the problem, but could look into the sent header flags
    //could look into the last send line that is supposed to enable the Rxdone flag
    //could try setting RH_RF95_REG_0E_FIFO_TX_BASE_ADDR to 0 in send as well as the FIFO pointer
    //could also try writing more bytes to the FIFO (more than supposed to based on length) to see if that initiates a send

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70 PAGE 70
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //this is only for non-LoRa mode, so not really relevant, it is in actually LoRa mode too
    //should trigger when fifoempty goes low (something in FIFO buffer)
    //start page of LoRa registers: can read other mode's registers in LoRa mode (try this to see if page 70 stuff is working)
    //try setting rxdone interrupt before setting mode to tx (could be done with tx before flag enable is set), example code has tx before interrupt flag enable, so shouldn't be the problem
    //could also try FSK mode instead of LoRa mode (harder to code (no example code, but can see what is happening better))

    //setting mode to LoRa sleep makes FIFO buffer have random info (this is supposed to clear it so maybe it is fine)
    //don't set RH_RF95_REG_26_MODEM_CONFIG3, but should be fine

//write formula: 57, reg | 0x80, 01, value (2 hex)

//read formula: 52, reg & ~0x80, 01


//to test if module is connected:
//52 0F 01 (on hterm, set baud = 57600), should get a response

//to test if message was received:
//52 13 01 //0B is default, value is number of payload bytes in last message
//can also read 12 to see if an interrupt has been triggered by a write
//52 12 01

//need to first set settings for the modules (both)
// 57 81 01 80
// 57 8E 01 00
// 57 8F 01 00
// 57 81 01 01
// 57 A0 01 00 
// 57 A1 01 08
// 57 86 01 D9
// 57 87 01 00
// 57 88 01 00
// 57 CD 01 04
// 57 89 01 88
// 57 9D 01 1A
// 57 9E 01 C4
//in one line:
// old 57 81 01 80 57 8E 01 00 57 8F 01 00 57 81 01 01 57 A0 01 00 57 A1 01 08 57 86 01 D9 57 87 01 00 57 88 01 00 57 CD 01 04 57 89 01 88 57 9D 01 1A 57 9E 01 C4
//new with implicit 57 81 01 80 57 8E 01 00 57 8F 01 00 57 81 01 01 57 A0 01 00 57 A1 01 08 57 86 01 D9 57 87 01 00 57 88 01 00 57 CD 01 04 57 89 01 88 57 9D 01 79 57 9E 01 C4 57 A2 01 06

//receive a message (one module)
// 57, 81, 01, 05 //set mode receive
// 57 C0 01 00 //interrupt on receive (should set a bit in register 12) DO NOT DO THIS BEFORE SETTING MODE TO RECEIVE
//52 10 01 //read start addr of last packet received
//57 8D 01 output of above //set FIFO pointer to addr of last packet received
//52 00 01 //read one byte of the message
//////////////////////////////////////////THIS WORKED

//send a message (other module)
// 57 81 01 01
// 57 8d 01 00
// 57 80 01 10 
// 57 80 01 10 
// 57 80 01 00 
// 57 80 01 00 
// 57 80 02 F0 0F 
// 57 A2 01 06
// 57 81 01 03 //goes to send mode
// 57 C0 01 40 //DO THIS LINE AFTER 57 8D 01 00, SENDING PROBABLY FINISHES BEFORE INTERRUPT CAN BE ENABLED, CAUSES AN INFINITE LOOP
// 57 92 01 ff //this clears interrupt register, likely not necessary for testing
//in one line:
// 57 81 01 01 57 8d 01 00 57 80 01 10 57 80 01 10 57 80 01 00 57 80 01 00 57 80 02 F0 0F 57 A2 01 06 57 81 01 03 57 C0 01 40 57 92 01 ff
// //data is F0 0F

//below send works (CRC error though, last part of data was corrupted)
//57 81 01 01 57 8d 01 00 57 C0 01 40 57 80 01 10 57 80 01 10 57 80 01 00 57 80 01 00 57 80 02 F0 0F 57 A2 01 06 57 81 01 03
////////////////////////THIS WORKS

//receive no longer works, never get interrupt that says a message has been received
//send now sends 49(letter I) back after the send is complete for some reason 
//both modules have an invalid 1D register, 1E also has strange value
//check to make sure init and send/receive set the registers to the correct values (read them after writing to them)

//try implicit header mode since all settings will be the same each time
//write to reg 1D (RegModemConfig1)


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
    USART5->BRR = 0x1A1; //baud rate of 115200 (table19)
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

        //setup FIFO
        lora_write_single(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0); //57 8E 01 00
        lora_write_single(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0); //57 8F 01 00

        //set mode to IDLE
        lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY); // 57 81 01 01

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

        lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, BANDWIDTH | CRC_ON | CODING_RATE | RH_RF95_IMPLICIT_HEADER_MODE_ON); // 57 9D 01 1E updated for implicit header
        lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, SPREADING_FACTOR | RH_RF95_AGC_AUTO_ON); //last 57 9E 01 C4

        //new config based on what should happen according to the manual
        lora_write_single(RH_RF95_REG_1D_MODEM_CONFIG1, 0x70 | 0x08 | 0x01); // 57 9D 01 79 updated for implicit header
        lora_write_single(RH_RF95_REG_1E_MODEM_CONFIG2, 0xC0  | 0x04); //last 57 9E 01 C4
        //end

        //AGC is automatic gain control, all examples use this, so I included it
        lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, length); //57 A2 01 06      update regpayload length (for implicit header mode only)
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

WHEN DOES LORA MODULE START SENDING MESSAGE?
at very end right?, once complete, valid message, then start sending?
I think you have to read addr0 several times in a row to read the data, it is automatically put in the FIFO buffer
//Since messages are not automatically sent to module, (have to write to read them), don't use DMA, just use read
//message receives are handled by lora_read_fifo_all
//message sends are handled by lora_send
//register reads are handled by lora_read_single and lora_read_multiple
//register writes are handled by lora_write_single and lora_write_multiple

unit8_t lora_read_fifo_single(){//done, not tested
    //THIS IS FOR READING ONE BYTE OF A LORA MESSAGE, 
    unit8_t val = 0;
    uart_write('R'); //0x52
    uart_write(0X00 & ~RH_WRITE_MASK); //0x00 & ~0x80, so 0x00
    uart_write(1); //0x01
    val = uart_read_single_only();
    return val; 
    // 52, 00, 01
}

void lora_read_fifo_all(unit8_t* data, unit8_t message_length){//done, not tested
    //THIS IS FOR READING THE ENTIRE LORA MESSAGE, 
     for (int i = 0; i < length; i ++) {
        *(data + i) = lora_read_fifo_single()
    }
}

bool lora_send(uint8_t* data, uint8_t length) { //not done, not tested
    //THIS IS FOR SENDING A LORA MESSAGE, NOT WRTING TO REGISTERS IN THE LORA MICRO
    //this handles sending a lora message
    //length is the length of the message in bytes
    //data is the payload data being sent

    //this function will need to be updated
    if (length > RH_RF95_MAX_MESSAGE_LEN) {
        return false;
    }

    //this->waitPacketSent(); // Make sure we dont interrupt an outgoing message
    //setModeIdle();
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY); //new 57, 81, 01, 01

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
    lora_write_single(RH_RF95_REG_22_PAYLOAD_LENGTH, length + RH_RF95_HEADER_LEN); //57 , A2, 01, 06

    //change module to send mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);  // 57, 81, 01, 03

    //lora_write_single(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags, new 57 92 01 ff
    return true;
}

void set_mode_sleep(){/////////////////////////////////////maybe or values | RH_RF95_LONG_RANGE_MODE to put in LoRa mode
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
}

void set_mode_standby(){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
}

void set_mode_continuous_receive(){
    lora_write_single(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS); // 57, 81, 01, 05
    lora_write_single(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // 57 C0 01 00
}

void lora_write_multiple(unit8_t reg, unit8_t* value, unit8_t length){//done, not tested
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


void lora_read_multiple(unit8_t reg, unit8_t* result, unit8_t length){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg and places it in result
    //reg is in the LoRa microcontroller
    //length is the number of bytes to read 
    unit8_t val = 0;
    uart_write('R');
    uart_write(reg & ~RH_WRITE_MASK);
    uart_write(length);

    int i = 0;
    while (1) {
        //available means uart is not currently reading a message, figure out how to do this
        *(result + i) = uart_read_single_only();
        i ++;
        if (i >= length) {
            break;
        }
    }
}

void lora_write_single(unit8_t reg, unit8_t value){//done, not tested
    //THIS IS FOR WRTING TO REGISTERS IN THE LORA MICRO, NOT SENDING A LORA MESSAGE
    //writes value to the address specified in reg 
    //reg is in the LoRa microcontroller 
    uart_write('W'); //0x57
    uart_write(reg | RH_WRITE_MASK); // try 00 | 80 = 80
    uart_write(1);
    uart_write(value);
    //57 80 01 FF //writes FF to addr 00 worked 
}

unit8_t lora_read_single(unit8_t reg){//done, not tested
    //THIS IS FOR READING REGISTERS IN THE LORA MICRO, NOT READING A LORA MESSAGE
    //reads value in the register reg
    //reg is in the LoRa microcontroller
    unit8_t val = 0;
    uart_write('R'); //0x52
    uart_write(reg & ~RH_WRITE_MASK); //try 0x0F & ~0x80, so 0x0F
    uart_write(1); //0x01
    val = uart_read_single_only();
    return val; //baud is 57600
    //worked with 52 0F 01
    // 52 00 01 read vale written by write
}

unit8_t uart_read(){ //not done (add timeout logic), not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR READING DATA SENT FROM THE LORA MICRO USING UART 
    //for reading received lora messages
    //USE lora_receive instead
    unit8_t c = 0;
    //UART_READ has to have timeout logic like in uartRx in RHUartDriver.cpp
    while (!(USART5->ISR & USART_ISR_RXNE)) { 
        c = USART5->RDR;
    }
    return c;
}

unit8_t uart_read_single_only(){ //not done (add timeout logic), not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR READING DATA SENT FROM THE LORA MICRO USING UART (reading register, not received message)
    //USE lora_read_single or lora_read_multiple

    //since DMA is setup for data the size of a lora message, don't use DMA
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off
    unit8_t c = 0;
    //UART_READ has to have timeout logic like in uartRx in RHUartDriver.cpp
    while (!(USART5->ISR & USART_ISR_RXNE)) { 
        c = USART5->RDR;
    }
    DMA2_Channel2->CCR |= DMA_CCR_EN; //enable DMA
    return c;
}



void uart_write(unit8_t data){ //done, not tested
    //DO NOT CALL THIS!!!!!! THIS IS FOR SENDING DATA TO THE LORA MICRO USING UART
    //USE lora_write_single, lora_write_multiple, or lora_send instead
    while(!(USART5->ISR & USART_ISR_TXE)) { 
        USART5->TDR = data;
    }
}



// char serfifo[FIFOSIZE]; //array of data read from LoRa module
// int seroffset = 0;

// void enable_tty_interrupt(void) { //DMA for receiving messages from LoRa module
//     RCC->AHBENR |= RCC_AHBENR_DMA2EN;
//     DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    
//     NVIC_EnableIRQ(USART3_6_IRQn); //enable interrupt for USART5
//     USART5->CR3 |= USART_CR3_DMAR; //enable DMA for reception
//     USART5->CR1 |= USART_CR1_RXNEIE;//raise interrupt when recieve data register is not empty

//     DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off
    
//     DMA2_Channel2->CMAR = (uint32_t)(&serfifo);//set CMAR
//     DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);//set CPAR
//     DMA2_Channel2->CNDTR = FIFOSIZE;//set CNDTR
//     DMA2_Channel2->CCR &= ~DMA_CCR_DIR;//set DIR to P->M
//     DMA2_Channel2->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE); //total-completion and half-transfer inturrupts are disabled
//     DMA2_Channel2->CCR &= ~(DMA_CCR_MSIZE_0 | DMA_CCR_MSIZE_1);//MSIZE to 8 bits
//     DMA2_Channel2->CCR &= ~(DMA_CCR_PSIZE_0 | DMA_CCR_PSIZE_1); //PSIZE to 8 bits
//     DMA2_Channel2->CCR |= DMA_CCR_MINC;//MINC increments on CMAR
//     DMA2_Channel2->CCR &= ~(DMA_CCR_PINC);//PINC is not set
//     DMA2_Channel2->CCR |= DMA_CCR_CIRC; //enable circular transfers
//     DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM; //do not enable MEM2MEM transfers
//     DMA2_Channel2->CCR |= DMA_CCR_PL_0 | DMA_CCR_PL_1;//set to the highest channel priority
    
//     DMA2_Channel2->CCR |= DMA_CCR_EN;
// }

// void USART3_8_IRQHandler(void) {  //UART interrupt handler
//     unit8_t index = 0;
//     while(DMA2_Channel2->CNDTR != index) {
//         serfifo[seroffset] = uart_read();
//         index += 1;
//     }
// }

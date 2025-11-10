#include "stm32f0xx.h"

#define PREAMBLE_LENGTH 8
#define CENTER_FREQUENCY 868
#define TXPOWER 13

//comment out one of the 2 below
#define USE_CUSTOM_SETTINGS
//#define USE_DEFAULT_SETTINGS

#ifdef USE_CUSTOM_SETTINGS
#define BANDWIDTH  RH_RF95_BW_125KHZ //values in RH_RF95.h
#define CODING_RATE RH_RF95_CODING_RATE_4_8 //chart on page 24, values in RH_RF95.h
#define SPREADING_FACTOR RH_RF95_SPREADING_FACTOR_4096CPS //chart on page 24, values in RH_RF95.h
#define CRC_ON RH_RF95_RX_PAYLOAD_CRC_ON //values in RH_RF95.h, 0 if no CRC else RH_RF95_RX_PAYLOAD_CRC_ON for CRC

#endif

#ifdef USE_DEFAULT_SETTINGS
#define MODEM_CONFIG_CHOICE Bw125Cr45Sf128 //(info page 22 of datasheet) Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on
    //is preset combo for Bandwidth, coding rate, spreading factor, CRC on/off (not using this)
#endif

//lora module datasheet (pg 84 is register addresses):
// https://files.seeedstudio.com/wiki/Grove_LoRa_Radio/res/RFM95_96_97_98_DataSheet.pdf
//example code:
// https://github.com/Seeed-Studio/Grove_LoRa_433MHz_and_915MHz_RF/blob/master/RH_RF95.cpp

void lora_uart_init(){
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
}

    bool lora_init(){
        //sets preamble length, center frequency, Tx power, and modem config
        ALSO NEED TO SET ADDRESS of the node

        //setPreambleLength Default is 8 bytes
        RH_RF95_REG_20_PREAMBLE_MSB = PREAMBLE_LENGTH >> 8; 
        RH_RF95_REG_21_PREAMBLE_LSB = PREAMBLE_LENGTH & 0xff;

        //setFrequency(868.0);
        uint32_t frf = (CENTER_FREQUENCY * 1000000.0) / RH_RF95_FSTEP;
        RH_RF95_REG_06_FRF_MSB = (frf >> 16) & 0xff;
        RH_RF95_REG_07_FRF_MID = (frf >> 8) & 0xff;
        RH_RF95_REG_08_FRF_LSB = frf & 0xff;

        //setTxPower(13);
        int8_t power = TXPOWER;
        if (power > 23) {
            power = 23;
        }
        if (power < 5) {
            power = 5;
        }
        if (power > 20) {
            RH_RF95_REG_4D_PA_DAC =  RH_RF95_PA_DAC_ENABLE;
            power -= 3;
        } else {
            RH_RF95_REG_4D_PA_DAC = RH_RF95_PA_DAC_DISABLE;
        }
        RH_RF95_REG_09_PA_CONFIG = RH_RF95_PA_SELECT | (power - 5);

        #ifdef USE_CUSTOM_SETTINGS

        RH_RF95_REG_1D_MODEM_CONFIG1 = BANDWIDTH | CRC_ON | CODING_RATE;
        RH_RF95_REG_1E_MODEM_CONFIG2 = SPREADING_FACTOR | RH_RF95_AGC_AUTO_ON; //AGC is automatic gain control, all examples use this, so I included it
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

//send a message
bool lora_send(uint8_t* data, uint8_t len) { //don't know what is going on here
    if (len > RH_RF95_MAX_MESSAGE_LEN) {
        return false;
    }

    this->waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    this->write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);

    // The headers
    this->write(RH_RF95_REG_00_FIFO, this->_txHeaderTo);
    this->write(RH_RF95_REG_00_FIFO, this->_txHeaderFrom);
    this->write(RH_RF95_REG_00_FIFO, this->_txHeaderId);
    this->write(RH_RF95_REG_00_FIFO, this->_txHeaderFlags);

    // The message data
    this->burstWrite(RH_RF95_REG_00_FIFO, data, len);
    this->write(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY

    //this->write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    return true;
}

//change mode to receive

//create data packet

//https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2527/113060006_Web.pdf
//https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE 

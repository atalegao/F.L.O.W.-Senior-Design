void lora_uart_init();
void uart_init_for_print();
bool lora_init();
uint8_t lora_read_fifo_single();
bool connected_test(void);
void enable_tty_interrupt(void);
void enable_tty_interrupt_send(void);
void lora_dma_write_send(int length);
void set_mode_continuous_receive();
void set_mode_sleep();
void lora_write_multiple(uint8_t reg, uint8_t* value, uint8_t length);
void lora_read_multiple(uint8_t reg, uint8_t* result, uint8_t length);
uint8_t lora_read_single(uint8_t reg);
uint8_t uart_read();
void uart_write(uint8_t data);
void lora_write_single(uint8_t reg, uint8_t value);
//new functions//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool check_irq_flags_receive(uint8_t* rxdone, uint8_t* valid_header, uint8_t *crc_error, bool clear);
void lora_read_fifo_all(uint8_t* data, uint8_t length);
void setup_leds(void);
void set_mode_cad(void);
bool cad_cyle(void);
void sleep_cycle(void);
bool continous_receive_for_cycle(uint8_t *rxdone,uint8_t *valid_header, uint8_t *crc_error, uint8_t *data);
void receive_cycle(void);
void data_processing(void);


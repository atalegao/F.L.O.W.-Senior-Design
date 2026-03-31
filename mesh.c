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
bool mesh_send_hello(uint8_t battery);
bool mesh_send_data(uint8_t dest, const uint8_t *data, uint8_t len);
bool mesh_send_ack(uint8_t dest, uint8_t acked_msg_id);
bool mesh_send_dead(uint8_t dest, uint8_t dead_addr, uint32_t dead_since, uint8_t battery); 
bool mesh_send_poll(uint8_t dest, uint8_t new_frequency);
bool mesh_send_add(uint8_t dest,uint8_t new_addr,uint32_t coords, uint16_t distance);
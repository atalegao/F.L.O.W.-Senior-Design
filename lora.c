//need a single receive mode or way to know if some data was received in continuous mode even if it was not a complete message
//RYLR998 has SMART RECEIVING POWER SAVING MODE, which may work, but also may not (datasheet is very bad)
    //probably won't be able to tell if it received part of a message

//need to find a new LoRa module?
    //one in lab does not have single receive or any way to know 
    //GAMMA62TR-89D no

//DS_SX1276-7-8-9_W_APP_V7.
    //has single receive mode, listens for a valid premable, so could just make first message's data a bunch of preambles 
    //then send the data next time 
    //is expensive and is just the chip (no PCB with antenna)

//https://www.digikey.com/en/products/detail/seeed-technology-co-ltd/113990830/16652881
    //uses DS_SX1276-7-8-9
    //https://files.seeedstudio.com/products/113990830/document/Datasheet_LoRaST(1).pdf
    //should be easy to program
    //$31, not many in stock

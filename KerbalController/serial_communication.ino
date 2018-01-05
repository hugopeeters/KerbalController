uint8_t rx_len;
uint16_t * address;
byte buffer[256]; //address for temporary storage and parsing buffer
uint8_t structSize;
uint8_t rx_array_inx;  //index for RX parsing buffer
uint8_t calc_CS;     //calculated Chacksum

struct HandShakePacket
{
  byte id;
  byte M1;
  byte M2;
  byte M3;
};

HandShakePacket HPacket;

void Handshake(){
  HPacket.id = 0;
  HPacket.M1 = 3;
  HPacket.M2 = 1;
  HPacket.M3 = 4;
  KSPBoardSendData(details(HPacket));
}

void InitTxPackets() {
  HPacket.id = 0;  
  CPacket.id = 101;
}

//This shit contains stuff borrowed from EasyTransfer lib
boolean KSPBoardReceiveData() {
  if ((rx_len == 0)&&(Serial.available()>3)){
    while(Serial.read()!= 0xBE) {
      if (Serial.available() == 0)
        return false;  
    }
    if (Serial.read() == 0xEF){
      rx_len = Serial.read();   
      id = Serial.read(); 
      rx_array_inx = 1;

      switch(id) {
      case 0:
        structSize = sizeof(HPacket);   
        address = (uint16_t*)&HPacket;     
        break;
      case 1:
        structSize = sizeof(VData);   
        address = (uint16_t*)&VData;     
        break;
      }
    }

    //make sure the binary structs on both Arduinos are the same size.
    if(rx_len != structSize){
      rx_len = 0;
      return false;
    }   
  }

  if(rx_len != 0){
    while(Serial.available() && rx_array_inx <= rx_len){
      buffer[rx_array_inx++] = Serial.read();
    }
    buffer[0] = id;

    if(rx_len == (rx_array_inx-1)){
      //seem to have got whole message
      //last uint8_t is CS
      calc_CS = rx_len;
      for (int i = 0; i<rx_len; i++){
        calc_CS^=buffer[i];
      } 

      if(calc_CS == buffer[rx_array_inx-1]){//CS good
        memcpy(address,buffer,structSize);
        rx_len = 0;
        rx_array_inx = 1;
        return true;
      }
      else{
        //failed checksum, need to clear this out anyway
        rx_len = 0;
        rx_array_inx = 1;
        return false;
      }
    }
  }

  return false;
}

void KSPBoardSendData(uint8_t * address, uint8_t len){
  uint8_t CS = len;
  Serial.write(0xBE);
  Serial.write(0xEF);
  Serial.write(len);
  
  for(int i = 0; i<len; i++){
    CS^=*(address+i);
    Serial.write(*(address+i));
  }
  
  Serial.write(CS);
}

void controlsInit() {
  pinMode(pTHROTTLE, INPUT);
  pinMode(pTX, INPUT);
  pinMode(pTY, INPUT);
  pinMode(pTZ, INPUT);
  pinMode(pRX, INPUT);
  pinMode(pRY, INPUT);
  pinMode(pRZ, INPUT);
  pinMode(pPOWER, INPUT_PULLUP);
  pinMode(pTB, INPUT_PULLUP);
  pinMode(pRB, INPUT_PULLUP);
  pinMode(pMODE, INPUT_PULLUP);
  pinMode(pLCDx, INPUT_PULLUP);
  pinMode(pLCDy, INPUT_PULLUP);
  pinMode(pLCDz, INPUT_PULLUP);
  pinMode(pSAS, INPUT_PULLUP);
  pinMode(pRCS, INPUT_PULLUP);
  pinMode(pABORT, INPUT);
  pinMode(pARM, INPUT);
  pinMode(pSTAGE, INPUT_PULLUP);
  pinMode(pSTAGELED, OUTPUT);
  pinMode(pLIGHTS, INPUT_PULLUP);
  pinMode(pLIGHTSLED, OUTPUT);
  pinMode(pLADDER, INPUT_PULLUP);
  pinMode(pLADDERLED, OUTPUT);
  pinMode(pSOLAR, INPUT_PULLUP);
  pinMode(pSOLARLED, OUTPUT);
  pinMode(pCHUTES, INPUT_PULLUP);
  pinMode(pCHUTESLED, OUTPUT);
  pinMode(pGEARS, INPUT_PULLUP);
  pinMode(pGEARSLED, OUTPUT);
  pinMode(pBRAKES, INPUT_PULLUP);
  pinMode(pBRAKESLED, OUTPUT);
  pinMode(pACTION1, INPUT_PULLUP);
  pinMode(pACTION1LED, OUTPUT);
  pinMode(pACTION2, INPUT_PULLUP);
  pinMode(pACTION2LED, OUTPUT);
  pinMode(pACTION3, INPUT_PULLUP);
  pinMode(pACTION3LED, OUTPUT);
  pinMode(pACTION4, INPUT_PULLUP);
  pinMode(pACTION4LED, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void testLEDS(int testdelay){
  digitalWrite(pSTAGELED,HIGH);
  delay(testdelay);
  digitalWrite(pSTAGELED,LOW);
  digitalWrite(pLIGHTSLED,HIGH);
  delay(testdelay);
  digitalWrite(pLIGHTSLED,LOW);
  digitalWrite(pLADDERLED,HIGH);
  delay(testdelay);
  digitalWrite(pLADDERLED,LOW);
  digitalWrite(pSOLARLED,HIGH);
  delay(testdelay);
  digitalWrite(pSOLARLED,LOW);
  digitalWrite(pCHUTESLED,HIGH);
  delay(testdelay);
  digitalWrite(pCHUTESLED,LOW);
  digitalWrite(pGEARSLED,HIGH);
  delay(testdelay);
  digitalWrite(pGEARSLED,LOW);
  digitalWrite(pBRAKESLED,HIGH);
  delay(testdelay);
  digitalWrite(pBRAKESLED,LOW);
  digitalWrite(pACTION1LED,HIGH);
  delay(testdelay);
  digitalWrite(pACTION1LED,LOW);
  digitalWrite(pACTION2LED,HIGH);
  delay(testdelay);
  digitalWrite(pACTION2LED,LOW);
  digitalWrite(pACTION3LED,HIGH);
  delay(testdelay);
  digitalWrite(pACTION3LED,LOW);
  digitalWrite(pACTION4LED,HIGH);
  delay(testdelay);
  digitalWrite(pACTION4LED,LOW);
  
  //prepare the shift register
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);

  inputBytes[0] = B11111111;
  inputBytes[1] = B11111111;
  inputBytes[2] = B11111111;
  inputBytes[3] = B11111111;
  inputBytes[4] = B11111111;
  inputBytes[5] = B11111111;
  inputBytes[6] = B11111111;
  //loop through the input bytes
  for (int j=0; j<=6; j++){
    byte inputByte = inputBytes[j];
    Serial.println(inputByte);
    shiftOut(dataPin, clockPin, MSBFIRST, inputByte);
  }
  
  //latch the values in when done shifting
  digitalWrite(latchPin, HIGH); 
  
  delay(testdelay);
  
  //prepare the shift register
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);

  inputBytes[0] = B00000000;
  inputBytes[1] = B00000000;
  inputBytes[2] = B00000000;
  inputBytes[3] = B00000000;
  inputBytes[4] = B00000000;
  inputBytes[5] = B00000000;
  inputBytes[6] = B00000000;
  //loop through the input bytes
  for (int j=0; j<=6; j++){
    byte inputByte = inputBytes[j];
    Serial.println(inputByte);
    shiftOut(dataPin, clockPin, MSBFIRST, inputByte);
  }
  
  //latch the values in when done shifting
  digitalWrite(latchPin, HIGH); 
}

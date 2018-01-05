#include <SoftwareSerial.h>
SoftwareSerial mySerial(15,14); //pin 14 connected to LCD, 15 unconnected

//analog pins
const int pTHROTTLE = A0; //slide pot
const int pTX = A1;       //translation x-axis
const int pTY = A2;       //translation y-axis
const int pTZ = A3;       //translation z-axis
const int pRX = A4;       //rotation x-axis
const int pRY = A5;       //rotation y-axis
const int pRZ = A6;       //rotation z-axis

//digital pins
const int pPOWER = 2;       //power switch
const int pTB = 3;          //translation joystick button
const int pRB = 4;          //rotation joystick button
const int latchPin = 8;     //ST_CP - green
const int dataPin = 11;     //DS - yellow
const int clockPin = 12;    //SH_CP - blue
const int pMODE = 22;       //mode switch (used for debug mode)
const int pLCDx = 27;       //toggle switch x (used for LCD display modes)
const int pLCDy = 24;       //toggle switch y (used for LCD display modes)
const int pLCDz = 29;       //toggle switch z (used for LCD display modes)
const int pSAS = 26;        //SAS switch
const int pRCS = 31;        //RCS switch
const int pABORT = 28;      //Abort switch (safety switch, active high)
const int pARM = 30;        //Arm switch (safety switch, active high)
const int pSTAGE = 32;      //Stage button
const int pSTAGELED = 33;   //Stage button LED
const int pLIGHTS = 34;     //Lights button
const int pLIGHTSLED = 35;  //Lights button LED
const int pLADDER = 36;     //Ladder button (action group 5)
const int pLADDERLED = 37;  //Ladder button LED
const int pSOLAR = 38;      //Solar button (action group 6)
const int pSOLARLED = 39;   //Solar button LED
const int pCHUTES = 40;     //Chutes button (action group 7)
const int pCHUTESLED = 41;  //Chutes button LED
const int pGEARS = 42;      //Gears button
const int pGEARSLED = 43;   //Gears button LED
const int pBRAKES = 44;     //Brakes button
const int pBRAKESLED = 45;  //Brakes button LED
const int pACTION1 = 46;    //Action Group 1 button
const int pACTION1LED = 47; //Action Group 1 button LED
const int pACTION2 = 48;    //Action Group 2 button
const int pACTION2LED = 49; //Action Group 2 button LED
const int pACTION3 = 50;    //Action Group 3 button
const int pACTION3LED = 51; //Action Group 3 button LED
const int pACTION4 = 52;    //Action Group 4 button
const int pACTION4LED = 53; //Action Group 4 button LED

//in-game state used to update button LEDs
bool lights_on = false;
bool ladder_on = false;
bool solar_on = false;
bool gears_on = false;
bool brakes_on = false;
bool chutes_on = false;
bool stage_on = false;
bool action1_on = false;
bool action2_on = false;
bool action3_on = false;
bool action4_on = false;

//toggle button states
bool tb_on = true;
bool rb_on = true;

//previous button state
bool tb_prev = false;
bool rb_prev = false;

//stage LED state
bool stage_led_on = false;

//input value variables
int throttle_value;
int tx_value;
int ty_value;
int tz_value;
int rx_value;
int ry_value;
int rz_value;

//variables used for fuel guages
byte inputBytes[7];
int vSF, vLF, vOX, vEL, vMP, SF, LF, OX, EL, MP;

//debug variable
bool debug = false;

//timing
const int IDLETIMER = 20000;        //if no message received from KSP for more than 20s, go idle (default 2000)
const int CONTROLREFRESH = 10;      //send control packet every 10 ms (default 25)
const int stageLedBlinkTime = 500;  //blink staging LED when armed every 500 ms

//variables used in timing
unsigned long deadtime, deadtimeOld, controlTime, controlTimeOld, stageLedTime, stageLedTimeOld;
unsigned long now;

//variables used in serial communication
boolean Connected = false;
byte id;

void setup() {
  
  Serial.begin(38400);  //KSPSerialIO connection
  mySerial.begin(9600); //LCD connection
  delay(500);           //wait for LCD boot
  
  //write to LCD
  clearLCD();
  writeLCD("KerbalController");
  jumpToLineTwo();
  writeLCD("booting...");
  delay(100);

  //Initialize
  controlsInit();   //set all pin modes
  testLEDS(50);     //blink every LED once to test (with a delay of 10 ms)
  InitTxPackets();  //initialize the serial communication
}

void loop() {
  
  //check mode
  if(!digitalRead(pMODE)){debug = true;} else {debug = false;}

  if(debug){
    //Debug mode does not communicate with KSPSerialIO, but simply displays the button states on the LCD.
    //Select analog input using LCDx/y/z. Xyz = Throttle. xYz = Translation. xyZ = Rotation.

    //previous state tracking only used in debug
    bool stage_prev = false;
    bool chutes_prev = false;
    bool action1_prev = false;
    bool action2_prev = false;
    bool action3_prev = false;
    bool action4_prev = false;
    bool lights_prev = false;
    bool ladder_prev = false;
    bool solar_prev = false;
    bool gears_prev = false;
    bool brakes_prev = false;

    //clear the LCD 
    clearLCD();
    
    //toggle switches
    if(!digitalRead(pSAS)){writeLCD("S");} else {writeLCD("s");}
    if(!digitalRead(pRCS)){writeLCD("R");} else {writeLCD("r");}
    if(digitalRead(pABORT)){writeLCD("A");} else {writeLCD("a");} //note abort switch is active high
    if(digitalRead(pARM)){writeLCD("A");} else {writeLCD("a");}   //note arm switch is active high
   
    //in debug mode, handle all buttons as toggle buttons

    if(!digitalRead(pSTAGE) && !stage_prev){stage_on = !stage_on; stage_prev = true;}
    if(digitalRead(pSTAGE) && stage_prev){stage_prev = false;}
    if(stage_on){writeLCD("S");} else {writeLCD("s");}
    digitalWrite(pSTAGELED, stage_on);

    if(!digitalRead(pLIGHTS) && !lights_prev){lights_on = !lights_on; lights_prev = true;}
    if(digitalRead(pLIGHTS) && lights_prev){lights_prev = false;}
    if(lights_on){writeLCD("L");} else {writeLCD("l");}
    digitalWrite(pLIGHTSLED, lights_on);
    
    if(!digitalRead(pLADDER) && !ladder_prev){ladder_on = !ladder_on; ladder_prev = true;}
    if(digitalRead(pLADDER) && ladder_prev){ladder_prev = false;}
    if(ladder_on){writeLCD("L");} else {writeLCD("l");}
    digitalWrite(pLADDERLED, ladder_on);
    
    if(!digitalRead(pSOLAR) && !solar_prev){solar_on = !solar_on; solar_prev = true;}
    if(digitalRead(pSOLAR) && solar_prev){solar_prev = false;}
    if(solar_on){writeLCD("S");} else {writeLCD("s");}
    digitalWrite(pSOLARLED, solar_on);
  
    if(!digitalRead(pCHUTES) && !chutes_prev){chutes_on = !chutes_on; chutes_prev = true;}
    if(digitalRead(pCHUTES) && chutes_prev){chutes_prev = false;}
    if(chutes_on){writeLCD("C");} else {writeLCD("c");}
    digitalWrite(pCHUTESLED, chutes_on);
    
    if(!digitalRead(pGEARS) && !gears_prev){gears_on = !gears_on; gears_prev = true;}
    if(digitalRead(pGEARS) && gears_prev){gears_prev = false;}
    if(gears_on){writeLCD("G");} else {writeLCD("g");}
    digitalWrite(pGEARSLED, gears_on);
  
    if(!digitalRead(pBRAKES) && !brakes_prev){brakes_on = !brakes_on; brakes_prev = true;}
    if(digitalRead(pBRAKES) && brakes_prev){brakes_prev = false;}
    if(brakes_on){writeLCD("B");} else {writeLCD("b");}
    digitalWrite(pBRAKESLED, brakes_on);

    if(!digitalRead(pACTION1) && !action1_prev){action1_on = !action1_on; action1_prev = true;}
    if(digitalRead(pACTION1) && action1_prev){action1_prev = false;}
    if(action1_on){writeLCD("A");} else {writeLCD("a");}
    digitalWrite(pACTION1LED, action1_on);
    
    if(!digitalRead(pACTION2) && !action2_prev){action2_on = !action2_on; action2_prev = true;}
    if(digitalRead(pACTION2) && action2_prev){action2_prev = false;}
    if(action2_on){writeLCD("A");} else {writeLCD("a");}
    digitalWrite(pACTION2LED, action2_on);
    
    if(!digitalRead(pACTION3) && !action3_prev){action3_on = !action3_on; action3_prev = true;}
    if(digitalRead(pACTION3) && action3_prev){action3_prev = false;}
    if(action3_on){writeLCD("A");} else {writeLCD("a");}
    digitalWrite(pACTION3LED, action3_on);
    
    if(!digitalRead(pACTION4) && !action4_prev){action4_on = !action4_on; action4_prev = true;}
    if(digitalRead(pACTION4) && action4_prev){action4_prev = false;}
    if(action4_on){writeLCD("A");} else {writeLCD("a");}
    digitalWrite(pACTION4LED, action4_on);
    
    if(!digitalRead(pTB) && !tb_prev){tb_on = !tb_on; tb_prev = true;}
    if(digitalRead(pTB) && tb_prev){tb_prev = false;}
    if(tb_on){writeLCD("T");} else {writeLCD("t");}
  
    if(!digitalRead(pRB) && !rb_prev){rb_on = !rb_on; rb_prev = true;}
    if(digitalRead(pRB) && rb_prev){rb_prev = false;}
    if(rb_on){writeLCD("R");} else {writeLCD("r");}
  
    //analog inputs
    if(!digitalRead(pLCDx) && digitalRead(pLCDy) && digitalRead(pLCDz)){
      throttle_value = analogRead(pTHROTTLE);
      char throttle_char[5];
      itoa(throttle_value, throttle_char, 10);
      writeLCD(throttle_char);
      writeLCD(" ");
    }
    if(digitalRead(pLCDx) && !digitalRead(pLCDy) && digitalRead(pLCDz)){
      tx_value = analogRead(pTX);
      char tx_char[5];
      itoa(tx_value, tx_char, 10);
      writeLCD(tx_char);
      writeLCD(" ");
      ty_value = analogRead(pTY);
      char ty_char[5];
      itoa(ty_value, ty_char, 10);
      writeLCD(ty_char);
      writeLCD(" ");
      tz_value = analogRead(pTZ);
      char tz_char[5];
      itoa(tz_value, tz_char, 10);
      writeLCD(tz_char);
      writeLCD(" ");
    }
     
    if(digitalRead(pLCDx) && digitalRead(pLCDy) && !digitalRead(pLCDz)){ 
      rx_value = analogRead(pRX);
      char rx_char[5];
      itoa(rx_value, rx_char, 10);
      writeLCD(rx_char);
      writeLCD(" ");    
      ry_value = analogRead(pRY);
      char ry_char[5];
      itoa(ry_value, ry_char, 10);
      writeLCD(ry_char);
      writeLCD(" ");
      rz_value = analogRead(pRZ);
      char rz_char[5];
      itoa(rz_value, rz_char, 10);
      writeLCD(rz_char);
    }
    //end of debug mode
  }
  else {
    
    //KSP mode

    //send and receive data
    get_vessel_data();
    send_control_packet();
  }
}

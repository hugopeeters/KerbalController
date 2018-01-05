//Define what controls to send to KSP

//check if it is time to send a control packet
void send_control_packet() {
  now = millis();
  controlTime = now - controlTimeOld;
  if (controlTime > CONTROLREFRESH) {
    controlTimeOld = now;
    define_control_packet();
  }
}

//define the structure of a control packet
struct ControlPacket {
  //the following controls can be sent to the KSPSerialIO plugin (defined by the plugin)
  byte id;
  byte MainControls;                  //SAS RCS Lights Gears Brakes Precision Abort Stage (see enum)
  byte Mode;                          //0 = stage, 1 = docking, 2 = map
  unsigned int ControlGroup;          //action groups 1-10
  byte NavballSASMode;                //AutoPilot mode
  byte AdditionalControlByte1;
  int Pitch;                          //-1000 -> 1000
  int Roll;                           //-1000 -> 1000
  int Yaw;                            //-1000 -> 1000
  int TX;                             //-1000 -> 1000
  int TY;                             //-1000 -> 1000
  int TZ;                             //-1000 -> 1000
  int WheelSteer;                     //-1000 -> 1000
  int Throttle;                       //    0 -> 1000
  int WheelThrottle;                  //    0 -> 1000
};

//Create an instance of a control packet
ControlPacket CPacket;

//macro used to generate the control packet (also used for the handshake packet)
#define details(name) (uint8_t*)&name,sizeof(name)

//Enumeration of MainControls
#define SAS       7
#define RCS       6
#define LIGHTS    5
#define GEARS     4
#define BRAKES    3
#define PRECISION 2
#define ABORT     1
#define STAGE     0

//Main controls uses enum above, e.g. MainControls(SAS,true);
void MainControls(byte n, boolean s) {
  if (s)
    CPacket.MainControls |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  else
    CPacket.MainControls &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
}

//Control groups (action groups) uses an integer to refer to a custom action group, e.g. ControlGroup(5,true);
void ControlGroups(byte n, boolean s) {
  if (s)
    CPacket.ControlGroup |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  else
    CPacket.ControlGroup &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
}

//Enumeration of SAS Modes
#define SMOFF           0
#define SMSAS           1
#define SMPrograde      2
#define SMRetroGrade    3
#define SMNormal        4
#define SMAntinormal    5
#define SMRadialIn      6
#define SMRadialOut     7
#define SMTarget        8
#define SMAntiTarget    9
#define SMManeuverNode  10

//SAS mode uses enum above, e.g. setSASMode(SMPrograde);
void setSASMode(byte m) {
  CPacket.NavballSASMode &= B11110000;
  CPacket.NavballSASMode += m;
}

//Enumeration of Navball Target Modes
#define NAVBallIGNORE   0
#define NAVBallORBIT    1
#define NAVBallSURFACE  2
#define NAVBallTARGET   3

//Navball mode uses enum above, e.g. setNavBallMode(NAVBallSURFACE);
void setNavballMode(byte m) {
  CPacket.NavballSASMode &= B00001111;
  CPacket.NavballSASMode += m << 4;
}


void define_control_packet() {
  if (Connected) {
    //here we define what controls to send when which pins are manipulated
    
    //toggle switches
    if(!digitalRead(pSAS)){MainControls(SAS, true);} else {MainControls(SAS, false);}
    if(!digitalRead(pRCS)){MainControls(RCS, true);} else {MainControls(RCS, false);}
    if(digitalRead(pABORT)){MainControls(ABORT, true);} else {MainControls(ABORT, false);}
  
    //momentary stage button
    if(!digitalRead(pSTAGE) && digitalRead(pARM)){MainControls(STAGE, true);} else {MainControls(STAGE, false);}
    if(digitalRead(pARM)){
      now = millis();
      stageLedTime = now - stageLedTimeOld;
      if (stageLedTime > stageLedBlinkTime) {
        stageLedTimeOld = now;
        stage_led_on = !stage_led_on;
      }
    }
    else {stage_led_on = false;}
    digitalWrite(pSTAGELED, stage_led_on);
    
    //toggle buttons
    if(!digitalRead(pLIGHTS)){MainControls(LIGHTS, !lights_on);}
    if(!digitalRead(pGEARS)){MainControls(GEARS, !gears_on);}
    if(!digitalRead(pBRAKES)){MainControls(BRAKES, !brakes_on);}
    if(!digitalRead(pACTION1)){ControlGroups(1, !action1_on);}
    if(!digitalRead(pACTION2)){ControlGroups(2, !action2_on);}
    if(!digitalRead(pACTION3)){ControlGroups(3, !action3_on);}
    if(!digitalRead(pACTION4)){ControlGroups(4, !action4_on);}
    if(!digitalRead(pLADDER)){ControlGroups(5, !ladder_on);}
    if(!digitalRead(pSOLAR)){ControlGroups(6, !solar_on);}
    if(!digitalRead(pCHUTES)){ControlGroups(7, !chutes_on);}

    //throttle
    CPacket.Throttle = constrain(map(analogRead(pTHROTTLE),30,990,0,1023),0,1000);

    //rotation joystick button toggles flight control modes
    if(!digitalRead(pRB) && !rb_prev){rb_on = !rb_on; rb_prev = true;}
    if(digitalRead(pRB) && rb_prev){rb_prev = false;}
    
    if(rb_on){
      //rocket mode
      if(analogRead(pRX) >= 530){CPacket.Yaw = constrain(map(analogRead(pRX),1023,530,-1000,0),-1000,0);}
      else if(analogRead(pRX) <= 470){CPacket.Yaw = constrain(map(analogRead(pRX),470,0,0,1000),0,1000);}
      else {CPacket.Yaw = 0;}
      if(analogRead(pRY) >= 530){CPacket.Pitch = constrain(map(analogRead(pRY),530,1023,0,1000),0,1000);}
      else if(analogRead(pRY) <= 470){CPacket.Pitch = constrain(map(analogRead(pRY),0,470,-1000,0),-1000,0);}
      else {CPacket.Pitch = 0;}
      if(analogRead(pRZ) <= 40){CPacket.Roll = constrain(map(analogRead(pRZ),0,40,-1000,0),-1000,0);}
      else if(analogRead(pRZ) >= 60){CPacket.Roll = constrain(map(analogRead(pRZ),60,500,0,1000),0,1000);}
      else {CPacket.Roll = 0;}

      if(analogRead(pTX) >= 530){CPacket.TX = constrain(map(analogRead(pTX),1023,530,0,1000),0,1000);}
      else if(analogRead(pTX) <= 470){CPacket.TX = constrain(map(analogRead(pTX),470,0,-1000,0),-1000,0);}
      else {CPacket.TX = 0;}
      if(analogRead(pTY) >= 530){CPacket.TY = constrain(map(analogRead(pTY),1023,530,-1000,0),-1000,0);}
      else if(analogRead(pTY) <= 470){CPacket.TY = constrain(map(analogRead(pTY),470,0,0,1000),0,1000);}
      else {CPacket.TY = 0;}
      if(analogRead(pTZ) <= 60){CPacket.TZ = constrain(map(analogRead(pTZ),0,60,-1000,0),-1000,0);}
      else if(analogRead(pTZ) >= 90){CPacket.TZ = constrain(map(analogRead(pTZ),90,500,0,1000),0,1000);}
      else {CPacket.TZ = 0;}
    }
    else {
      //airplane mode
      if(analogRead(pRX) >= 530){CPacket.Roll = constrain(map(analogRead(pRX),1023,530,-1000,0),-1000,0);}
      else if(analogRead(pRX) <= 470){CPacket.Roll = constrain(map(analogRead(pRX),470,0,0,1000),0,1000);}
      else {CPacket.Yaw = 0;}
      if(analogRead(pRY) >= 530){CPacket.Pitch = constrain(map(analogRead(pRY),530,1023,0,1000),0,1000);}
      else if(analogRead(pRY) <= 470){CPacket.Pitch = constrain(map(analogRead(pRY),0,470,-1000,0),-1000,0);}
      else {CPacket.Pitch = 0;}
      if(analogRead(pTX) >= 530){CPacket.Yaw = constrain(map(analogRead(pTX),1023,530,-1000,0),-1000,0);}
      else if(analogRead(pTX) <= 470){CPacket.Yaw = constrain(map(analogRead(pTX),470,0,0,1000),0,1000);}
      else {CPacket.Yaw = 0;}
      CPacket.TX = 0;
      CPacket.TY = 0;
      CPacket.TZ = 0;
    }

    //    //translation joystick button toggles between modes?
    //    if(!digitalRead(pTB) && !tb_prev){tb_on = !tb_on; tb_prev = true;}
    //    if(digitalRead(pTB) && tb_prev){tb_prev = false;}
    //    if(tb_on){
    //      
    //    }
    //    else {
    //      
    //    }

    //send the control packet to the KSPSerialIO plugin
    KSPBoardSendData(details(CPacket)); 
  }
}


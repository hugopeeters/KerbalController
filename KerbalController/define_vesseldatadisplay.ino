//grab info from KSP here (VData object) and write out results to the Arduino pins

//connect to the KSPSerialIO plugin and receive data
int get_vessel_data() {
  int returnValue = -1;
  now = millis();

  if (KSPBoardReceiveData()){
    //data is being received
    deadtimeOld = now;
    returnValue = id;
    switch(id) {
    case 0: //First packet is a handshake packet
      Handshake();
      clearLCD();
      writeLCD("KerbalController");
      writeLCD("handshake...");
      break;
    case 1:
      //subsequent packets are data from the plugin
      define_vessel_data_display(); //define what to do with the data below
      break;
    }
    Connected = true;
  }
  else
  { //if no message received for a while, go idle
    deadtime = now - deadtimeOld; 
    if (deadtime > IDLETIMER)
    {
      deadtimeOld = now;
      Connected = false;
      clearLCD();
      writeLCD("KerbalController");
      writeLCD("idle...");
    }    
  }
  return returnValue;
}

//define the structure of a VesseData packet
struct VesselData
{
    //the following data is available in the packet received from the plugin (defined by the KSPSerialIO plugin)
    byte id;                //1   packet id
    float AP;               //2   apoapsis (m to sea level)
    float PE;               //3   periapsis (m to sea level)
    float SemiMajorAxis;    //4   orbit semi major axis (m)
    float SemiMinorAxis;    //5   orbit semi minor axis (m)
    float VVI;              //6   vertical velocity (m/s)
    float e;                //7   orbital eccentricity (unitless, 0 = circular, > 1 = escape)
    float inc;              //8   orbital inclination (degrees)
    float G;                //9   acceleration (Gees)
    long TAp;               //10  time to AP (seconds)
    long TPe;               //11  time to Pe (seconds)
    float TrueAnomaly;      //12  orbital true anomaly (degrees)
    float Density;          //13  air density (presumably kg/m^3, 1.225 at sea level)
    long period;            //14  orbital period (seconds)
    float RAlt;             //15  radar altitude (m)
    float Alt;              //16  altitude above sea level (m)
    float Vsurf;            //17  surface velocity (m/s)
    float Lat;              //18  Latitude (degree)
    float Lon;              //19  Longitude (degree)
    float LiquidFuelTot;    //20  Liquid Fuel Total
    float LiquidFuel;       //21  Liquid Fuel remaining
    float OxidizerTot;      //22  Oxidizer Total
    float Oxidizer;         //23  Oxidizer remaining
    float EChargeTot;       //24  Electric Charge Total
    float ECharge;          //25  Electric Charge remaining
    float MonoPropTot;      //26  Mono Propellant Total
    float MonoProp;         //27  Mono Propellant remaining
    float IntakeAirTot;     //28  Intake Air Total
    float IntakeAir;        //29  Intake Air remaining
    float SolidFuelTot;     //30  Solid Fuel Total
    float SolidFuel;        //31  Solid Fuel remaining
    float XenonGasTot;      //32  Xenon Gas Total
    float XenonGas;         //33  Xenon Gas remaining
    float LiquidFuelTotS;   //34  Liquid Fuel Total (stage)
    float LiquidFuelS;      //35  Liquid Fuel remaining (stage)
    float OxidizerTotS;     //36  Oxidizer Total (stage)
    float OxidizerS;        //37  Oxidizer remaining (stage)
    uint32_t MissionTime;   //38  Time since launch (s)
    float deltaTime;        //39  Time since last packet (s)
    float VOrbit;           //40  Orbital speed (m/s)
    uint32_t MNTime;        //41  Time to next node (s) [0 when no node]
    float MNDeltaV;         //42  Delta V for next node (m/s) [0 when no node]
    float Pitch;            //43  Vessel Pitch relative to surface (degrees)
    float Roll;             //44  Vessel Roll relative to surface (degrees)
    float Heading;          //45  Vessel Heading relative to surface (degrees)
    uint16_t ActionGroups;  //46  status bit order:SAS, RCS, Light, Gears, Brakes, Abort, Custom01 - 10
    byte SOINumber;         //47  SOI Number (decimal format: sun-planet-moon e.g. 130 = kerbin, 131 = mun)
    byte MaxOverHeat;       //48  Max part overheat (% percent)
    float MachNumber;       //49  Mach number
    float IAS;              //50  Indicated Air Speed
    byte CurrentStage;      //51  Current stage number
    byte TotalStage;        //52  TotalNumber of stages
    float TargetDist;       //53  Distance to targeted vessel (m)
    float TargetV;          //54  Target vessel relative velocity
    byte NavballSASMode;    //55  Combined byte for navball target mode and SAS mode
                                // First four bits indicate AutoPilot mode:
                                // 0 SAS is off  //1 = Regular Stability Assist //2 = Prograde
                                // 3 = RetroGrade //4 = Normal //5 = Antinormal //6 = Radial In
                                // 7 = Radial Out //8 = Target //9 = Anti-Target //10 = Maneuver node
                                // Last 4 bits set navball mode. (0=ignore,1=ORBIT,2=SURFACE,3=TARGET)
};

//create an instance of a VesselData object
VesselData VData;

//Enumeration of ActionGroups (includes main controls and custom action groups)
#define AGSAS      0
#define AGRCS      1
#define AGLight    2
#define AGGears    3
#define AGBrakes   4
#define AGAbort    5
#define AGCustom01 6
#define AGCustom02 7
#define AGCustom03 8
#define AGCustom04 9
#define AGCustom05 10
#define AGCustom06 11
#define AGCustom07 12
#define AGCustom08 13
#define AGCustom09 14
#define AGCustom10 15

//get the current state of main controls and custom action groups using enumeration above, e.g. ControlStatus(AGBrakes);
byte ControlStatus(byte n)
{
  return ((VData.ActionGroups >> n) & 1) == 1;
}

//get the current SAS Mode. Can be compared to enum values, e.g. if(getSASMode() == SMPrograde){//do stuff}
byte getSASMode() {
  return VData.NavballSASMode & B00001111; // leaves alone the lower 4 bits of; all higher bits set to 0.
}

//get the current navball mode. Can be compared to enum values, e.g. if (getNavballMode() == NAVBallTARGET){//do stuff}
byte getNavballMode() {
  return VData.NavballSASMode >> 4; // leaves alone the higher 4 bits of; all lower bits set to 0.
}

//define what to do with the vessel data here, e.g. turn on LED's, display text on the LCD
void define_vessel_data_display() {
  
  //Fuel LED bar charts - NEED TO USE A SHIFT REGISTER to drive the LED bar charts!
  // to be implemented

  //LCD Display Modes
  // 0 xyz TakeOff Mode:     Suface Velocity / Acceleration (G)
  // 1 Xyz Orbit Mode:       Apoapsis + Time to Apoapsis / Periapsis + Time to Periapsis
  // 2 xYz Maneuver Mode:    Time to next maneuver node / Remaining Delta-V for next maneuver node
  // 3 XYz Rendezvouz Mode:  Distance to target / Velocity relative to target
  // 4 xyZ Re-Entry Mode:    Percentage overheating (max) /   Deceleration (G)
  // 5 XyZ Flying Mode:      Altitude / Mach number
  // 6 xYZ Landing Mode:     Radar Altitude / Vertical Velocity
  // 7 XYZ Extra Mode:       not implemented (yet)

  if(digitalRead(pLCDx) && digitalRead(pLCDy) && digitalRead(pLCDz)){
    //MODE 0 : TakeOff Mode
    //Vsurf
    clearLCD();
    char bufferVsurf[17];
    String strVsurf = "Vsurf: ";
    strVsurf += String(VData.Vsurf, 0);
    strVsurf += " m/s";
    strVsurf.toCharArray(bufferVsurf,17);
    writeLCD(bufferVsurf);
    //Acceleration (G)
    jumpToLineTwo();
    char bufferGee[17];
    String strGee = "Accel: ";
    strGee += String(VData.G, 0);
    strGee += " G";
    strGee.toCharArray(bufferGee,17);
    writeLCD(bufferGee);
  }
  
  if(!digitalRead(pLCDx) && digitalRead(pLCDy) && digitalRead(pLCDz)){
    //MODE 1: Orbit Mode
    clearLCD();
    
    //Apoapsis
    char bufferAP[17];
    String strApo = "AP: ";
    if (VData.AP < 10000 && VData.AP > -10000) {
      strApo += String(VData.AP,0);
      strApo += "m ";
    } else if ((VData.AP >= 10000 && VData.AP < 10000000) || (VData.AP <= -10000 && VData.AP > -10000000)) {
      strApo += String((VData.AP / 1000),0);
      strApo += "km ";
    } else if ((VData.AP >= 10000000 && VData.AP < 10000000000) || (VData.AP <= -10000000 && VData.AP > -10000000000)) {
      strApo += String((VData.AP / 1000000),0);
      strApo += "Mm ";
    } else {
      strApo += String((VData.AP / 1000000000),0);
      strApo += "Gm ";
    }
    strApo += String(VData.TAp); //time to apoapsis
    strApo += "s";
    strApo.toCharArray(bufferAP,17);
    writeLCD(bufferAP);
    
    //Periapsis
    char bufferPE[17];
    String strPeri = "PE: ";
    if (VData.PE < 10000 && VData.PE > -10000) {
      strPeri += String(VData.PE,0);
      strPeri += "m ";
    } else if ((VData.PE >= 10000 && VData.PE < 10000000) || (VData.PE <= -10000 && VData.PE > -10000000)) {
      strPeri += String((VData.PE / 1000.0),0);
      strPeri += "km ";
    } else if ((VData.PE >= 10000000 && VData.PE < 10000000000) || (VData.PE <= -10000000 && VData.PE > -10000000000)) {
      strPeri += String((VData.PE / 1000000.0),0);
      strPeri += "Mm ";
    } else {
      strPeri += String((VData.PE / 1000000000.0),0);
      strPeri += "Gm ";
    }
    strPeri += String(VData.TPe); //time to periapsis
    strPeri += "s";
    strPeri.toCharArray(bufferPE,17);
    jumpToLineTwo();
    writeLCD(bufferPE);
  }

  if(digitalRead(pLCDx) && !digitalRead(pLCDy) && digitalRead(pLCDz)){
    //MODE 2: Maneuver Mode
    //MNTime
    clearLCD();
    char t[10];
    dtostrf(VData.MNTime,8,0,t);
    writeLCD("Tnode: ");
    writeLCD(t);
    writeLCD("s");
    //MNDeltaV
    jumpToLineTwo();
    char bufferMNDeltaV[17];
    String strMNDeltaV = "dV: ";
    strMNDeltaV += String(VData.MNDeltaV, 0);
    strMNDeltaV += " m/s";
    strMNDeltaV.toCharArray(bufferMNDeltaV,17);
    writeLCD(bufferMNDeltaV);
  }

  if(!digitalRead(pLCDx) && !digitalRead(pLCDy) && digitalRead(pLCDz)){
    //MODE 3: Rendezvouz Mode
    //Target Distance
    clearLCD();
    char bufferTargetDist[17];
    String strTargetDist = "TDist: ";
    strTargetDist += String(VData.TargetDist, 0);
    strTargetDist += " m";
    strTargetDist.toCharArray(bufferTargetDist,17);
    writeLCD(bufferTargetDist);
    //Target Velocity
    jumpToLineTwo();
    char bufferTargetV[17];
    String strTargetV = "TVel: ";
    strTargetV += String(VData.TargetV, 0);
    strTargetV += " m/s";
    strTargetV.toCharArray(bufferTargetV,17);
    writeLCD(bufferTargetV);
  }

  if(digitalRead(pLCDx) && digitalRead(pLCDy) && !digitalRead(pLCDz)){
    //MODE 4: Re-Entry Mode
    //MaxOverHeat
    clearLCD();
    char bufferMaxOverHeat[17];
    String strMaxOverHeat = "Heat:  ";
    int overheatperc = VData.MaxOverHeat - 48; //convert byte into integer (char 48  is zero).
    strMaxOverHeat += String(overheatperc, 0);
    strMaxOverHeat += " %";
    strMaxOverHeat.toCharArray(bufferMaxOverHeat,17);
    writeLCD(bufferMaxOverHeat);
    //Acceleration (G)
    jumpToLineTwo();
    char bufferGee[17];
    String strGee = "Decel: ";
    strGee += String(VData.G, 0);
    strGee += " G";
    strGee.toCharArray(bufferGee,17);
    writeLCD(bufferGee);
  }

  if(!digitalRead(pLCDx) && digitalRead(pLCDy) && !digitalRead(pLCDz)){
    //MODE 5: Flying Mode
    //Alt
    clearLCD();
    char bufferAtl[17];
    String strAlt = "Alt:  ";
    strAlt += String(VData.Alt, 0);
    strAlt += " m/s";
    strAlt.toCharArray(bufferAtl,17);
    writeLCD(bufferAtl);
    //Mach Number
    jumpToLineTwo();
    char bufferMachNumber[17];
    String strMach = "Mach:";
    strMach += String(VData.G, 0);
    strMach.toCharArray(bufferMachNumber,17);
    writeLCD(bufferMachNumber);
  }

  if(digitalRead(pLCDx) && !digitalRead(pLCDy) && !digitalRead(pLCDz)){
    //MODE 6: Landing Mode
    //RAlt
    clearLCD();
    char bufferRAtl[17];
    String strRAlt = "RAlt: ";
    strRAlt += String(VData.RAlt, 0);
    strRAlt += " m/s";
    strRAlt.toCharArray(bufferRAtl,17);
    writeLCD(bufferRAtl);
    //Vertical Velocity
    jumpToLineTwo();
    char bufferVVI[17];
    String strVVI = "VVI:  ";
    strVVI += String(VData.VVI, 0);
    strVVI += " m/s";
    strVVI.toCharArray(bufferVVI,17);
    writeLCD(bufferVVI);
  }

  if(!digitalRead(pLCDx) && !digitalRead(pLCDy) && !digitalRead(pLCDz)){
    //MODE 7: Extra Mode
    clearLCD();
    writeLCD("KerbalController");
  }
  
  //get in-game status for updating the LED statuses on the controller  
  lights_on = ControlStatus(AGLight);
  gears_on = ControlStatus(AGGears);
  brakes_on = ControlStatus(AGBrakes);
  action1_on = ControlStatus(AGCustom01);
  action2_on = ControlStatus(AGCustom02);
  action3_on = ControlStatus(AGCustom03);
  action4_on = ControlStatus(AGCustom04);
  ladder_on = ControlStatus(AGCustom05);
  solar_on = ControlStatus(AGCustom06);
  chutes_on = ControlStatus(AGCustom07);

  //update button LEDs based on in-game status
  digitalWrite(pLIGHTSLED, lights_on); 
  digitalWrite(pGEARSLED, gears_on);
  digitalWrite(pBRAKESLED, brakes_on);
  digitalWrite(pACTION1LED, action1_on);
  digitalWrite(pACTION2LED, action2_on);
  digitalWrite(pACTION3LED, action3_on);
  digitalWrite(pACTION4LED, action4_on);
  digitalWrite(pLADDERLED, ladder_on);
  digitalWrite(pSOLARLED, solar_on);
  digitalWrite(pCHUTESLED, chutes_on);

  //Fuel Gauges
  vSF = 100 * VData.SolidFuel / VData.SolidFuelTot; //percentage of solid fuel remaining
  vLF = 100 * VData.LiquidFuelS / VData.LiquidFuelTotS; //percentage of liquid fuel remaining in current stage
  vOX = 100 * VData.OxidizerS / VData.OxidizerTotS; //percentage of oxidized remaining in current stage
  vEL = 100 * VData.ECharge / VData.EChargeTot; //percentage of electric charge remaining
  vMP = 100 * VData.MonoProp / VData.MonoPropTot; //percentage of monopropellant remaining

  //scale down to 0-9 for binary calculations
  SF = constrain(map(vSF, 100, 0, 0, 9), 0, 9);
  LF = constrain(map(vLF, 100, 0, 0, 9), 0, 9);
  OX = constrain(map(vOX, 100, 0, 0, 9), 0, 9);
  EL = constrain(map(vEL, 0, 100, 0, 9), 0, 9); //EL bar soldered wrong way around
  MP = constrain(map(vMP, 100, 0, 0, 9), 0, 9);

  //calculate the power of 2. Now each value in binary is all zeroes an a single 1. we can use that to light one LED in each LED bar (dot mode)
  int powOX = 0.1+pow(2,OX);
  int powEL = 0.1+pow(2,EL);
  int powMP = 0.1+pow(2,MP);
  int powSF = 0.1+pow(2,SF);
  int powLF = 0.1+pow(2,LF);

  //map the 8-bit 595 shift registers to the 10-bit LED bars, specific to the way I wired them
  inputBytes[0] = powSF >> 6;
  inputBytes[1] = (powSF << 2) | (powLF >> 8);
  inputBytes[2] = powLF;
  inputBytes[3] = powEL >> 3;
  bitWrite(inputBytes[3], 0, bitRead(powEL, 4)); //fix the skipped 595 pin
  inputBytes[4] = (powEL << 4) | (powMP >> 6);
  inputBytes[5] = (powMP << 2) | (powOX >> 8);
  inputBytes[6] = powOX;

  
  //prepare the shift register
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);

  //loop through the input bytes
  for (int j=0; j<=6; j++){
    byte inputByte = inputBytes[j];
    Serial.println(inputByte);
    shiftOut(dataPin, clockPin, MSBFIRST, inputByte);
  }
  
  //latch the values in when done shifting
  digitalWrite(latchPin, HIGH);  
  
}

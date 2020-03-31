#ifndef COMMS_IO_H
#define COMMS_IO_H

#include <EEPROM.h>

char str[20] = {};
char message[20] = {};

const byte numChars = 32;
String value0;
char receivedChars0[numChars];
boolean newData0 = false;
String value1;
char receivedChars1[numChars];
boolean newData1 = false;

//---------------------------------------------------------------
// BREATHING VARIABLES
//---------------------------------------------------------------

// define bytes to send
unsigned int RR = 20;         // Number of breaths per minute setting
unsigned int VT = 400;        // Tidal volume= target to deliver
unsigned int PK = 35;         // Peak pressure
float IE = 0.5;               // Inspiration-expiration rate
unsigned int PP = 15; //TODO         // PEEP Pressure = Max pressure to deliver
bool Mode = true;            // Mode: true = flow trigger, false = Pressure
bool ACTIVE = true;           // active: start or stop

float PS = 30;                // Support pressure
float RP = 0.5;               // Ramp time
float TI = 1;// TODO                 // Inhale time
float TS = 0;                 // Breath Trigger Sensitivity FLOW
float TP = 2;                 // Breath Trigger Sensitivity PRES

unsigned int ADPK = 10;
unsigned int ADVT = 10;
unsigned int ADPP = 5;

// init booleans for startup
bool RRok = false;
bool VTok = false;
bool PKok = false;
bool TSok = false;
bool IEok = false;
bool PPok = false;
bool Modeok = false;
bool Activeok = false;
bool ADPKok = false;
bool ADVTok = false;
bool ADPPok = false;
bool PSok = false;
bool RPok = false;
bool TIok = false;
bool TPok = false;

//---------------------------------------------------------------
// PYTHON VARIABLES
//---------------------------------------------------------------

unsigned int BPM = 10;      // Breaths per minute
int VOL = 20;               // volume
unsigned int TRIG = 30;     // trigger
int PRES = 40;              // pressure
int FLOW = 50;              // flow

//---------------------------------------------------------------
// EEPROM
//---------------------------------------------------------------

void initEEPROM() {
  EEPROM.get(0, RR);
  EEPROM.get(4, VT);
  EEPROM.get(8, PK);
  EEPROM.get(12, TS);
  EEPROM.get(16, IE);
  EEPROM.get(20, PP);
  EEPROM.get(24, Mode);
  EEPROM.get(28, ACTIVE);

  EEPROM.get(32, ADPKok);
  EEPROM.get(36, ADVTok);
  EEPROM.get(40, ADPPok);

  EEPROM.get(44, PS);
  EEPROM.get(48, RP);
  EEPROM.get(52, TI);
  EEPROM.get(60, TP);
}

//---------------------------------------------------------------
// GETTERS & SETTERS
//---------------------------------------------------------------

unsigned long comms_getInhaleTime(){
  float target_inhale_duration = 1000.0 * 60.0 * IE / RR  ;   
  unsigned long target_inhale_duration_int = (unsigned long) target_inhale_duration;
  return target_inhale_duration_int;
}

unsigned long comms_getExhaleTime(){
  float target_exhale_duration = 1000.0 * 60.0 * (1-IE) / RR  ;   
  unsigned long target_exhale_duration_int = (unsigned long) target_exhale_duration;
  return target_exhale_duration_int;
}

float comms_getPressure(bool inhale_detected){
  if(inhale_detected){
    return PS;
  }
  else{
    return PK;
  }
}

unsigned int comms_getRR() {
  return RR;
}
unsigned int comms_getVT() {
  return VT;
}
unsigned int comms_getPK() {
  return PK;
}
int comms_getTS() {
  return TS;
}
float comms_getIE() {
  return IE;
}
unsigned int comms_getPP() {
  return PP;
}
bool comms_getMode() {
  return Mode;
}
bool comms_getActive() {
  return ACTIVE;
}
unsigned int comms_getADPP() {
  return RR;
}
unsigned int comms_getADVT() {
  return VT;
}
unsigned int comms_getADPK() {
  return PK;
}
unsigned int comms_getPS() {
  return PS;
}
unsigned int comms_getRP() {
  return RP*1000;
}
unsigned int comms_getTI() {
  return TI;
}
unsigned int comms_getTP() {
  return TP;
}

void comms_setBPM(unsigned long bpm_time) {
  BPM = 60000 / (float)bpm_time;
}
void comms_setVOL(int vol) {
  VOL = vol;
}
void comms_setTRIG(unsigned int trig) {
  TRIG = trig;
}
void comms_setPRES(int pres) {
  PRES = pres;
}
void comms_setFLOW(int flow) {
  FLOW = flow;
}

int comms_getVOL(){
  return VOL;
}


//---------------------------------------------------------------
// FUNCTIONS TO PYTHON
//---------------------------------------------------------------

void processPython(String input) {
  value0 = getvalue(input, '=', 1);
  if (input.startsWith("ALARM")) {
    setAlarmState(value0.toInt());
  }
}

void sendDataToPython() {
  strcpy(message, "");
  sprintf(message, "BPM=%d=", BPM * 10);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "VOL=%d=", VOL);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "TRIG=%d=", TRIG);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "PRES=%d=", PRES);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "FLOW=%d=", FLOW);
  getCRC(message);
  Serial.println(message);  
}

void sendInitToPython() {

}

//---------------------------------------------------------------
// FUNCTIONS FROM PYTHON
//---------------------------------------------------------------

bool initCOMM() {
  Serial1.println("SETUP START");
  while (!getSettings()) {
    recvWithEndMarkerSer0();
  }
  Serial1.println("SETUP DONE");
}

bool getSettings() {
  if (!(RRok && VTok && PKok && TSok && IEok && PPok && Modeok && ADPKok && ADVTok && ADPPok)){// && Activeok && PSok && RPok && TIok && TPok)) {
//    Serial1.println("BOOLEANS: ");
//    Serial1.println(RRok);
//    Serial1.println(VTok);
//    Serial1.println(PKok);
//    Serial1.println(TSok);
//    Serial1.println(IEok);
//    Serial1.println(PPok);
//    Serial1.println(Modeok);
//    Serial1.println(ADPKok);
//    Serial1.println(ADVTok);
//    Serial1.println(ADPPok);
    
    recvWithEndMarkerSer0();

    if (!RRok) {
      strcpy(message, "");
      sprintf(message, "RR=%d=", RR);
      getCRC(message);
      Serial.println(message);
    }
    if (!VTok) {
      strcpy(message, "");
      sprintf(message, "VT=%d=", VT);
      getCRC(message);
      Serial.println(message);
    }
    if (!PKok) {
      strcpy(message, "");
      sprintf(message, "PK=%d=", PK);
      getCRC(message);
      Serial.println(message);
    }
    if (!TSok) {
      strcpy(message, "");
      sprintf(message, "TS=%d=", TS);
      getCRC(message);
      Serial.println(message);
    }
    if (!IEok) {
      strcpy(message, "");
      sprintf(message, "IE=%d=", IE);
      getCRC(message);
      Serial.println(message);
    }
    if (!PPok) {
      strcpy(message, "");
      sprintf(message, "PP=%d=", PP);
      getCRC(message);
      Serial.println(message);
    }
    if (!Modeok) {
      strcpy(message, "");
      sprintf(message, "MODE=%d=", Mode);
      getCRC(message);
      Serial.println(message);
    }
    if (!Activeok) {
      strcpy(message, "");
      sprintf(message, "ACTIVE=%d=", ACTIVE);
      getCRC(message);
      Serial.println(message);
    }
    if (!ADPKok) {
      strcpy(message, "");
      sprintf(message, "ADPK=%d=", ADPK);
      getCRC(message);
      Serial.println(message);
    }
    if (!ADVTok) {
      strcpy(message, "");
      sprintf(message, "ADVT=%d=", ADVT);
      getCRC(message);
      Serial.println(message);
    }
    if (!ADPPok) {
      strcpy(message, "");
      sprintf(message, "ADPP=%d=", ADPP);
      getCRC(message);
      Serial.println(message);
    }
    if (!PSok) {
      strcpy(message, "");
      sprintf(message, "PS=%d=", PS);
      getCRC(message);
      Serial.println(message);
    }
    if (!RPok) {
      strcpy(message, "");
      sprintf(message, "RP=%d=", RP);
      getCRC(message);
      Serial.println(message);
    }    
    if (!TIok) {
      strcpy(message, "");
      sprintf(message, "TI=%d=", TI);
      getCRC(message);
      Serial.println(message);
    }     
    if (!TPok) {
      strcpy(message, "");
      sprintf(message, "TP=%d=", TP);
      getCRC(message);
      Serial.println(message);
    }    
    return false;
  }
  else {
    return true;
  }
}

bool resetComm() {
  bool RRok = false;
  bool VTok = false;
  bool PKok = false;
  bool TSok = false;
  bool IEok = false;
  bool PPok = false;
  bool Modeok = false;
  bool Activeok = false;
  bool ADPKok = false;
  bool ADVTok = false;
  bool ADPPok = false;
  bool PS = false;
  bool RP = false;
  bool TI = false;
  bool TP = false;
}

void processSerialPort(String input) {
  value0 = getvalue(input, '=', 1);
  if (input.startsWith("PS")) {
    PS = value0.toInt(); // update value0
    PSok = true;
    EEPROM.put(44, PS);
  }
    if (input.startsWith("RP")) {
    RP = value0.toInt(); // update value0
    RPok = true;
    EEPROM.put(48, RP);
  }
    if (input.startsWith("TI")) {
    TI = value0.toInt(); // update value0
    TIok = true;
    EEPROM.put(52, TI);
  }
    if (input.startsWith("TP")) {
    TP = value0.toInt(); // update value0
    TPok = true;
    EEPROM.put(60, TP);
  }
  if (input.startsWith("ACTIVE")) {
    ACTIVE = value0.toInt(); // update value0
    Activeok = true;
    EEPROM.put(28, ACTIVE);
  }
  if (input.startsWith("ADPK")) {
    ADPK = value0.toInt(); // update value0
    ADPKok = true;
    EEPROM.put(32, ADPKok);
  }
  else if (input.startsWith("ADVT")) {
    ADVT = value0.toInt(); // update value0
    ADVTok = true;
    EEPROM.put(36, ADVTok);
  }
  else if (input.startsWith("ADPP")) {
    ADPP = value0.toInt(); // update value0
    ADPPok = true;
    EEPROM.put(40, ADPPok);
  }
  else if (input.startsWith("MODE")) {
    Mode = value0.toInt(); // update value0
    Modeok = true;
    EEPROM.put(24, Mode);
  }
  else if (input.startsWith("PP")) {
    PP = value0.toInt(); // update value0
    PPok = true;
    EEPROM.put(20, PP);
  }
  else if (input.startsWith("IE")) {
    IE = value0.toFloat(); // update value
    IEok = true;
    EEPROM.put(16, IE);
  }
  else if (input.startsWith("TS")) {
    TS = value0.toInt(); // update value0
    TSok = true;
    EEPROM.put(12, TS);
  }
  else if (input.startsWith("PK")) {
    PK = value0.toInt(); // update value0
    PKok = true;
    EEPROM.put(8, PK);
  }
  else if (input.startsWith("VT")) {
    VT = value0.toInt(); // update value0
    VTok = true;
    EEPROM.put(4, VT);
  }
  else if (input.startsWith("RR")) {
    RR = value0.toInt(); // update value0
    RRok = true;
    EEPROM.put(0, RR);
  }
}

//---------------------------------------------------------------
// FUNCTIONS ALARM
//---------------------------------------------------------------

int sendAlarmState(void) {
  strcpy(message, "");
  sprintf(message, "ALARM=%d=", ALARM);
  getCRC(message);
  Serial.println(message);
  return 1;
}

//---------------------------------------------------------------
// FUNCTIONS SERIAL 0
//---------------------------------------------------------------

void recvWithEndMarkerSer0() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData0 == false) {
    rc = Serial.read();
Serial1.println(rc);
    if (rc != endMarker) {
      receivedChars0[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars0[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData0 = true;
    }
  }

  if (newData0 == true) {
    updateWatchdog(millis());
    // check CRC
    Serial1.println("message received");
    if (!checkCRC(receivedChars0)) {
      processPython(receivedChars0);
      processSerialPort(receivedChars0);
      // confirm message
      Serial.println(receivedChars0);
      Serial1.println(receivedChars0);
    }
    else {
      Serial1.print("Resend data: ");
      Serial1.println(receivedChars0);
    }
    newData0 = false;
  }
}

//---------------------------------------------------------------
// FUNCTIONS SERIAL 1
//---------------------------------------------------------------

void recvWithEndMarkerSer1() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial1.available() > 0 && newData1 == false) {
    rc = Serial1.read();

    if (rc != endMarker) {
      receivedChars1[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars1[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData1 = true;
    }
  }

  if (newData1 == true) {
    // manual input: do not check CRC
    processSerialPort(receivedChars1);
    // confirm message
    Serial1.println(receivedChars1);
    newData1 = false;
  }
}

String getvalue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//---------------------------------------------------------------
// FUNCTIONS CRC
//---------------------------------------------------------------

int getCRC(char* str) {
  char checksum = 0;
  char i;
  int test = 500;

  // calculate CRC with bitwise XOR of each character
  for (i = 0; i < strlen(str); i++) {
    checksum ^= str[i];
  }
  // append CRC (1 byte) to string
  sprintf(str + strlen(str), "%c", checksum);

  return 1;
}

int checkCRC(char* str) {
  char checksum = 0;
  char i;
  // compute bitwise XOR of full string ==> should be 0!
  for (i = 0; i < strlen(str); i++) {
    checksum ^= str[i];
  }
  return checksum;
}

#endif

#ifndef COMMS_IO_H
#define COMMS_IO_H

#include <EEPROM.h>

char str[20] = {};
char message[20] = {};
char counter = 0;

typedef struct{
   char  settingname[10];
   float settingvalue;
   bool  settingok;
   int   eepromloc;
   char  messageid;
   unsigned long messagetime;
} SETTING;  

SETTING settingarray[15]= {
  {"RR", 20, false, 0, 0, 0},
  {"VT", 400, false, 4, 0, 0},
  {"PK", 50, false, 8, 0, 0},
  {"PS", 25, false, 12, 0, 0},
  {"PP", 20, false, 16, 0, 0},
  {"IE", 0.3, false, 20, 0, 0},
  {"RP", 0.5, false, 24, 0, 0},
  {"TS", 10, false, 28, 0, 0},
  {"TP", 2, false, 32, 0, 0},
  {"ADPK", 10, false, 36, 0, 0},
  {"ADVT", 10, false, 40, 0, 0},
  {"ADPP", 5, false, 44, 0, 0},
  {"MODE", 0, false, 48, 0, 0},
  {"ACTIVE", 0, false, 52, 0, 0},
  {"ALARM", 0, false, 56, 0, 0}
};

const byte numChars = 32;
String value0;
char receivedChars0[numChars];
boolean newData0 = false;
String value1;
char receivedChars1[numChars];
boolean newData1 = false;

//---------------------------------------------------------------
// PYTHON VARIABLES
//---------------------------------------------------------------

unsigned int BPM = 10;      // Breaths per minute
int VOL = 20;               // volume
unsigned int TRIG = 0;     // trigger
int PRES = 40;              // pressure
int FLOW = 50;              // flow
int TPRES = 60;              // target pressure

//---------------------------------------------------------------
// EEPROM
//---------------------------------------------------------------

void initEEPROM() {
  for (int i=0; i<15; i++){
    EEPROM.get(settingarray[i].eepromloc, settingarray[i].settingvalue);
  }
}

//---------------------------------------------------------------
// GETTERS & SETTERS SETTINGS
//---------------------------------------------------------------

unsigned long comms_getInhaleTime(){
  float target_inhale_duration = 1000.0 * 60.0 * settingarray[5].settingvalue / settingarray[0].settingvalue  ;   
  unsigned long target_inhale_duration_int = (unsigned long) target_inhale_duration;
  return target_inhale_duration_int;
}

unsigned long comms_getExhaleTime(){
  float target_exhale_duration = 1000.0 * 60.0 * (1-settingarray[5].settingvalue) / settingarray[0].settingvalue  ;   
  unsigned long target_exhale_duration_int = (unsigned long) target_exhale_duration;
  return target_exhale_duration_int;
}

float comms_getPressure(bool inhale_detected){
  if(inhale_detected){
    return settingarray[3].settingvalue;
  }
  else{
    return settingarray[2].settingvalue;
  }
}

unsigned int comms_getRR() {
  return settingarray[0].settingvalue;
}
unsigned int comms_getVT() {
  return settingarray[1].settingvalue;
}
unsigned int comms_getPK() {
  return settingarray[2].settingvalue;
}
int comms_getTS() {
  return settingarray[7].settingvalue;
}
float comms_getIE() {
  return settingarray[5].settingvalue;
}
unsigned int comms_getPP() {
  return settingarray[4].settingvalue;
}
bool comms_getMode() {
  return settingarray[12].settingvalue;
}
bool comms_getActive() {
  if (PYTHON){
    return (bool)settingarray[13].settingvalue;
  }
  else{
    return true;
  }  
}
unsigned int comms_getADPP() {
  return settingarray[9].settingvalue;
}
unsigned int comms_getADVT() {
  return settingarray[10].settingvalue;
}
unsigned int comms_getADPK() {
  return settingarray[11].settingvalue;
}
float comms_getPS() {
  return settingarray[3].settingvalue;
}
float comms_getRP() {
  return settingarray[6].settingvalue*1000;
}
float comms_getTP() {
  return settingarray[8].settingvalue;
}

//---------------------------------------------------------------
// GETTERS & SETTERS MEASUREMENTS
//---------------------------------------------------------------

void comms_setBPM(unsigned long bpm_time) {
  BPM = round(60000.0 / bpm_time);
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
void comms_setTPRES(int tpres) {
  TPRES = tpres;
}

//---------------------------------------------------------------
// FUNCTIONS TO PYTHON
//---------------------------------------------------------------

void sendDataToPython() {
  strcpy(message, "");
  sprintf(message, "BPM=%d=1=", BPM);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "VOL=%d=1=", VOL);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "TRIG=%d=1=", TRIG);
  getCRC(message);
  Serial.println(message);
  comms_setTRIG(0);

  strcpy(message, "");
  sprintf(message, "PRES=%d=1=", PRES);
  getCRC(message);
  Serial.println(message);

  strcpy(message, "");
  sprintf(message, "FLOW=%d=1=", FLOW);
  getCRC(message);
  Serial.println(message);  

  strcpy(message, "");
  sprintf(message, "TPRES=%d=1=", TPRES);
  getCRC(message);
  Serial.println(message); 
}

void sendInitToPython() {

}

//---------------------------------------------------------------
// FUNCTIONS FROM PYTHON
//---------------------------------------------------------------

// Init communication at startup: blocking
bool initCOMM() {
  DEBUGserial.println("SETUP START");
  while (!getSettings()) {
    recvWithEndMarkerSer0();
  }
  DEBUGserial.println("SETUP DONE");
}

// Get settings from python
bool getSettings() {
  bool allsettingsok = true;
  for (int i=0; i<14; i++){
    allsettingsok = allsettingsok && settingarray[i].settingok;
  }
  // check if all settings except alarm are OK
  if (!allsettingsok) {
    for (int i=0; i<14; i++){
      if((!settingarray[i].settingok) && (millis() - settingarray[i].messagetime > 1000)){
        strcpy(message, "");
        sprintf(message, "%s=%d.%d=%c=", settingarray[i].settingname, int(settingarray[i].settingvalue), int(settingarray[i].settingvalue * 100) - int(settingarray[i].settingvalue) * 100, ++counter);
        getCRC(message);
        Serial.println(message);  
        settingarray[i].messageid = counter;     
        settingarray[i].messagetime = millis();
      }
      recvWithEndMarkerSer0();
    }
    return false;
  }
  // if all settings are ok, send the alarm setting and wait for it to be OK
  else if(allsettingsok && settingarray[14].settingok == false){
    if((!settingarray[14].settingok) && (millis() - settingarray[14].messagetime > 1000)){
      strcpy(message, "");
      sprintf(message, "%s=%d.%d=%c=", settingarray[14].settingname, int(settingarray[14].settingvalue), int(settingarray[14].settingvalue * 100) - int(settingarray[14].settingvalue) * 100, ++counter);
      getCRC(message);
      Serial.println(message);  
      settingarray[14].messageid = counter;     
      settingarray[14].messagetime = millis();
    }
    recvWithEndMarkerSer0();
  }
  // if all settings and alarm are OK, return true
  else {
    return true;
  }
}

// reset python communication
bool resetComm() {
  for (int i=0; i<15; i++){
    settingarray[i].settingok = false;
  }
}

// Read incoming data and ack
void processSerialPort(String input) {
  value0 = getvalue(input, '=', 1);
  value1 = getvalue(input, '=', 2);
  char value2[10];
  char value3[10];
  value0.toCharArray(value2, 10);
  value1.toCharArray(value3, 10);
  char id_ack = value2[0];
  char id_msg = value3[0];
  
  for (int i=0; i<15; i++){
    if (input.startsWith(settingarray[i].settingname)) {
      settingarray[i].settingvalue = value0.toFloat();
      EEPROM.put(settingarray[i].eepromloc, settingarray[i].settingvalue);
      // send ack
      strcpy(message, "");
      sprintf(message, "ACK=%c=", id_msg);
      getCRC(message);
      Serial.println(message);       
      settingarray[i].messagetime = millis();
    }
  }
  
  if (input.startsWith("ALARM")) {
    lastWatchdogTime = millis();
  }
  
  if (input.startsWith("ACK")) {
    for (int i=0; i<15; i++){
      if(settingarray[i].messageid == id_ack){
        settingarray[i].settingok = true;
      }
    }
  }
}

//---------------------------------------------------------------
// FUNCTIONS ALARM
//---------------------------------------------------------------

int sendAlarmState(void) {
  strcpy(message, "");
  sprintf(message, "ALARM=%d=%c=", ALARM, ++counter);
  getCRC(message);
  Serial.println(message);
  settingarray[14].messageid = counter;     
  settingarray[14].messagetime = millis();
  return 1;
}

//---------------------------------------------------------------
// FUNCTIONS CPU
//---------------------------------------------------------------

int sendCPUState(void) {
  strcpy(message, "");
  sprintf(message, "CPU=%d=%c=", CPU_TIMER_get(), ++counter);
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
    if (!checkCRC(receivedChars0)) {
      processSerialPort(receivedChars0);
      DEBUGserial.println(receivedChars0);
    }
    else {
      DEBUGserial.print("Resend data: ");
      DEBUGserial.println(receivedChars0);
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

  while (DEBUGserial.available() > 0 && newData1 == false) {
    rc = DEBUGserial.read();

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
    DEBUGserial.println(receivedChars1);
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

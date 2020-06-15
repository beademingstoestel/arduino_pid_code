#ifndef COMMS_IO_H
#define COMMS_IO_H

#include <EEPROM.h>

char str[20] = {};
char message[50] = {};
char counter = 0;

typedef struct{
   char  settingname[10];
   float settingvalue;
   bool  settingok;
   int   eepromloc;
   char  messageid;
   unsigned long messagetime;
} SETTING;  

SETTING settingarray[22]= {
  {"ALARM", 0, false, 64, 0, 0},    // 0  alarm state
  {"RR", 20, false, 0, 0, 0},       // 1  respiratory rate
  {"VT", 400, false, 4, 0, 0},      // 2  tidal volume
  {"PK", 30, false, 8, 0, 0},       // 3  peak pressure
  {"PS", 25, false, 12, 0, 0},      // 4  support pressure
  {"PP", 10, false, 16, 0, 0},      // 5  peep
  {"IE", 0.33, false, 20, 0, 0},    // 6  I/E as float ==> 1:2 = 0.33
  {"RP", 0.5, false, 24, 0, 0},     // 7  Ramp time
  {"TS", 1, false, 28, 0, 0},        // 8  Flow trigger
  {"TP", 15, false, 32, 0, 0},      // 9  Pressure trigger
  {"ADPK", 10, false, 36, 0, 0},    // 10 Peak pressure deviation
  {"ADVT", 50, false, 40, 0, 0},    // 11 Tidal volume deviation
  {"ADPP", 5, false, 44, 0, 0},     // 12 Peep pressure deviation
  {"MODE", 0, false, 48, 0, 0},     // 13 Mode: 0 = pressure triggered, 1 = flow triggered
  {"ACTIVE", 0, false, 52, 0, 0},   // 14 Active: 0 = disabled, 1 = startup peep, 2 = active
  {"MT", 0, false, 56, 0, 0},       // 15 Mute: 0 = no mute / sound, 1 = mute, no sound
  {"FIO2", 0.21, false, 60, 0, 0},  // 16 Oxygen level
  {"ADFIO2", 0.1, false, 64, 0, 0}, // 17 Oxygen level
  {"LPK", 20, false, 64, 0, 0},     // 18 Lower limit PK
  {"HPK", 40, false, 64, 0, 0},     // 19 Upper limit PK
  {"HRR", 35, false, 64, 0, 0},     // 20 Upper limit RR
  {"FW", 3.49, false, 68, 0, 0}     // 21 Firmware version
};

int arr_size = sizeof(settingarray)/sizeof(settingarray[0]);

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

float BPM = 0;            // Breaths per minute
float VOL = 20;           // volume
unsigned int TRIG = 0;    // trigger
float PRES = 40;          // pressure
float FLOW = 50;          // flow
float TPRES = 60;         // target pressure
float FIO2 = 0.21;        // oxygen percentage

//---------------------------------------------------------------
// EEPROM
//---------------------------------------------------------------

void initEEPROM() {
  for (int i=0; i<arr_size; i++){
    EEPROM.get(settingarray[i].eepromloc, settingarray[i].settingvalue);
  }
}

//---------------------------------------------------------------
// GETTERS & SETTERS SETTINGS
//---------------------------------------------------------------

unsigned long comms_getInhaleTime(){
  float target_inhale_duration = 1000.0 * 60.0 * comms_getIE() / comms_getRR();   
  unsigned long target_inhale_duration_int = (unsigned long) target_inhale_duration;
  return target_inhale_duration_int;
}

unsigned long comms_getExhaleTime(){
  float target_exhale_duration = 1000.0 * 60.0 * (1-comms_getIE()) / comms_getRR();   
  unsigned long target_exhale_duration_int = (unsigned long) target_exhale_duration;
  return target_exhale_duration_int;
}

unsigned int comms_getAlarmSatusFromPython() 
{
   return settingarray[0].settingvalue;
}
float comms_getRR() {
  return settingarray[1].settingvalue;
}
unsigned int comms_getVT() {
  return settingarray[2].settingvalue;
}
unsigned int comms_getPK() {
  return settingarray[3].settingvalue;
}
float comms_getPressure(bool inhale_detected){
  if(!inhale_detected){
    return settingarray[3].settingvalue;
  }
  else{
    return settingarray[4].settingvalue;
  }
}
float comms_getPS() {
  return settingarray[4].settingvalue;
}
unsigned int comms_getPP() {
  return settingarray[5].settingvalue;
}
float comms_getIE() {
  return settingarray[6].settingvalue;
}
float comms_getRP() {
  return settingarray[7].settingvalue*1000;
}
int comms_getTS() {
  return settingarray[8].settingvalue;
}
float comms_getTP() {
  return settingarray[9].settingvalue;
}
unsigned int comms_getADPK() {
  return settingarray[10].settingvalue;
}
unsigned int comms_getADVT() {
  return settingarray[11].settingvalue;
}
unsigned int comms_getADPP() {
  return settingarray[12].settingvalue;
}
int comms_getMode() {
  if (comms_getTrigger()){
    return (((int) settingarray[13].settingvalue) & 0x01);
  }
  else{
    return 2;
  }
}
bool comms_getTrigger() {
  return (((int) settingarray[13].settingvalue) >> 1 & 0x01);
}
bool comms_getVolumeLimitControl() {
  return (((int) settingarray[13].settingvalue) >> 2 & 0x01);
}
bool comms_getAPRV() {
  return (((int) settingarray[13].settingvalue) >> 3 & 0x01);
}
bool comms_getAutoFlow() {
  return (((int) settingarray[13].settingvalue) >> 4 & 0x01);
}
int comms_getActive() {
  if (PYTHON){
    return settingarray[14].settingvalue;
  }
  else{
    return 2;
  }  
}
bool comms_resetActive() {
  settingarray[14].settingvalue = 0;
}
float comms_getMT() {
  return settingarray[15].settingvalue;
}
float comms_getFIO2() {
  return settingarray[16].settingvalue;
}
float comms_getADFIO2() {
  return settingarray[17].settingvalue;
}
float comms_getFW() {
  return settingarray[arr_size-1].settingvalue;
}

//---------------------------------------------------------------
// GETTERS & SETTERS MEASUREMENTS
//---------------------------------------------------------------

void comms_setBPM(unsigned long bpm_time) {
  BPM = 60000.0 / bpm_time;
}
void comms_setVOL(float vol) {
  VOL = vol;
}
void comms_setTRIG(unsigned int trig) {
  TRIG = trig;
}
void comms_setPRES(float pres) {
  PRES = pres;
}
void comms_setFLOW(float flow) {
  FLOW = flow;
}
void comms_setTPRES(float tpres) {
  TPRES = tpres;
}
void comms_setFIO2(float fio2) {
  FIO2 = fio2;
}

//---------------------------------------------------------------
// FUNCTIONS TO PYTHON
//---------------------------------------------------------------

void sendDataToPython() {
  int messagelength = 18;
  unsigned long currenttime = millis();
  strcpy(message, "");
  
  message[0] = 0x02;
  message[1] = 0x01;
  message[2] = messagelength;
  message[3] = (char)TRIG;
  message[4] = (char)((int)(VOL*10));
  message[5] = (char)((int)(VOL*10) >> 8);
  message[6] = (char)((int)(PRES*100));
  message[7] = (char)((int)(PRES*100) >> 8);
  message[8] = (char)((int)(TPRES*100));
  message[9] = (char)((int)(TPRES*100) >> 8);
  message[10] = (char)((int)(BPM*100));
  message[11] = (char)((int)(BPM*100) >> 8);
  message[12] = (char)((int)(FLOW*100));
  message[13] = (char)((int)(FLOW*100) >> 8);
  message[14] = (char)((int)(FIO2*100));
  message[15] = (char)((int)(FIO2*100) >> 8);
  message[16] = (char)(currenttime);
  message[17] = (char)(currenttime >> 8);
  message[18] = (char)(currenttime >> 16);
  message[19] = (char)(currenttime >> 24);
  message[20] = getCRCvalue(message, messagelength + 2);
  message[21] = 0x0A;

  comms_setTRIG(0);
  Serial.write(message, 22);
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
  for (int i=1; i<arr_size; i++){
    allsettingsok = allsettingsok && settingarray[i].settingok;
  }
  // check if all settings except alarm are OK
  if (!allsettingsok) {
    for (int i=1; i<arr_size; i++){
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
  else if(allsettingsok && settingarray[0].settingok == false){
    if((!settingarray[0].settingok) && (millis() - settingarray[0].messagetime > 1000)){
      strcpy(message, "");
      sprintf(message, "%s=%u=%c=", settingarray[0].settingname, (unsigned int)(settingarray[0].settingvalue), ++counter);
      getCRC(message);
      Serial.println(message);  
      settingarray[0].messageid = counter;     
      settingarray[0].messagetime = millis();
    }
    recvWithEndMarkerSer0();
    // update watchdog: communication OK
    updateWatchdog(millis());
    return false;
  }
  // if all settings and alarm are OK, return true
  else {
    return true;
  }
}

// reset python communication
bool resetComm() {
  for (int i=0; i<arr_size; i++){
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
  
  for (int i=0; i<arr_size; i++){
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
    updateWatchdog(millis());
  }

  // Get measured PEEP and update PEEP valve if necessary
  if (input.startsWith("PEEP")) {
    //PEEP_update(value0.toFloat());
  }
  
  if (input.startsWith("ACK")) {
    for (int i=0; i<arr_size; i++){
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
  sprintf(message, "ALARM=%u=%c=", ALARM_getAlarmState(), ++counter);
  getCRC(message);
  Serial.println(message);
  settingarray[0].messageid = counter;     
  settingarray[0].messagetime = millis();

  if (ALARM != 0){
    DEBUGserial.print(" ==> ALARM = ");
    DEBUGserial.println(ALARM, BIN);
  }
  
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
    // check CRC
    if (!checkCRC(receivedChars0)) {
      processSerialPort(receivedChars0);
      DEBUGserial.println(receivedChars0);
    } 
    else {
//      DEBUGserial.print("Resend data: ");
//      DEBUGserial.println(receivedChars0);
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

  // calculate CRC with bitwise XOR of each character
  for (i = 0; i < strlen(str); i++) {
    checksum ^= str[i];
  }
  // append CRC (1 byte) to string
  sprintf(str + strlen(str), "%c", checksum);

  return 1;
}

char getCRCvalue(char* str, int strlength) {
  char checksum = 0;
  char i;

  // calculate CRC with bitwise XOR of each character
  for (i = 0; i < strlength; i++) {
    checksum ^= str[i];
  }

  return checksum;
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

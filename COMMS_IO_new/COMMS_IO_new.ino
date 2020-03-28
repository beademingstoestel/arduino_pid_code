#ifndef COMMS_IO_H
#define COMMS_IO_H

#include <EEPROM.h>

unsigned int ALARM = 0;
unsigned int ALARMMASK = 0xFF; // give alarm for all states by default

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
unsigned int RR = 10;         // Number of breaths per minute setting
unsigned int VT = 700;        // Tidal volume= target to deliver
unsigned int PK = 50;         // Peak pressure
int TS = 0; // TODO           // Breath Trigger Sensitivity = amount the machine should look for
float IE = 0.5;               // Inspiration-expiration rate
unsigned int PP = 0;          //PEEP Pressure = Max pressure to deliver
bool Mode = false; // TODO    //Mode: false = Pressure
bool ACTIVE = true;           // active: start or stop

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

//---------------------------------------------------------------
// PYTHON VARIABLES
//---------------------------------------------------------------

unsigned int BPM = 10;      // Breaths per minute
int VOL = 20;               // volume
unsigned int TRIG = 30;     // trigger
int PRES = 40;              // pressure


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("start");
  Serial1.println("start1");

  float test;
  EEPROM.put(0, 10.23556664);
  EEPROM.get(0, test);
  Serial1.println(test, 20);

  EEPROM.get(4, test);
  Serial1.println(test, 20);

  Serial1.println(sizeof(int));

}
void loop() {
  recvWithEndMarkerSer1();
  delay(1000);
}

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
}

//---------------------------------------------------------------
// GETTERS & SETTERS
//---------------------------------------------------------------

unsigned long comms_getInhaleTime() {
  float target_inhale_duration = 1000.0 * 60.0 * IE / RR  ;
  unsigned long target_inhale_duration_int = (unsigned long) target_inhale_duration;
  return target_inhale_duration_int;
}
unsigned long comms_getExhaleTime() {
  float target_exhale_duration = 1000.0 * 60.0 * (1 - IE) / RR  ;
  unsigned long target_exhale_duration_int = (unsigned long) target_exhale_duration;
  return target_exhale_duration_int;
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
}

void sendInitToPython() {

}

//---------------------------------------------------------------
// FUNCTIONS FROM PYTHON
//---------------------------------------------------------------

bool initCOMM0() {
  while (!getSettings()) {
    recvWithEndMarkerSer0();
  }
}

bool getSettings() {
  if (!(RRok && VTok && PKok && TSok && IEok && PPok && Modeok && Activeok && ADPKok && ADVTok && ADPPok)) {
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
}


void processSerialPort(String input) {
  value0 = getvalue(input, '=', 1);

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

void setAlarmState(int alarm) {
  alarmbyte = 0x01 << alarm;
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;

  // BITWISE AND current alarm with mask to check if alarm needs to be triggered
  if (ALARM & ALARMMASK) {
    // SOUND ALARM
    // digitalWrite(ALARMLED, HIGH);
  }
  else {
    //digitalWrite(ALARMLED, LOW);
  }

  strcpy(message, "");
  sprintf(message, "ALARM=%d=", ALARM);
  getCRC(message);
  Serial.println(message);
}

void resetAlarmState(int alarm) {
  // BITWISE AND current alarm with new to RESET
  ALARM &= alarm;
}

int getAlarmState(void) {
  return ALARM;
}

int sendAlarmState(void) {
  strcpy(message, "");
  sprintf(message, "ALARM=%d=", ALARM);
  getCRC(message);
  Serial.println(message);
  return 1;
}

void checkALARM(float pressure, int volume, unsigned long timer, controller_state_t state){
  if (pressure < 5 && state == inhale && timer > 500){
    //no pressure
    setAlarmState(3);
  }
  if (volume < 100 && state == inhale && timer > 500){
    // no flow
    setAlarmState(4);
  }
  if (pressure > comms_getPK() + comms_getADPK()){
    // max pressure exceeded
    setAlarmState(5);
  }
  if (pressure < comms_getPP - comms_getADPP()){
    // Peep deviation exceeded
    setAlarmState(6);
  }
  if (volume > comms_getVT() + comms_getADVT()){
    // max volume exceeded
    setAlarmState(7);
  }
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
    //lastWatchdogTime = millis(); //TODO
    // check CRC
    if (!checkCRC(receivedChars0)) {
      processPython(receivedChars0);
      processSerialPort(receivedChars0);
      // confirm message
      Serial.println(receivedChars0);
    }
    else {
      Serial.println("Resend data");
    }
    newData0 = false;
  }
}

//---------------------------------------------------------------
// FUNCTIONS SERIAL 1
//---------------------------------------------------------------

//only for backward compatibility
bool initCOMM() {
  while (!(RRok && VTok && PKok && TSok && IEok && PPok && Modeok && Activeok && ADPKok && ADVTok && ADPPok)) {
    recvWithEndMarkerSer1();
  }
  delay(20);
  Serial1.println("OK");
  delay(20);
  Serial1.println("OK");
  delay(20);
  Serial1.println("OK");
  return true;
}

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
    processSerialPort(receivedChars0);
    // confirm message
    Serial1.println(receivedChars0);
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

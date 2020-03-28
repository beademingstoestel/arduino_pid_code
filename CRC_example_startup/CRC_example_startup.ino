char str[20] = {};
char message[20] = {};

const byte numChars = 32;
String value0;
char receivedChars0[numChars];
boolean newData0 = false;
enum {setBPM = 0x00, setVOL = 0x01, setTRIG = 0x02, setPRES = 0x03};

//---------------------------------------------------------------
// BREATHING VARIABLES
//---------------------------------------------------------------

// define bytes to send
unsigned int RR = 10; 
// Number of breaths per minute setting
unsigned int VT = 700;
// Tidal volume= target to deliver
unsigned int PK = 50;
//Peak pressure
int TS = 0; // TODO
// Breath Trigger Sensitivity = amount the machine should look for
float IE = 0.5;
// Inspiration-expiration rate
unsigned int PP = 0; // NOT USED
//PEEP Pressure = Max pressure to deliver
bool Mode = false; //false = Pressure // TODO
//Mode
bool ACTIVE = true;
// active: start or stop

unsigned int ADPK = 10;
unsigned int ADVT = 10;
unsigned int ADPP = 5;

// init booleans for startup
bool RRok = false;
bool VTok = true;
bool PKok = true;
bool TSok = true;
bool IEok = true;
bool PPok = true;
bool Modeok = true;
bool Activeok = true;
bool ADPKok = true;
bool ADVTok = true;
bool ADPPok = true;

//---------------------------------------------------------------
// PYTHON VARIABLES
//---------------------------------------------------------------

unsigned int ALARM = 0;

// send value * 10!!!
unsigned int BPM = 10;
// Breaths per minute
int VOL = 20;
// volume
unsigned int TRIG = 30;
// trigger
int PRES = 40;
// pressure

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello");

  initCOMM();
}

//---------------------------------------------------------------
// LOOP
//---------------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  recvWithEndMarkerSer0();
  delay(1000);
  sendDataToPython();
  BPM++;
  VOL++;
  PRES++;
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

int checkCRC(char* str){
  char checksum = 0;
  char i;
  // compute bitwise XOR of full string ==> should be 0!
  for (i = 0; i < strlen(str); i++) {
        checksum ^= str[i];
  }
  return checksum;
}

//---------------------------------------------------------------
// FUNCTIONS TO PYTHON
//---------------------------------------------------------------

void processPython(String input){
  value0 = getvalue(input, '=', 1); 
  if (input.startsWith("ALARM")){
    setAlarmState(value0.toInt());
  }
}

void sendDataToPython(){
  strcpy(message, "");
  sprintf(message, "BPM=%d=", BPM*10);
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

void sendInitToPython(){
  
}

//---------------------------------------------------------------
// FUNCTIONS FROM PYTHON
//---------------------------------------------------------------

bool initCOMM(){
  while(!(RRok && VTok && PKok && TSok && IEok && PPok && Modeok && Activeok && ADPKok && ADVTok && ADPPok)){
    recvWithEndMarkerSer0();

    if(!RRok){
        strcpy(message, "");
        sprintf(message, "RR=%d=", RR);
        getCRC(message);
        Serial.println(message);
    }
    if(!VTok){
        strcpy(message, "");
        sprintf(message, "VT=%d=", VT);
        getCRC(message);
        Serial.println(message);
    }
    if(!PKok){
        strcpy(message, "");
        sprintf(message, "PK=%d=", PK);
        getCRC(message);
        Serial.println(message);
    }    
    if(!TSok){
        strcpy(message, "");
        sprintf(message, "TS=%d=", TS);
        getCRC(message);
        Serial.println(message);
    }
    if(!IEok){
        strcpy(message, "");
        sprintf(message, "IE=%d=", IE);
        getCRC(message);
        Serial.println(message);
    }
    if(!PPok){
        strcpy(message, "");
        sprintf(message, "PP=%d=", PP);
        getCRC(message);
        Serial.println(message);
    }
    if(!Modeok){
        strcpy(message, "");
        sprintf(message, "MODE=%d=", Mode);
        getCRC(message);
        Serial.println(message);
    }
    if(!Activeok){
        strcpy(message, "");
        sprintf(message, "ACTIVE=%d=", ACTIVE);
        getCRC(message);
        Serial.println(message);
    }
    if(!ADPKok){
        strcpy(message, "");
        sprintf(message, "ADPK=%d=", ADPK);
        getCRC(message);
        Serial.println(message);
    }
    if(!ADVTok){
        strcpy(message, "");
        sprintf(message, "ADVT=%d=", ADVT);
        getCRC(message);
        Serial.println(message);
    }
    if(!ADPPok){
        strcpy(message, "");
        sprintf(message, "ADPP=%d=", ADPP);
        getCRC(message);
        Serial.println(message);
    }
  }
  return true;
}

void processSerialPort(String input){
  value0 = getvalue(input, '=', 1);

  if (input.startsWith("ACTIVE")){
    ACTIVE = value0.toInt(); // update value0
    Activeok = true;
  }
  if (input.startsWith("ADPK")){
    ADPK = value0.toInt(); // update value0
    ADPKok = true;
  }
  else if (input.startsWith("ADVT")){
    ADVT = value0.toInt(); // update value0
    ADVTok = true;
  }
  else if (input.startsWith("ADPP")){
    ADPP = value0.toInt(); // update value0
    ADPPok = true;
  }
  else if (input.startsWith("MODE")){
    Mode = value0.toInt(); // update value0
    Modeok = true;
  }
  else if (input.startsWith("PP")){
    PP = value0.toInt(); // update value0
    PPok = true;
  }
  else if (input.startsWith("IE")){
    IE = value0.toFloat(); // update value
    IEok = true;
  }
  else if (input.startsWith("TS")){
    TS = value0.toInt(); // update value0
    TSok = true;
  }
  else if (input.startsWith("PK")){
    PK = value0.toInt(); // update value0
    PKok = true;
  }
  else if (input.startsWith("VT")){
    VT = value0.toInt(); // update value0
    VTok = true;
  }  
  else if (input.startsWith("RR")){
    RR = value0.toInt(); // update value0
    RRok = true;
  }
}

//---------------------------------------------------------------
// FUNCTIONS ALARM
//---------------------------------------------------------------

void setAlarmState(int alarm){
  if (alarm >= ALARM){ // don't overwrite alarm with 0 if alarm state exists!
    if (alarm >= ALARM){
      ALARM = alarm;
    }
  }
}

int getAlarmState(void){
  return ALARM;
}

//---------------------------------------------------------------
// FUNCTIONS SERIAL
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
       Serial.println(receivedChars0);
     }
   }

   if (newData0 == true){
      //lastWatchdogTime = millis(); //TODO
      // check CRC
      if(!checkCRC(receivedChars0)){
        processPython(receivedChars0);
        processSerialPort(receivedChars0);
        // confirm message
        Serial.println(receivedChars0);
      }
      else{
        Serial.println("Resend data");
      }
      newData0 = false;
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
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

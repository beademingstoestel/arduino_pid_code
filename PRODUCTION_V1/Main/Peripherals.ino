void initPeripherals(){
  initFan();
  initLight();
  initSpeaker();
  initValve();
  PEEP_motor_init();
}


/////////////////////Support functions for SPEAKER
#ifdef Speaker_PWM

unsigned long SpeakerTimeStamp;
bool SpeakerBeepState = false;
int SpeakerBeepLength;
bool isSpeakerOn = false;

void initSpeaker(){
  pinMode(Speaker_PWM, OUTPUT);
}

void SpeakerOn() {
  //analogWrite(Speaker_PWM, 127); //Turn on PWM @50% duty
  isSpeakerOn = true;
}
void SpeakerOff() {
  analogWrite(Speaker_PWM, 0); //Turn set duty to 0;
  SpeakerBeepState = false;
  isSpeakerOn = false;
}

void SpeakerBeep(int lengthInMillis){
  SpeakerBeepLength  = lengthInMillis; //store the lenght of the BEEP
  SpeakerBeepState = true; //set beep to true
  SpeakerTimeStamp = millis(); //store timestamp
  SpeakerOn(); //set speaker ON
}

void doBeepingAlarm() { //added by Lieven 23-04 to implement beeps
  if (SpeakerBeepState == true) {
    analogWrite(Speaker_PWM, 127); //Turn on PWM @50% duty
    if(millis()-SpeakerTimeStamp > SpeakerBeepLength){
      SpeakerOff();
    }
    return;
  }

  if (isSpeakerOn == true && (millis() % 2000 < 60 || (millis() % 2000 > 120 && millis() % 2000 < 180) || (millis() % 2000 > 300 && millis() % 2000 < 330))) {
    analogWrite(Speaker_PWM, 127); //Turn on PWM @50% duty
  }
  else {
    analogWrite(Speaker_PWM, 0); //Turn off

  }
}

#else
void SpeakerOn() {}   //if no speakerpin defined do nothing
void SpeakerOff() {}   //if no speakerpin defined do nothing
void SpeakerTimingSupportRoutine(){}
void SpeakerBeep(int lengthInMillis){}
#endif



/////////////////////Support functions for Light
#ifdef Light_PWM
void initLight(){
  pinMode(Light_PWM, OUTPUT);
}

void LightOn() {
  Timer3.pwm(Light_PWM, 1023);
//  analogWrite(Light_PWM, 255); //Turn on PWM @100% duty
}
void LightOnPWM(uint8_t intensity) {
  Timer3.pwm(Light_PWM, intensity*4);
//  analogWrite(Light_PWM, intensity); //Turn on PWM
}
void LightOff() {
  Timer3.pwm(Light_PWM, 0);
//  analogWrite(Light_PWM, 0); //Turn set duty to 0;
}
#else
void LightOn() { }
void LightOnPWM(uint8_t intensity) { }
void LightOff() { }
#endif


/////////////////////Support functions for FAN
#ifdef Fan_PWM
void initFan(){
  pinMode(Fan_PWM, OUTPUT);
  pinMode(fan_speed, INPUT_PULLUP);
  FanOnPWM(FANPWMSETTING);
}

void FanOn() {
  Timer3.pwm(Fan_PWM, 1023);
//  analogWrite(Fan_PWM, 255); //Turn on PWM @100% duty
}
void FanOnPWM(uint8_t intensity) {
  Timer3.pwm(Fan_PWM, intensity*4);
//  analogWrite(Fan_PWM, intensity); //Turn on PWM
}
void FanOff() {
  Timer3.pwm(Fan_PWM, 0);
//  analogWrite(Fan_PWM, 0); //Turn set duty to 0;
}
bool FanState = true;
int FanCounterLow = 10;
int FanCounterHigh = 10;
const int FanTimeout = 3000; //sampling period
const int FanThreshold = 5; //minimum amount of counts over sampling period
unsigned long FanTimeStamp = 0;


bool FanPollingRoutine(){  // Run in LOOP, polls RPM pin, and returns state of the fan
  //crude way of detecting fan is running. Sample RPM pin regularly, and if over a certain period amount of samples not reached, FAN is not running
  if(digitalRead(fan_speed)){
    FanCounterHigh++;
  }
  else{
    FanCounterLow++;
  }

  if(millis()-FanTimeStamp > FanTimeout){ //if measuringperiod elapsed
    if(FanCounterLow > FanThreshold && FanCounterHigh > FanThreshold){ //if required amount of counts not reached
      FanState = true;
    }else{
      FanState = false;
    }
    FanCounterLow = 0; //reset counter
    FanCounterHigh = 0;
    FanTimeStamp = millis(); //new timestamp
  }
  return FanState;
}
#else
void initFan() { }
void FanOn() { }
void FanOnPWM(uint8_t intensity) { }
void FanOff() { }
bool FanPollingRoutine(){
  return true;
}
#endif

#ifdef main_supply_voltage
int MainSupplyVoltage() {
  int ADCvalue = analogRead(main_supply_voltage); //readout adc
  int scaledVoltage =  map(ADCvalue, 0, 1023, 0, 5000); //rescale ADC value to 5V Vref in millivolts
  return scaledVoltage * MainSupplyVoltageScaling;
}
#else
int MainSupplyVoltage() {
  return 24000; //return safe value
}
#endif

#ifdef PSU_supply_voltage
int PSUSupplyVoltage() {
  int ADCvalue = analogRead(PSU_supply_voltage); //readout adc
  int scaledVoltage =  map(ADCvalue, 0, 1023, 0, 5000); //rescale ADC value to 5V Vref in millivolts
  return scaledVoltage * PSUSupplyVoltageScaling;
}

bool OnMainsPower(){
  int ADCvalue = analogRead(main_supply_voltage); //readout adc
  int scaledVoltage =  map(ADCvalue, 0, 1023, 0, 5000); //rescale ADC value to 5V Vref in millivolts
  if (scaledVoltage * MainSupplyVoltageScaling < 10000){ // check if scaled mains voltage smaller than 10V
    return false;
  }else{
    return true;
  }
}
#else
int PSUSupplyVoltage() {
  return 24000; //return safe value
}
bool OnMainsPower(){
  return true; //if no way to check return true;
}
#endif

// ---------- Endswitches
#ifdef endswitches_inverted
bool read_endswitch_stop() {
  return !digitalRead(ENDSWITCH_FULL_PIN);
}
bool read_endswitch_start() {
  return !digitalRead(ENDSWITCH_PUSH_PIN);
}
#else
bool read_endswitch_stop() {
  return digitalRead(ENDSWITCH_FULL_PIN);
}
bool read_endswitch_start() {
  return digitalRead(ENDSWITCH_PUSH_PIN);
}
#endif

// ###################################################################
unsigned long turn_time_total = 0;
unsigned long turn_time_start = 0;

#ifdef automatic_peep
void PEEP_motor_init(){
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  
  analogWrite(46, LOW);
  analogWrite(44, 50); 
  delay(500);
  analogWrite(44, LOW);
  analogWrite(46, 50); 
  delay(500);
  analogWrite(46, LOW);
  analogWrite(44, LOW); 
  
  // TODO: place in pinout.h
}

void PEEP_turn_motor(int turn_direction, int turn_time){
  if(turn_time > 100){
    if(turn_direction == 1){
      // start motor cw
      analogWrite(44, LOW);
      analogWrite(46, 50);       
    }
    else{
      // start motor ccw
      analogWrite(46, LOW);
      analogWrite(44, 50);
    }
    turn_time_total = turn_time;
    turn_time_start = millis();
  }
}

void PEEP_check_motor(){
  if(millis() - turn_time_start > turn_time_total){
     // stop motor
    analogWrite(44, LOW);
    analogWrite(46, LOW);
  }
}
#else

void PEEP_motor_init(){}
void PEEP_turn_motor(int turn_direction, int turn_time){}
void PEEP_check_motor(){}

#endif
static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

// valves


// ---------- Valves
unsigned long ValveTime = 0;
unsigned long ValveStartTime = 0;

bool initValve(){
  pinMode(O2_valve, OUTPUT);
  digitalWrite(O2_valve, LOW);
  pinMode(O2_safety_valve, OUTPUT);
  digitalWrite(O2_safety_valve, HIGH);
}

bool ValveOn(){
  digitalWrite(O2_valve, HIGH);
}

bool safetyValveOn(){
  digitalWrite(O2_safety_valve, HIGH);
}

bool ValveOn(unsigned long valvetime){
  ValveTime = valvetime;
  ValveStartTime = millis();
  if (ValveTime > 1){
    ValveOn();
  }
}

bool ValveOff(){
  digitalWrite(O2_valve, LOW);
}

bool safetyValveOff(){
  digitalWrite(O2_safety_valve, LOW);
}

bool safetyValveState(){
  return digitalRead(O2_safety_valve);
}

// check if we need to turn off
// keep track of max volume to patient
void ValveCheck(){
  if (millis() - ValveStartTime > ValveTime){ 
    ValveOff();
  }
}

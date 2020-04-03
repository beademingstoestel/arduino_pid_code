void initPeripherals(){
  initFan();
  initLight();
  initSpeaker();
}


/////////////////////Support functions for SPEAKER
#ifdef Speaker_PWM

unsigned long SpeakerTimeStamp;
bool SpeakerBeepState = false;
int SpeakerBeepLength;

void initSpeaker(){
  pinMode(Speaker_PWM, OUTPUT);
}

void SpeakerOn() {
  analogWrite(Speaker_PWM, 127); //Turn on PWM @50% duty
}
void SpeakerOff() {
  analogWrite(Speaker_PWM, 0); //Turn set duty to 0;
  SpeakerBeepState = false;
}

void SpeakerBeep(int lengthInMillis){
  SpeakerBeepLength  = lengthInMillis; //store the lenght of the BEEP
  SpeakerBeepState = true; //set beep to true
  SpeakerTimeStamp = millis(); //store timestamp
  SpeakerOn(); //set speaker ON
}
//Call this function regularly in MAIN
void SpeakerTimingSupportRoutine(){
  if(!SpeakerBeepState) return; //if not beeping return immidiately

  if(millis()-SpeakerTimeStamp > SpeakerBeepLength){ //if time has elapsed, turn of the speaker
    SpeakerOff();
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
  analogWrite(Light_PWM, 255); //Turn on PWM @100% duty
}
void LightOnPWM(uint8_t intensity) {
  analogWrite(Light_PWM, intensity); //Turn on PWM
}
void LightOff() {
  analogWrite(Light_PWM, 0); //Turn set duty to 0;
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
  analogWrite(Fan_PWM, 255); //Turn on PWM @100% duty
}
void FanOnPWM(uint8_t intensity) {
  analogWrite(Fan_PWM, intensity); //Turn on PWM
}
void FanOff() {
  analogWrite(Fan_PWM, 0); //Turn set duty to 0;
}
bool FanState = true;
int FanCounter = 0;
const int FanTimeout = 1000; //sampling period
const int FanThreshold = 5; //minimum amount of counts over sampling period
unsigned long FanTimeStamp = 0;


bool FanPollingRoutine(){  // Run in LOOP, polls RPM pin, and returns state of the fan
  //crude way of detecting fan is running. Sample RPM pin regularly, and if over a certain period amount of samples not reached, FAN is not running
  if(analogRead(fan_speed) < 100){
    FanCounter++;
  }

  if(millis()-FanTimeStamp > FanTimeout){ //if measuringperiod elapsed
    if(FanCounter < FanThreshold){ //if required amount of counts not reached
      FanState = false;
    }else{
      FanState = true;
    }
    FanCounter = 0; //reset counter
    FanTimeStamp = millis(); //new timestamp
  }
  return FanState;
}
#else
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
  int ADCvalue = analogRead(PSU_supply_voltage); //readout adc
  int scaledVoltage =  map(ADCvalue, 0, 1023, 0, 5000); //rescale ADC value to 5V Vref in millivolts
  if (scaledVoltage > 1000){ // check if scaled PSU voltage larger than 1V
    return true;
  }else{
      return false;
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

#include "TimerThree.h"
#include "PINOUT.h"
// for debuggin purposes: allows to turn off features
#define PYTHON 1
#define HARDWARE 0
#define DEBUGserial Serial
 
//---------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------

unsigned long controllerTime = 10000; // us 
unsigned long interrupttime; 

volatile float CurrentPressurePatient = 0;
volatile float CurrentFlowPatient = 0;
volatile float Volume2Patient = 0;
volatile float CurrentVolumePatient = 0;

volatile float CurrentFlowOxygen = 0;
volatile float CurrentVolumeOxygen = 0;

float target_fio2 = 0.2;

typedef enum {ini = 0x00, wait = 0x01, inhale = 0x02, exhale = 0x03} controller_state_t;
controller_state_t controller_state = 0x00;

volatile unsigned long exhale_start_time = millis();
volatile unsigned long inhale_start_time = millis();
volatile unsigned long time_diff = 1;
float Speed; 
bool inhale_detected = 0;
int transientMute = 0;
int transientMuteCycles = 5;

float target_risetime = 500;           
unsigned int target_pressure = 20; 
unsigned int target_volume = 700;
unsigned int target_inhale_time = 0;
unsigned int target_exhale_time = 0;
unsigned int trigger_mode = 0;

//---------------------------------------------------------------
// SAFETY FEATURE
//---------------------------------------------------------------

//safety volume and pressure sensor check
float maxpressureinhale=-99999.9;   // initialize low negative--> no error on start-up
float minpressureinhale=99999.9;    // initialize high positive --> no error on start-up
float maxpressure=-99999.9;         // initialize low negative--> no error on start-up
float minpressure=99999.9;          // initialize high positive --> no error on start-up
float maxflowinhale=-99999.9;       // initialize low negative--> no error on start-up
float minflowinhale=99999.9;        // initialize high positive --> no error on start-up
float maxflow=-99999.9;             // initialize low negative--> no error on start-up
float minflow=99999.9;              // initialize high positive --> no error on start-up

float battery_SoC = 1.0;   
float main_supply = 0.0;
float batt_supply = 0.0;
   
// safety minimum degraded mode
bool min_degraded_mode_ON = true;
bool temperature_OK = false;
bool battery_above_25 = false;
bool fan_OK = true;   
bool battery_powered = false;
bool motor_sens_init_ok = false;
bool hall_sens_init_ok = false;
bool pressure_sens_init_ok = false;
bool flow_sens_init_ok = false;
bool isFlow2PatientRead = false;
bool isPatientPressureCorrect = false;
bool isAngleOK = false;
bool isPythonOK = false;
bool isAmbientPressureCorrect = false;
bool oxygen_init_ok = false;
bool isFlowOfOxygenRead = false;

unsigned long starttime = millis();

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  DEBUGserial.begin(115200); 

  Timer3.initialize(controllerTime);   // initialize timer3 in us, set 10 ms timing
  
  DEBUGserial.println("=== TEST SCRIPT ===");

  //-- set up peripherals
  initPeripherals();
  FanOnPWM(0);

  //--- check mains supply and battery voltage
  main_supply = MainSupplyVoltage()/1000;
  batt_supply = PSUSupplyVoltage()/1000;
  battery_SoC = batt_supply/25;

  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("1) POWER SUPPLIES");
  DEBUGserial.print("  - Mains voltage input: ");
  DEBUGserial.print(main_supply);
  DEBUGserial.println(" V");
  DEBUGserial.print("  - Battery voltage input: ");
  DEBUGserial.print(batt_supply);
  DEBUGserial.println(" V");

  REQUEST_INPUT();

  //-- set up oxygen sensor
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("2) OXYGEN SENSORS");
  int oxygen_sensor_init_value = OXYGEN_SENSOR_INIT();
  if (oxygen_sensor_init_value & 0x01) {
    flow_sens_init_ok = true;
    DEBUGserial.println("  - OXYGEN INHALE SENSOR OK");
  }
  else {
    DEBUGserial.println("  - OXYGEN INHALE SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  if (oxygen_sensor_init_value & 0x02) {
    flow_sens_init_ok = true;
    DEBUGserial.println("  - OXYGEN EXHALE SENSOR OK");
  }
  else {
    DEBUGserial.println("  - OXYGEN EXHALE SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  REQUEST_INPUT();
    
  //--- set up flow sensor
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("3) FLOW SENSOR");
  int flow_sensor_init_value = FLOW_SENSOR_INIT();
  if (flow_sensor_init_value & 0x01) {
    flow_sens_init_ok = true;
    DEBUGserial.println("  - TUBE FLOW SENSOR OK");
    DEBUGserial.println("  - MEASURING FLOW TUBE...");
    starttime = millis();
    while(millis() - starttime < 1000){
      FLOW_SENSOR_MeasurePatient(&CurrentFlowPatient,maxflowinhale,minflowinhale);
      DEBUGserial.print("     - ");
      DEBUGserial.println(CurrentFlowPatient);
      delay(100);
    }
  }
  else {
    DEBUGserial.println("  - TUBE FLOW SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  if (flow_sensor_init_value & 0x02) {
    flow_sens_init_ok = true;
    DEBUGserial.println("  - OXYGEN FLOW SENSOR OK");
    DEBUGserial.println("  - MEASURING FLOW OXYGEN...");
    starttime = millis();
    while(millis() - starttime < 1000){
      FLOW_SENSOR_MeasureO2(&CurrentFlowOxygen);;
      DEBUGserial.print("     - ");
      DEBUGserial.println(CurrentFlowPatient);
      delay(100);
    }
  }
  else {
    DEBUGserial.println("  - OXYGEN FLOW SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  REQUEST_INPUT();

  //-- set up pressure sensors
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("4) PRESSURE SENSOR");
  int pressure_sensor_init_value = PRESSURE_SENSOR_INIT();
  if (pressure_sensor_init_value & 0x01){
    pressure_sens_init_ok = true;
    DEBUGserial.println("  - TUBE PRESSURE SENSOR OK");
    DEBUGserial.println("  - MEASURING TUBE SENSOR...");
    starttime = millis();
    while(millis() - starttime < 1000){
      BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
      DEBUGserial.print("     - ");
      DEBUGserial.println(CurrentPressurePatient);
      delay(100);
    }
  }
  else{
    DEBUGserial.println("  - TUBE PRESSURE SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  if (pressure_sensor_init_value & 0x02){
    pressure_sens_init_ok = true;
    DEBUGserial.println("  - AMBIENT PRESSURE SENSOR OK");
    DEBUGserial.println("  - MEASURING AMBIENT SENSOR...");
    starttime = millis();
    while(millis() - starttime < 1000){
      float ambientpressure = BME280_readPressureAmbient();
      DEBUGserial.print("     - ");
      DEBUGserial.println(ambientpressure);
      delay(100);
    }
  }
  else{
    DEBUGserial.println("  - AMBIENT PRESSURE SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  REQUEST_INPUT();

  //-- set up motor
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("5) MOTOR & ENDSWITCHES");
  if (MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN)) {
    motor_sens_init_ok = true;
    DEBUGserial.println("  - MOTOR & ENDSWITCHES OK");
  }
  else {
    DEBUGserial.println("  - MOTOR & ENDSWITCHES FAILED");
    if(HARDWARE)ALARM_init();
  }

  // empty oxygen bag
  MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);
  delay(500);

  REQUEST_INPUT();

  //-- set up oxygen supply
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("6) OXYGEN SUPPLY");
  
  ValveOn();
  unsigned long valvestarttime = millis();
  unsigned int valveinittime = 500;
  int mincalibrationvolume = 50;
  int counter = 0;
  while(millis() - valvestarttime < valveinittime){
    FLOW_SENSOR_MeasureO2(&CurrentFlowOxygen);
    FLOW_SENSOR_updateVolumeO2init(CurrentFlowOxygen);
    counter++;
  }
  ValveOff();
  
  // calculate K_O2
  float sampletime = (float) valveinittime / counter;
  float calibrationvolume = FLOW_SENSOR_getTotalVolumeIntO2() * sampletime;

  FLOW_SENSOR_setK_O2(0.0); 
  FLOW_SENSOR_resetVolumeO2();

  if (calibrationvolume > mincalibrationvolume) {
    oxygen_init_ok = true; 
    DEBUGserial.println("  - OXYGEN SUPPLY OK");
  }
  else {
    DEBUGserial.println("  - OXYGEN SUPPLY FAILED");
    if(HARDWARE)ALARM_init();
  }

  // empty oxygen bag
  MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);
  MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);
  delay(500);

  REQUEST_INPUT();

  //-- check alarms
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("7) TEMPERATURE");
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  if(temperature_OK){
    DEBUGserial.println("  - TEMPERATURE OK");
    DEBUGserial.println("  - MEASURING...");
    starttime = millis();
    while(millis() - starttime < 1000){
      DEBUGserial.print("     - ");
      DEBUGserial.println(BME_280_GET_TEMPERATURE());
      delay(100);
    }
  }
  else{
    DEBUGserial.println("  - TEMPERATURE FAILED");
    if(HARDWARE)ALARM_init();
  }

  REQUEST_INPUT();  

  //-- check FAN
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("8) FAN");
  // fan off!
  DEBUGserial.println("  - MEASURING SPEED 0...");
  starttime = millis();
  while(millis() - starttime < 2000){
      fan_OK = FanPollingRoutine();
      delay(1);
  }
  if(!fan_OK){
    DEBUGserial.println("  - FAN SPEED 0 OK");
  }
  else{
    DEBUGserial.println("  - FAN SPEED 0 FAILED");
  }
  FanOnPWM(100);
  delay(500);
  
  DEBUGserial.println("  - MEASURING SPEED 1...");
  starttime = millis();
  while(millis() - starttime < 2000){
      fan_OK = FanPollingRoutine();
      delay(1);
  }
  if(fan_OK){
    DEBUGserial.println("  - FAN SPEED 1 OK");
  }
  else{
    DEBUGserial.println("  - FAN SPEED 1 FAILED");
  }

  FanOnPWM(200);
  DEBUGserial.println("  - MEASURING SPEED 2...");
  starttime = millis();
  while(millis() - starttime < 2000){
      fan_OK = FanPollingRoutine();
      delay(1);
  }
  if(fan_OK){
    DEBUGserial.println("  - FAN SPEED 2 OK");
  }
  else{
    DEBUGserial.println("  - FAN SPEED 2 FAILED");
  }
  FanOnPWM(100);

  REQUEST_INPUT();

  //-- check LIGHT & BUZZER
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("9) LIGHT & BUZZER");
  DEBUGserial.println("  - BLINKING LIGHT");
  LightOn();
  delay(1000);
  LightOff();
  DEBUGserial.println("  - BUZZING BUZZER");
  SpeakerOn();
  delay(1000);
  SpeakerOff();
  
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("=== TEST FINISHED===");

while(1);
  
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  //checkALARM_init(pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, hall_sens_init_ok, 
//                  fan_OK, battery_powered, battery_SoC, temperature_OK);
    
  //-- set up communication with screen
  if(PYTHON) initCOMM();
  if (!PYTHON) isPythonOK = true;

  //-- set up interrupt
  Timer3.initialize(controllerTime);   // initialize timer3 in us, set 10 ms timing
  Timer3.attachInterrupt(controller);  // attaches callback() as a timer overflow interrupt
}

//---------------------------------------------------------------
// LOOP
//---------------------------------------------------------------

void loop()
{
  // Handle uart send to PC
  if (PYTHON) sendDataToPython();
  // Handle uart receive from PC
  recvWithEndMarkerSer0();
  // Check alarm and watchdog
  if (PYTHON) doWatchdog();
  if (PYTHON) doCPU_TIMER();
  // Handle uart receive for debugging
  if (!PYTHON) recvWithEndMarkerSer1();

  // update ambient pressure and temperature
  isAmbientPressureCorrect = BME_280_UPDATE_AMBIENT();
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  // check fan
  fan_OK = FanPollingRoutine();
  // check buzzer
//  SpeakerTimingSupportRoutine();
  // delay loop to avoid full serial buffers
  delay(50); 
}

// ---------------------------------------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------------------------------------

void controller()
{
  CPU_TIMER_start(millis());
  
  // readout sensors
  interrupts();
  //isFlow2PatientRead = FLOW_SENSOR_Measure(&CurrentFlowPatient,maxflowinhale,minflowinhale);
  isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
  isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  noInterrupts();
  
  // update volume 
  FLOW_SENSOR_updateVolume(CurrentFlowPatient);
  FLOW_SENSOR_getVolume(&CurrentVolumePatient);

  // update python values
  comms_setFLOW(CurrentFlowPatient);
  comms_setVOL(CurrentVolumePatient);
  comms_setPRES(CurrentPressurePatient);
  comms_setTPRES(BREATHE_CONTROL_getPointInhalePressure());
  
  // read switches
  int END_SWITCH_VALUE_STOP = read_endswitch_stop();
  int END_SWITCH_VALUE_START = read_endswitch_start(); 
   
  // Check power supply and minimal degraded mode
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  min_degraded_mode_ON = checkDegradedMode(isFlow2PatientRead, isPatientPressureCorrect, isAmbientPressureCorrect);
  
  // check alarm
//  checkALARM(CurrentPressurePatient, CurrentVolumePatient, controller_state, isPatientPressureCorrect,
//             isFlow2PatientRead, fan_OK, battery_powered, battery_SoC, isAmbientPressureCorrect, temperature_OK);
    
  // State machine
  switch (controller_state) {
    case ini:{
      FLOW_SENSOR_hardresetVolume();
      // Check user input to start controller
      if (comms_getActive() == 1) {
        comms_resetActive(); // reset state to 0 (avoid buzzer resetting)
        SpeakerBeep(500); // turn on BUZZER   
      }
      if (comms_getActive() == 2) {
        transientMute = transientMuteCycles;
        controller_state = wait; // start controller
      }
    }break;
    case inhale:{ 
      // Reset volume to zero when flow detected
      FLOW_SENSOR_resetVolume_flowtriggered();
      // Call PID for inhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime, min_degraded_mode_ON);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // update motor speed
      Speed = BREATHE_CONTROL_Regulate_With_Volume(END_SWITCH_VALUE_STOP,min_degraded_mode_ON);
      MOTOR_CONTROL_setValue(Speed);
      // check if we need to change state based on time or endswitch
//      controller_state = BREATHE_setToEXHALE();  

      //safety  pressure & flow sensor check
      if (maxpressure<CurrentPressurePatient)
      { maxpressure=CurrentPressurePatient;}
      if (minpressure>CurrentPressurePatient)
      { minpressure=CurrentPressurePatient;}
      if (maxflow<CurrentFlowPatient)
      { maxflow=CurrentFlowPatient;}
      if (minflow>CurrentFlowPatient)
      { minflow=CurrentFlowPatient;}
      
    }break;
    case exhale: {
      // Call PID for exhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime, min_degraded_mode_ON);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // Motor to start position
      Speed = BREATHE_CONTROL_Regulate_With_Volume(END_SWITCH_VALUE_START,min_degraded_mode_ON); 
      MOTOR_CONTROL_setValue(Speed);
      // check if motor has returned
      //inhale_detected = BREATHE_CONTROL_CheckInhale();
      controller_state = BREATHE_setToWAIT(END_SWITCH_VALUE_START);
      // Check alarm ==> setAlarm() in PID!
    }break;
    case wait: {
      // Reset trigger for flow detection
      FLOW_SENSOR_resetVolume();
      // Call PID for wait
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime, min_degraded_mode_ON);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // Stop motor
      Speed = BREATHE_CONTROL_Regulate(min_degraded_mode_ON); 
      MOTOR_CONTROL_setValue(Speed);
      // check if we need to inhale
      inhale_detected = BREATHE_CONTROL_CheckInhale();
      controller_state = BREATHE_setToINHALE(inhale_detected);
      // reset timers for inhale
      if (controller_state == inhale){
        // update timers
        comms_setBPM(millis() - inhale_start_time);
        inhale_start_time = millis();

        // check mute setting
        if(transientMute > 0) transientMute--;
        
        // load new setting values from input
        target_inhale_time = comms_getInhaleTime();
        target_exhale_time = comms_getExhaleTime();
        target_volume = comms_getVT();
        trigger_mode = comms_getMode();
        target_risetime = comms_getRP();
        target_pressure = comms_getPressure(inhale_detected);

        // reset pressure and volume sensor check values
        maxpressureinhale=maxpressure;   // initialize low negative--> no error on start-up
        minpressureinhale=minpressure;    // initialize high positive --> no error on start-up
        maxpressure=-99999.9;   // initialize low negative--> no error on start-up
        minpressure=99999.9;    // initialize high positive --> no error on start-up
                
        maxflowinhale=maxflow;   // initialize low negative--> no error on start-up
        minflowinhale=minflow;    // initialize high positive --> no error on start-up
        maxflow=-99999.9;   // initialize low negative--> no error on start-up
        minflow=99999.9;    // initialize high positive --> no error on start-up
      }
      // Check user input to stop controller
      if (comms_getActive() == 0) { 
        controller_state = ini; // stop controller
        SpeakerOff(); // turn off BUZZER
      }
    }break;
    default: controller_state = wait;
  }
      
//  ALARM_debounceAlarm(); // take current alarms into account for debounce
  CPU_TIMER_stop(millis());
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  

void REQUEST_INPUT(){
  DEBUGserial.println("\n PRESS ENTER TO CONTINUE");
  serialFlush(); while(!Serial.available() ){} 
}

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
  
  //-- set up hall sensor
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("2) HALL SENSOR");
  if (HALL_SENSOR_INIT()) {
    hall_sens_init_ok = true;
    HALL_SENSOR_calibrateHall();
    DEBUGserial.println("  - HALL SENSOR OK");
  }
  else {
    DEBUGserial.println("  - HALL SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }
  
  //--- set up flow sensor
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("3) FLOW SENSOR");
  if (FLOW_SENSOR_INIT()) {
    flow_sens_init_ok = true;
    FLOW_SENSOR_setDeltaT(controllerTime);
    DEBUGserial.println("  - FLOW SENSOR OK");
    DEBUGserial.println("  - MEASURING...");
    while(millis() - starttime < 2000){
      FLOW_SENSOR_Measure(&CurrentFlowPatient,maxflowinhale,minflowinhale);
      DEBUGserial.print("     - ");
      DEBUGserial.println(CurrentFlowPatient);
      delay(100);
    }
  }
  else {
    DEBUGserial.println("  - FLOW SENSOR FAILED");
    if(HARDWARE)ALARM_init();
  }

  //-- set up pressure sensors
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("4) PRESSURE SENSOR");
  if (PRESSURE_SENSOR_INIT()){
    pressure_sens_init_ok = true;
    DEBUGserial.println("  - PRESSURE SENSORS OK");
    DEBUGserial.println("  - MEASURING TUBE SENSOR...");
    while(millis() - starttime < 2000){
      BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
      DEBUGserial.print("     - ");
      DEBUGserial.println(CurrentPressurePatient);
      delay(100);
    }
    DEBUGserial.println("  - MEASURING AMBIENT SENSOR...");
    while(millis() - starttime < 2000){
      float ambientpressure = BME280_readPressureAmbient();
      DEBUGserial.print("     - ");
      DEBUGserial.println(ambientpressure);
      delay(100);
    }
  }
  else{
    DEBUGserial.println("  - PRESSURE SENSORS FAILED");
    if(HARDWARE)ALARM_init();
  }

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

  //-- check alarms
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("6) TEMPERATURE");
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  if(temperature_OK){
    DEBUGserial.println("  - TEMPERATURE OK");
    DEBUGserial.println("  - MEASURING...");
    while(millis() - starttime < 2000){
      DEBUGserial.print("     - ");
      DEBUGserial.println(BME_280_GET_TEMPERATURE());
      delay(100);
    }
  }
  else{
    DEBUGserial.println("  - TEMPERATURE FAILED");
    if(HARDWARE)ALARM_init();
  }

  //-- check FAN
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("7) FAN");
  // fan off!
  DEBUGserial.println("  - MEASURING SPEED 0...");
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

  //-- check LIGHT & BUZZER
  delay(1000);
  DEBUGserial.println("");
  DEBUGserial.println("8) LIGHT & BUZZER");
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
  checkALARM_init(pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, hall_sens_init_ok, 
                  fan_OK, battery_powered, battery_SoC, temperature_OK);
    
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
  SpeakerTimingSupportRoutine();
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
  isFlow2PatientRead = FLOW_SENSOR_Measure(&CurrentFlowPatient,maxflowinhale,minflowinhale);
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
  checkALARM(CurrentPressurePatient, CurrentVolumePatient, controller_state, isPatientPressureCorrect,
             isFlow2PatientRead, fan_OK, battery_powered, battery_SoC, isAmbientPressureCorrect, temperature_OK);
    
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
      controller_state = BREATHE_setToEXHALE();  

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
      
  ALARM_debounceAlarm(); // take current alarms into account for debounce
  CPU_TIMER_stop(millis());
}

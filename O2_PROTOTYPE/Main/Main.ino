#include "TimerThree.h"
#include "PINOUT.h"
#include <avr/wdt.h>
// for debuggin purposes: allows to turn off features
#define PYTHON 0
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

typedef enum {ini = 0x00, wait = 0x01, inhale = 0x02, exhale = 0x03} controller_state_t;
controller_state_t controller_state = 0x00;

volatile unsigned long exhale_start_time = millis();
volatile unsigned long inhale_start_time = millis();
volatile unsigned long time_diff = 1;
float Speed; 
bool inhale_detected = 0;
int transientMute = 0;
int transientMuteCycles = 5;

// oxygen variables
float fio2 = 0.4; //TODO put in comms
int peakoffset = 100; // offset to compensate for peak flow, which the slow sensor can't detect!

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
bool isFlowOfOxygenRead = false;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  DEBUGserial.begin(115200); 
  
  Serial.print("FW version: ");
  Serial.println(comms_getFW());
  DEBUGserial.print("FW version: ");
  DEBUGserial.println(comms_getFW());

  //-- set up timer3
  Timer3.initialize(controllerTime);   // initialize timer3 in us, set 10 ms timing

  //-- set up peripherals
  initPeripherals();
  FanOff(); // TODO 
  
  //--- check mains supply and battery voltage
  main_supply = MainSupplyVoltage()/1000;
  batt_supply = PSUSupplyVoltage()/1000;
  battery_SoC = batt_supply/25;
  
  //-- set up hall sensor
  DEBUGserial.println("Setting up HALL sensor: ");
  if (HALL_SENSOR_INIT()) {
    hall_sens_init_ok = true;
    HALL_SENSOR_calibrateHall();
    DEBUGserial.println("HALL SENSOR OK");
  }
  else {
    DEBUGserial.println("HALL SENSOR Failed");
    if(HARDWARE)ALARM_init();
  }
  
  //--- set up flow sensor
  DEBUGserial.println("Setting up flow sensor: ");
  if (FLOW_SENSOR_INIT()) {
    flow_sens_init_ok = true;
    FLOW_SENSOR_setDeltaT(controllerTime);
    DEBUGserial.println("FLOW SENSOR OK");
  }
  else {
    DEBUGserial.println("FLOW SENSOR Failed");
    if(HARDWARE)ALARM_init();
  }

  //-- set up pressure sensors
  DEBUGserial.println("Setting up PRESSURE sensors: ");
  if (PRESSURE_SENSOR_INIT()){
    pressure_sens_init_ok = true;
    DEBUGserial.println("PRESSURE SENSORS OK");
  }
  else{
    DEBUGserial.println("PRESSURE SENSORS Failed");
    if(HARDWARE)ALARM_init();
  }

  //-- set up motor
  DEBUGserial.println("Setting up MOTOR: ");
  if (MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN)) {
    motor_sens_init_ok = true;
    DEBUGserial.println("MOTOR OK");
  }
  else {
    DEBUGserial.println("MOTOR Failed");
    if(HARDWARE)ALARM_init();
  }

  delay(500);

  //-- set up oxygen
  DEBUGserial.println("Setting up Oxygen supply: ");
  
  ValveOn();
  unsigned long valvestarttime = millis();
  unsigned long valveinittime = 600;
  int mincalibrationvolume = 100;
  int counter = 0;
  while(millis() - valvestarttime < valveinittime){
    FLOW_SENSOR_MeasureO2(&CurrentFlowOxygen);
    FLOW_SENSOR_updateVolumeO2init(CurrentFlowOxygen);
    counter++;
  }
  ValveOff();
  
  // calculate K_O2
  float sampletime = valveinittime / counter;
  float calibrationvolume = peakoffset + FLOW_SENSOR_getTotalVolumeIntO2() * sampletime;
  float ratio = valveinittime/calibrationvolume;
  FLOW_SENSOR_setK_O2(ratio);
  FLOW_SENSOR_resetVolumeO2();

  if (calibrationvolume > mincalibrationvolume) {
    // TODO: add alarm 
    DEBUGserial.println("OXYGEN SUPPLY OK");
  }
  else {
    DEBUGserial.println("OXYGEN SUPPLY Failed");
    if(HARDWARE)ALARM_init();
  }

  // TODO: remove debug prints
  DEBUGserial.print("volume: ");
  DEBUGserial.println(calibrationvolume);
  
  // empty oxygen bag
  MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);

  //-- setup done
  DEBUGserial.println("Setup done");

  //-- check alarms
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  checkALARM_init(pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, hall_sens_init_ok, 
                  fan_OK, battery_powered, battery_SoC, temperature_OK);
    
  //-- set up communication with screen
  if(PYTHON) initCOMM();
  if (!PYTHON) isPythonOK = true;

  //-- set up interrupt and watchdog if no alarms during initialisation
  if(!ALARM_getAlarmState()){
    configure_wdt();                     // configures watchdog timer to 16 ms
    Timer3.attachInterrupt(controller);  // attaches callback() as a timer overflow interrupt
  }
}

//---------------------------------------------------------------
// LOOP
//---------------------------------------------------------------

void loop()
{
  // Handle uart send to PC
  if (PYTHON && isPythonOK) sendDataToPython();
  // Handle uart receive from PC
  recvWithEndMarkerSer0();
  // Check alarm and watchdog
  if (PYTHON) doWatchdog();
  if (PYTHON && isPythonOK) doCPU_TIMER();
  // Handle uart receive for debugging
  if (!PYTHON) recvWithEndMarkerSer1();

  // update ambient pressure and temperature
  isAmbientPressureCorrect = BME_280_UPDATE_AMBIENT();
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  // check buzzer
  SpeakerTimingSupportRoutine();
  // delay loop to avoid full serial buffers
  unsigned long waitstarttime = millis();
  while(millis() - waitstarttime < 50){
    // check fan at high sample rate
    fan_OK = FanPollingRoutine();
  }
}

// ---------------------------------------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------------------------------------

void controller()
{
  CPU_TIMER_start(millis());
  
  // Enable interrupts, because this ISR takes approx 5ms and we need millis() to update
  interrupts(); 
  // readout sensors
  isFlow2PatientRead = FLOW_SENSOR_MeasurePatient(&CurrentFlowPatient,maxflowinhale,minflowinhale);
  isFlowOfOxygenRead = FLOW_SENSOR_MeasureO2(&CurrentFlowOxygen);
  isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
  //isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  
  // update volume 
  FLOW_SENSOR_updateVolume(CurrentFlowPatient);
  FLOW_SENSOR_getVolume(&CurrentVolumePatient);
  FLOW_SENSOR_updateVolumeO2(CurrentFlowOxygen);
  FLOW_SENSOR_getVolumeO2(&CurrentVolumeOxygen);

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
      ValveCheck();
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
      ValveOff();
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
      ValveOff();
      // Reset trigger for flow detection
      FLOW_SENSOR_resetVolume();
      // Call PID for wait
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime, min_degraded_mode_ON);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // Stop motor
      Speed = BREATHE_CONTROL_Regulate_With_Volume(END_SWITCH_VALUE_START,min_degraded_mode_ON);
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

        // oxygen control
        ValveOn(FLOW_SENSOR_getTime(fio2));
        FLOW_SENSOR_updateK_O2();
        FLOW_SENSOR_resetVolumeO2();

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
      
  ALARM_processAlarm(); 
  CPU_TIMER_stop(millis());
  noInterrupts();
  wdt_reset();
}

// ---------------------------------------------------------------------------------------------------------
// WATCHDOG
// ---------------------------------------------------------------------------------------------------------

void configure_wdt(void){
  cli();                           // disable interrupts for changing the registers
  MCUSR = 0;                       // reset status register flags
                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b000000; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds
  sei();                           // re-enable interrupts
}

ISR(WDT_vect){
  DEBUGserial.println("WDT");
  // Disable motor, enable speaker
  SpeakerOn();
  digitalWrite(Motor_PWM_PIN, LOW);
  // Only short delay microseconds work inside ISR
  while(1){
    for(int i=0; i<10000; i++){
      delayMicroseconds(100);  
    }
    digitalWrite(Light_PWM, HIGH);
    for(int i=0; i<10000; i++){
      delayMicroseconds(100);  
    }
    digitalWrite(Light_PWM, LOW); 
  }
}

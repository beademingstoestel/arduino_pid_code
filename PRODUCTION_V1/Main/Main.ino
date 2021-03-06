#include "TimerThree.h"
#include "PINOUT.h"
#include <avr/wdt.h>
// for debuggin purposes: allows to turn off features
#define PYTHON 1

//This is 3.83 with modifications on the oxygen control by Lieven

/*
 * 27/08/2020 -Lieven
 * changes in Flowsensor_oxygencontrol
The controller now uses an approximate vlaue for valvetime, for 10 cycles.
 * This value is taken from float valvetime_presetvalues[11], which is a list of valvetimes for vt=800, then corrected for the actual Vt
 * After 10 cycles the previous controller takes over.
 * 
 * FiO2 reported in de GUI has been changed to not be the average, but the last measured value - this makes it update faster and is more intuitive
 * The FIO2 used in de controller is still the running average
 * 
 * Also: when changing desired Fio2 the cumulative error and K_02 now get reset - this makes the controller repeatable
 * 
 * 
 * */
 
// set #define OXYGENCONTROL 1 in PINOUT.h for use with PYTHON 0
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
bool Active_1_beep_played = false;

volatile unsigned long exhale_start_time = millis();
volatile unsigned long inhale_start_time = millis();
volatile unsigned long time_diff = 1;
float Speed; 
bool inhale_detected = 0;
int transientMute = 0;
int transientMuteCycles = 5;
int numberofretries = 0;

float target_risetime = 500;           
unsigned int target_pressure = 20; 
unsigned int target_volume = 700;
unsigned int target_inhale_time = 0;
unsigned int target_exhale_time = 0;
unsigned int trigger_mode = 0;


//OXYGEN CONTROL - SENSORVERSION
//---------------------------------------------------------------
float valvetime = 0;
byte control_oxygen=0; // status to disable regulation if fio2 < fio2min
float previousvalvetime = 0;
float previous_I_concentration = 21;
float previous_target_fio2 = 0.2;
float valveControlIterations = 0;
float valvetime_presetvalues[11] = {0, 0, 55, 70, 120, 200, 280, 350, 430, 510, 600}; //stored preset values for valve time per 10% for a VT=800ml, firdst value is for 20%, 30% does nothing, 40% is 40ms...
//70 = 0.3


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
float maxflowoxygen=-99999.9;       // initialize low negative--> no error on start-up
float minflowoxygen=99999.9;        // initialize high positive --> no error on start-up
float maxflow=-99999.9;             // initialize low negative--> no error on start-up
float minflow=99999.9;              // initialize high positive --> no error on start-up
float maxflowO2=-99999.9;             // initialize low negative--> no error on start-up
float minflowO2=99999.9;              // initialize high positive --> no error on start-up

float battery_SoC = 1.0;   
float main_supply = 0.0;
float batt_supply = 0.0;

bool OXYGENCONTROL_PYTHON = 0;
   
// safety minimum degraded mode
bool min_degraded_mode_ON = false;
bool temperature_OK = false;
bool battery_above_25 = false;
bool fan_OK = true;   
bool battery_powered = false;
bool motor_sens_init_ok = false;
bool sensor_calibration_ok = false;
bool pressure_sens_init_ok = false;
bool flow_sens_init_ok = false;
bool isFlow2PatientRead = false;
bool isPatientPressureCorrect = false;
bool isAngleOK = false;
bool isPythonOK = false;
bool isAmbientPressureCorrect = false;
bool oxygen_init_ok = false;
bool isFlowOfOxygenRead = false;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  
  Serial.print("FW version: ");
  Serial.println(comms_getFW());

  //-- set up timer3
  Timer3.initialize(controllerTime);   // initialize timer3 in us, set 10 ms timing

  //-- set up peripherals
  initPeripherals();

  //--- check mains supply and battery voltage
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);

  //---------------------------------------------------------------
  // INIT PERIPHERALS
  //---------------------------------------------------------------

  //-- set up oxygen sensor
  DEBUGserialprintln("Setting up OXYGEN sensors: ");
  if (OXYGEN_SENSOR_INIT()) {
    oxygen_init_ok = true; 
    DEBUGserialprintln("OXYGEN SENSORS OK");
  }
  else {
    DEBUGserialprintln("OXYGEN SENSORS Failed");
  }
   
  //--- set up flow sensor
  DEBUGserialprintln("Setting up flow sensor: ");
  if (FLOW_SENSOR_INIT() == 3) {
    flow_sens_init_ok = true;
    DEBUGserialprintln("FLOW SENSOR OK");
  }
  else {
    DEBUGserialprintln("FLOW SENSOR Failed");
  }

  //-- set up pressure sensors
  DEBUGserialprintln("Setting up PRESSURE sensors: ");
  if (PRESSURE_SENSOR_INIT() == 3){
    pressure_sens_init_ok = true;
    DEBUGserialprintln("PRESSURE SENSORS OK");
  }
  else{
    DEBUGserialprintln("PRESSURE SENSORS Failed");
  }

  //-- set up motor
  DEBUGserialprintln("Setting up MOTOR: ");
    if (MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN)) {
    motor_sens_init_ok = true;
    DEBUGserialprintln("MOTOR OK");
  }
  if (MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN)) {
    motor_sens_init_ok = true;
    DEBUGserialprintln("MOTOR OK");
  }
  else {
    DEBUGserialprintln("MOTOR Failed");
  }
  
  //-- check alarms
  isAmbientPressureCorrect = BME_280_UPDATE_AMBIENT();
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  checkALARM_init(oxygen_init_ok, pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, 
                  fan_OK, battery_powered, battery_SoC, temperature_OK);

  //---------------------------------------------------------------
  // INIT COMMUNICATION
  //---------------------------------------------------------------  
                
  //-- set up communication with screen
  if(PYTHON) initCOMM();
  if (!PYTHON) isPythonOK = true;

  //-- send active state
  if(!ALARM_getAlarmState()){
    comms_setActive(-4);
    DEBUGserialprintln("INIT OK");
  }
  else{
    comms_setActive(-5);
    DEBUGserialprintln("INIT OK");
  }
 if (PYTHON) sendActiveState();

  //---------------------------------------------------------------
  // CALIBRATE
  //---------------------------------------------------------------

  //-- wait for calibration of flow and pressure sensors
  isFlow2PatientRead = false;
  isPatientPressureCorrect = false;
  DEBUGserialprintln("WAIT FOR CALIBRATION");
  while(!isPatientPressureCorrect || !isFlow2PatientRead){
    recvWithEndMarkerSer0();
    if (PYTHON) doWatchdog();
    
    if (comms_getActive() == -3 || !PYTHON) { 
      comms_resetActive();
      numberofretries = 5;
    }

    if (numberofretries > 0){
      numberofretries--; 
      isFlow2PatientRead = FLOW_SENSOR_CALIBRATE();
      isPatientPressureCorrect = PRESSURE_SENSOR_CALIBRATE();
      if(numberofretries == 0){
        comms_setActive(-4);
        sendActiveState();
        DEBUGserialprintln("CALIBRATIONS FAILED");
      }
    }
  }
  comms_setActive(-2);
  if (PYTHON) sendActiveState();
  DEBUGserialprintln("CALIBRATION OK");
  checkALARM_calib(isPatientPressureCorrect, isFlow2PatientRead, isFlowOfOxygenRead);

  //---------------------------------------------------------------
  // OXYGEN
  //---------------------------------------------------------------

  // wait for initialisation of oxygen supply
  isFlowOfOxygenRead = false;
  DEBUGserialprintln("WAIT FOR OXYGEN");
  while(!isFlowOfOxygenRead){
    recvWithEndMarkerSer0();
    if (PYTHON) doWatchdog();

    if (comms_getActive() == 0 || (!PYTHON && !OXYGENCONTROL)) { 
      OXYGENCONTROL_PYTHON = 0;
      isFlowOfOxygenRead = true;
    }
    else if (comms_getActive() == -1 || (!PYTHON && OXYGENCONTROL)) { 
      OXYGENCONTROL_PYTHON = 1;
      comms_resetActive(); 
      DEBUGserialprintln("Setting up Oxygen supply: ");

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
        isFlowOfOxygenRead = true; 
        DEBUGserialprintln("OXYGEN SUPPLY OK");
      }
      else {
        comms_setActive(-2);
        if (PYTHON) sendActiveState();
        DEBUGserialprintln("OXYGEN SUPPLY Failed");
      }
    
      // empty oxygen bag
      MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);
      MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);
    }
  }
  comms_resetActive();
  if (PYTHON) sendActiveState();
  DEBUGserialprintln("OXYGEN INIT OK");  
  checkALARM_calib(isPatientPressureCorrect, isFlow2PatientRead, isFlowOfOxygenRead);

  //---------------------------------------------------------------
  // START LOOP
  //---------------------------------------------------------------
  
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
  if (PYTHON) recvWithEndMarkerSer0();
  // Check alarm and watchdog
  if (PYTHON) doWatchdog();
  if (PYTHON && isPythonOK) doCPU_TIMER();
  // Handle uart receive for debugging
  if (!PYTHON) recvWithEndMarkerSer1();

  // update ambient pressure and temperature
  isAmbientPressureCorrect = BME_280_UPDATE_AMBIENT();
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  // check the motor to regulate PEEP
  PEEP_check_motor();
  // Measure oxygen each second
  OXYGEN_SENSOR_MEASURE();
  // delay loop to avoid full serial buffers
  delay(50);
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
  isFlowOfOxygenRead = FLOW_SENSOR_MeasureO2(&CurrentFlowOxygen,maxflowoxygen,minflowoxygen);
  fan_OK = FanPollingRoutine();
  ValveCheck();
  isFlow2PatientRead = FLOW_SENSOR_MeasurePatient(&CurrentFlowPatient,maxflowinhale,minflowinhale);
  fan_OK = FanPollingRoutine();
  ValveCheck();
  isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
  fan_OK = FanPollingRoutine();
  ValveCheck();
  isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  fan_OK = FanPollingRoutine();
  ValveCheck();
  
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
  checkALARM(FLOW_SENSOR_getFIO2(), isFlowOfOxygenRead, CurrentPressurePatient, CurrentVolumePatient, controller_state, isPatientPressureCorrect,
             isFlow2PatientRead, fan_OK, battery_powered, battery_SoC, isAmbientPressureCorrect, temperature_OK);
    
  // State machine
  switch (controller_state) {
    case ini:{
      FLOW_SENSOR_hardresetVolume();
      // Check user input to start controller
      if (comms_getActive() == 1) {
        comms_resetActive(); // reset state to 0 (avoid buzzer resetting)
        SpeakerBeep(500); // turn on BUZZER 
        Active_1_beep_played = true;  
      }
      // Only accept when Active=1 has been received first 
      if(comms_getActive() == 2 && !Active_1_beep_played) {
        comms_resetActive();
        reset_sendActiveState();
      }
      if (comms_getActive() == 2 && Active_1_beep_played) {
        Active_1_beep_played = false;
        comms_setActive(3);
        reset_sendActiveState();
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
      controller_state = BREATHE_setToEXHALE(target_pressure, min_degraded_mode_ON);  

      //safety  pressure & flow sensor check
      if (maxpressure<CurrentPressurePatient)
      { maxpressure=CurrentPressurePatient;}
      if (minpressure>CurrentPressurePatient)
      { minpressure=CurrentPressurePatient;}
      if (maxflow<CurrentFlowPatient)
      { maxflow=CurrentFlowPatient;}
      if (minflow>CurrentFlowPatient)
      { minflow=CurrentFlowPatient;}
      if (maxflowO2<CurrentFlowOxygen)
      { maxflowO2=CurrentFlowOxygen;}
      if (minflowO2>CurrentFlowOxygen)
      { minflowO2=CurrentFlowOxygen;}
      
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
        target_risetime = updateAutoFlow(target_risetime, target_inhale_time);
        target_pressure = comms_getPressure(inhale_detected);
        target_fio2 = comms_getFIO2(); //this reads 

        // oxygen control   
        comms_setFIO2(FLOW_SENSOR_getFIO2());
        FLOW_SENSOR_updateK_O2();
        FLOW_SENSOR_resetVolumeO2();
        ValveOn(FLOW_SENSOR_getTime(target_fio2));

        // adjust PEEP valve
        PEEP_update();

        // reset pressure and volume sensor check values
        maxpressureinhale=maxpressure;   // initialize low negative--> no error on start-up
        minpressureinhale=minpressure;    // initialize high positive --> no error on start-up
        maxpressure=-99999.9;   // initialize low negative--> no error on start-up
        minpressure=99999.9;    // initialize high positive --> no error on start-up
                
        maxflowinhale=maxflow;   // initialize low negative--> no error on start-up
        minflowinhale=minflow;    // initialize high positive --> no error on start-up
        maxflowoxygen=maxflowO2;   // initialize low negative--> no error on start-up
        minflowoxygen=minflowO2;    // initialize high positive --> no error on start-up
        maxflow=-99999.9;   // initialize low negative--> no error on start-up
        minflow=99999.9;    // initialize high positive --> no error on start-up
        maxflowO2=-99999.9;   // initialize low negative--> no error on start-up
        minflowO2=99999.9;    // initialize high positive --> no error on start-up
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
  doBeepingAlarm();
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
  DEBUGserialprintln("WDT");
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

// ---------------------------------------------------------------------------------------------------------
// DEBUGGING
// ---------------------------------------------------------------------------------------------------------

void DEBUGserialprintln(String text){
  if (!PYTHON){
    Serial.println(text);
  }
}

void DEBUGserialprintln(float text){
  if (!PYTHON){
    Serial.println(text);
  }
}

void DEBUGserialprint(String text){
  if (!PYTHON){
    Serial.print(text);
  }
}

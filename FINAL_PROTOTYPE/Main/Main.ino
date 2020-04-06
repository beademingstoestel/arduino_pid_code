#include "TimerThree.h"
#include "PINOUT.h"
// for debuggin purposes: allows to turn off features
#define PYTHON 1
#define HARDWARE 1
#define DEBUGserial Serial3

//#define hall_sensor_i2c  // comment to use SPI
#define BME_tube 1
#define BME_ambient 0
#define MPL_tube 1
 
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
//bool min_degraded_mode_ON = false;
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
bool isVolumeOK = false;
bool isPythonOK = false;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  DEBUGserial.begin(115200); 

  Serial.println(comms_getFW());
  DEBUGserial.println(comms_getFW());

  //-- set up peripherals
  initPeripherals();

  //-- set up communication with screen
  if(PYTHON) initCOMM();
  if (!PYTHON) isPythonOK = true;

  //--- check mains supply and battery voltage
  DEBUGserial.print("Supply Voltage (V): ");
  main_supply = MainSupplyVoltage()/1000;
  DEBUGserial.println(main_supply);
  DEBUGserial.print("Battery Voltage (V): ");
  batt_supply = PSUSupplyVoltage()/1000;
  DEBUGserial.println(batt_supply);
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
  
  //--- set up flow sensors here, if init fails, we can continue
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

  //-- setup done
  DEBUGserial.println("Setup done");

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
  if (PYTHON) sendAlarmState();
  // Handle uart receive from PC
  recvWithEndMarkerSer0();
  // Check alarm and watchdog
  if (PYTHON) doWatchdog();
  if (PYTHON) doCPU_TIMER();
  // Handle uart receive for debugging
  if (!PYTHON) recvWithEndMarkerSer1();

  // update speaker
  SpeakerTimingSupportRoutine();
  // check fan
  FanPollingRoutine();
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
  isVolumeOK = FLOW_SENSOR_getVolume(&CurrentVolumePatient);
  noInterrupts();
  
  // Check power supply and minimal degraded mode
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  min_degraded_mode_ON = checkDegradedMode(isFlow2PatientRead, isPatientPressureCorrect, battery_above_25);
  
  // update values 
  FLOW_SENSOR_updateVolume(CurrentFlowPatient);
  comms_setFLOW(CurrentFlowPatient);
  comms_setVOL(CurrentVolumePatient);
  comms_setPRES(CurrentPressurePatient);
  comms_setTPRES(BREATHE_CONTROL_getPointInhalePressure());
  
  // read switches
  int END_SWITCH_VALUE_STOP = digitalRead(ENDSWITCH_FULL_PIN);
  int END_SWITCH_VALUE_START = digitalRead(ENDSWITCH_PUSH_PIN);  

  // check alarm
  checkALARM(CurrentPressurePatient, CurrentVolumePatient,  isPatientPressureCorrect, 
    isFlow2PatientRead, pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, hall_sens_init_ok, 
    fan_OK, battery_powered, battery_SoC);
    
  // State machine
  switch (controller_state) {
    case ini:{
      FLOW_SENSOR_hardresetVolume();
      // Check user input to start controller
      if (comms_getActive() == 1) {
        comms_resetActive(); // reset state to 0 (avoid buzzer resetting)
        ALARM_Short_Beep(); // turn on BUZZER   
      }
      if (comms_getActive() == 2) {
        controller_state = wait; // start controller
      }
    }break;
    case inhale:{ 
      // Reset volume to zero when flow detected
      FLOW_SENSOR_resetVolume_flowtriggered();
      // Call PID for inhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
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
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
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
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
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
      }
    }break;
    default: controller_state = wait;
  }
      
  debounceAlarm(); // take current alarms into account for debounce
  CPU_TIMER_stop(millis());
}

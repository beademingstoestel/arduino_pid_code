#include "TimerThree.h"
#include "PINOUT.h"
// for debuggin purposes: allows to turn off features
#define PYTHON 0
#define HARDWARE 1
#define DEBUGserial Serial3

//#define hall_sensor_i2c  // comment to use SPI
#define BME_tube 1
#define BME_ambient 0
#define MPL_tube 0
 
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
bool isPythonOK = false;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  DEBUGserial.begin(115200);

  //-- set up peripherals
  initPeripherals();

  //-- set up motor
  Serial.println("Setting up MOTOR: ");
  if (MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN)) {
    Serial.println("MOTOR OK");
  }
  else {
    Serial.println("MOTOR Failed");
    if(HARDWARE)ALARM_init();
  }

  //-- set up hall sensor
  Serial.println("Setting up HALL sensor: ");
  if (HALL_SENSOR_INIT()) {
    HALL_SENSOR_calibrateHall();
    Serial.println("HALL SENSOR OK");
  }
  else {
    Serial.println("HALL SENSOR Failed");
    if(HARDWARE)ALARM_init();
  }
  
  //-- set up pressure sensors
  Serial.println("Setting up PRESSURE sensors: ");
  if (PRESSURE_SENSOR_INIT()){
    Serial.println("PRESSURE SENSORS OK");
  }
  else{
    Serial.println("PRESSURE SENSORS Failed");
    if(HARDWARE)ALARM_init();
  }

  //--- set up flow sensors here, if init fails, we can continue
  Serial.print("Setting up flow sensor: ");
  if (FLOW_SENSOR_INIT()) {
    FLOW_SENSOR_setDeltaT(controllerTime);
    Serial.println("FLOW SENSOR OK");
  }
  else {
    Serial.println("FLOW SENSOR Failed");
    if(HARDWARE)ALARM_init();
  }

  //-- setup done
  Serial.println("Setup done");
  MOTOR_CONTROL_setup(ENDSWITCH_PUSH_PIN, ENDSWITCH_FULL_PIN);

  //-- set up communication with screen
  if(PYTHON){
    initCOMM();
  }
  else{
    isPythonOK = true;
  }

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

  // update speaker
  SpeakerTimingSupportRoutine();
  // check fan
  FanPollingRoutine();
  delay(40); 
}

// ---------------------------------------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------------------------------------

void controller()
{
  CPU_TIMER_start(millis());
  // readout sensors
  interrupts();
  bool isCurrentFlowPatientRead = FLOW_SENSOR_Measure(&CurrentFlowPatient);
  bool isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient);
  bool isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  bool isVolumeOK = FLOW_SENSOR_getVolume(&CurrentVolumePatient);
  noInterrupts();
  // update values 
  FLOW_SENSOR_updateVolume(CurrentFlowPatient);
  comms_setFLOW(CurrentFlowPatient);
  comms_setVOL(CurrentVolumePatient);
  comms_setPRES(CurrentPressurePatient);
  comms_setTPRES(BREATHE_CONTROL_getPointInhalePressure());
  // read switches
  int END_SWITCH_VALUE_STOP = digitalRead(ENDSWITCH_FULL_PIN);
  int END_SWITCH_VALUE_START = digitalRead(ENDSWITCH_PUSH_PIN);

  // State machine
  switch (controller_state) {
    case ini:{
      FLOW_SENSOR_hardresetVolume();
      // Check user input to start controller
      if (comms_getActive() == 1) {
        SpeakerOn(); // turn on BUZZER
      }
      if (comms_getActive() == 2) {
        controller_state = wait; // start controller
        SpeakerOff(); // turn off BUZZER
      }
    }break;
    case inhale:{ 
      // Reset volume to zero when flow detected
      FLOW_SENSOR_resetVolume_flowtriggered();
      // Call PID for inhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // update motor speed
      Speed = BREATHE_CONTROL_Regulate_With_Volume(END_SWITCH_VALUE_STOP);
      MOTOR_CONTROL_setValue(Speed);
      // check if we need to change state based on time or endswitch
      controller_state = BREATHE_setToEXHALE();      
    }break;
    case exhale: {
      // Call PID for exhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // Motor to start position
      Speed = BREATHE_CONTROL_Regulate_With_Volume(END_SWITCH_VALUE_START); 
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
      Speed = BREATHE_CONTROL_Regulate(); 
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
      }
      // Check user input to stop controller
      if (comms_getActive() == 0) { 
        controller_state = ini; // stop controller
      }
    }break;
    default: controller_state = wait;
  }
  CPU_TIMER_stop(millis());
}


#include "TimerThree.h"
#include "PINOUT.h"
// for debuggin purposes: allows to turn off features
#define PYTHON 0
#define HARDWARE 0
#define BUTTONS 0
 
//---------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------

unsigned long controllerTime = 10000; // us 
unsigned long Watchdog = 3000; // 3 seconds watchdog

unsigned long lastWatchdogTime = millis();

volatile float CurrentPressurePatient = 0;
volatile float Flow2Patient = 0;
volatile float Volume2Patient = 0;

typedef enum {ini = 0x00, wait = 0x01, inhale = 0x02, exhale = 0x03} controller_state_t;
controller_state_t controller_state = 0x00;

volatile unsigned long exhale_start_time = millis();
volatile unsigned long start_time_pressure;
volatile unsigned long inhale_start_time;
volatile unsigned long time_diff = 1;
float Speed; 

float target_risetime = 500;            // init at low time, set in loop below
unsigned int target_pressure = 20; 
unsigned int target_volume = 700;
unsigned int target_inhale_time = 0;
unsigned int target_exhale_time = 0;
unsigned int trigger_mode = 0;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  //--- set up flow sensors here, if init fails, we can continue
  Serial.print("Setting up flow sensor: ");
  if (FLOW_SENSOR_INIT()) {
    Serial.println("FLOW SENSOR OK");
  }
  else {
    Serial.println("FLOW SENSOR Failed");
    if(HARDWARE)while(1){};
  }
  setDeltaT(controllerTime);
  
  //-- set up BME
  Serial.println("Setting up BME sensor: ");
  if (BME280_Setup()){
    Serial.println("BME OK");
  }
  else{
    Serial.println("BME Failed");
    if(HARDWARE)while(1){};
  }

  //-- set up hall sensor
  Serial.println("Setting up HALL sensor: ");
  if (HALL_SENSOR_INIT()) {
    Serial.println("HALL SENSOR OK");
  }
  else {
    Serial.println("HALL SENSOR Failed");
    if(HARDWARE)while(1){};
  }

  //-- set up limit switches
  pinMode(ENDSIWTCH_FULL_PIN,INPUT_PULLUP);
  pinMode(ENDSWITCH_PUSH_PIN,INPUT_PULLUP);

  //-- set up motor
  MOTOR_CONTROL_setp();

  //-- set up communication with screen
  if(PYTHON){
    initCOMM();
  }

  //-- set up interrupt
  Timer3.initialize(controllerTime);   // initialize timer3 in us, set 100 ms timing
  Timer3.attachInterrupt(controller);  // attaches callback() as a timer overflow interrupt

  //-- setup done
  Serial.println("Setup done");
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
  if (getAlarmState() != 0) {
    // SOUND BUZZER
    // COMMUNICATE TO SCREEN
  }

  // Handle uart receive from display module
  recvWithEndMarkerSer1();

  delay(20); 
}

// ---------------------------------------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------------------------------------

void controller()
{
  // readout sensors
  interrupts();
  bool isFlow2PatientRead = FLOW_SENSOR_Measure(&Flow2Patient);
  bool isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient);
  bool isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  noInterrupts();
  // update values 
  updateVolume(Flow2Patient*0.1);
  comms_setVOL(Volume2Patient);
  comms_setVOL(getTotalVolumeInt());
  comms_setPRES(CurrentPressurePatient);
  // read switches
  int END_SWITCH_VALUE_STOP = digitalRead(ENDSIWTCH_FULL_PIN); //inhale
  int END_SWITCH_VALUE_START = digitalRead(ENDSWITCH_PUSH_PIN);

  // State machine
  switch (controller_state) {
    case ini:{
      // Check user input to start controller
      if (comms_getActive() == true) {
        controller_state = wait; // start controller
      }
    }break;
    case inhale:{ 
      // Call PID for inhale
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // update motor speed
      Speed = BREATHE_CONTROL_Regulate_With_Volume(); 
      MOTOR_CONTROL_setValue(Speed);
      // check if we need to change state based on time or endswitch
      controller_state = BREATHE_setToEXHALE(END_SWITCH_VALUE_STOP);      
    }break;
    case exhale: {
      // Motor to start position
      Speed = BREATHE_CONTROL_Regulate(); 
      MOTOR_CONTROL_setValue(Speed);
      // check if motor has returned
      controller_state = BREATHE_setToWAIT(END_SWITCH_VALUE_START);
      // Check alarm ==> setAlarm() in PID!
      if (getAlarmState() != 0) {
        // SOUND BUZZER
        // COMMUNICATE TO SCREEN
      }
    }break;
    case wait: {
      // Stop motor
      Speed = BREATHE_CONTROL_Regulate(); 
      MOTOR_CONTROL_setValue(Speed);
      // check if we need to inhale
      controller_state = BREATHE_setToINHALE();
      // reset timers for inhale
      if (controller_state == inhale){
        comms_setBPM(millis() - inhale_start_time);

        // update timers and variables
        inhale_start_time = millis();
        start_time_pressure = millis();
        resetVolume();

        // load new setting values from input
        target_inhale_time = comms_getInhaleTime();
        target_exhale_time = comms_getExhaleTime();
        target_pressure = comms_getPK();
        target_volume = comms_getVT();
        trigger_mode = comms_getMode();
      }
      // Check user input to stop controller
      if (comms_getActive() == false) { 
        controller_state = ini; // stop controller
      }
    }break;
    default: controller_state = wait;
  }
}

// ---------------------------------------------------------------------------------------------------------

void doWatchdog(void) {
  if (millis() - lastWatchdogTime > Watchdog) {
    setAlarmState(5);
    digitalWrite(13, HIGH);
  }
  Serial.print("ALARM=");
  Serial.println(getAlarmState());
}

void doWatchdogIO(void){
//  Serial1.println("A");
}

#include "TimerThree.h"

// for debuggin purposes: allows to turn off features
#define PYTHON 1
#define HARDWARE 0
#define BUTTONS 1

#define ENDSIWTCH_FULL_PIN 3 //inhale
#define ENDSWITCH_PUSH_PIN 2

//---------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------

unsigned long controllerTime = 10000; // us
unsigned long Watchdog = 3000; // 3 seconds watchdog

unsigned long lastWatchdogTime = millis();

volatile float CurrentPressurePatient = 0;
volatile float Flow2Patient = 0;
volatile unsigned int angle = 0;

typedef enum {ini = 0x00, wait = 0x01, inhale = 0x02, exhale = 0x03} controller_state_t;
controller_state_t controller_state = 0x00;

volatile unsigned long exhale_start_time = millis();
volatile unsigned long start_time_pressure;
volatile unsigned long inhale_start_time;
volatile bool time_pressure_reached=0;
volatile unsigned long time_diff = 1;
float Speed; 

float target_risetime = 500;            // init at low time, set in loop below
unsigned int target_pressure = 0; 
unsigned int target_volume = 0;
unsigned int target_inhale_time = 0;
unsigned int target_exhale_time = 0;

//---------------------------------------------------------------
// SETUP
//---------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  //-- set up motor
  MOTOR_CONTROL_setp();

  //-- set up limit switches
  pinMode(ENDSIWTCH_FULL_PIN,INPUT_PULLUP);
  pinMode(ENDSWITCH_PUSH_PIN,INPUT_PULLUP);

  // move motor up to calibrate sensors
  while(!digitalRead(ENDSWITCH_PUSH_PIN)){
    MOTOR_CONTROL_setValue(150);
  }
  MOTOR_CONTROL_setValue(0);

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
  while (!BME280_Setup()) // must start, if not, do not continue
  {
    delay(100);
  }
  Serial.println("BME OK");

  //-- set up hall sensor
  Serial.println("Setting up HALL sensor: ");
  if (HALL_SENSOR_INIT()) {
    Serial.println("HALL SENSOR OK");
  }
  else {
    Serial.println("HALL SENSOR Failed");
    if(HARDWARE)while(1){};
  }
  
    //-- set up communication with screen
    pinMode(5, OUTPUT);
    delay(200);
  pinMode(5, INPUT);
  if(BUTTONS) initCOMM();

//  while(1){
//    Serial.println("test");
//    MOTOR_CONTROL_setValue(-40);
//delay(2000);
//    //MOTOR_CONTROL_setValue(40);
//delay(2000);
//  }

  //-- set up interrupt
  pinMode(13, OUTPUT);
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

  delay(50);

//  Serial.println(angle);
//  Serial.println(Flow2Patient);
//  Serial.println(-(getTotalVolumeInt()));
  // Serial.println(CurrentPressurePatient);
//   Serial.println(comms_getActive());

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
  bool isAngleOK = HALL_SENSOR_readHall(&angle);
  noInterrupts();
  updateVolume(Flow2Patient);
  comms_setVOL(getTotalVolumeInt());
  comms_setPRES(CurrentPressurePatient);

  int END_SWITCH_VALUE_STOP = digitalRead(ENDSIWTCH_FULL_PIN); //inhale
  int END_SWITCH_VALUE_START = digitalRead(ENDSWITCH_PUSH_PIN);
  //Serial.println(END_SWITCH_VALUE_STOP);
  //Serial.println(END_SWITCH_VALUE_START);

  switch (controller_state) {
    case ini:{
      resetVolume();
      // Check user input to start controller
      if (comms_getActive() == true) {
        controller_state = wait; // start controller
      }
    }break;
    case inhale:{ 
      // CALL PID for inhale
      BREATHE_setCurrentTime(inhale_start_time);
      BREATHE_CONTROL_setPointInhalePressure(target_pressure, target_risetime);
      BREATHE_CONTROL_setInhalePressure(CurrentPressurePatient);
      // update motor
      Speed = BREATHE_CONTROL_Regulate(); 
      //MOTOR_CONTROL_setValue(Speed);
      // check if end of inhale is reached
      if ((millis()-inhale_start_time) > target_inhale_time){
        time_pressure_reached=1;
      }
      // check if max volume is reached
      if (abs(getTotalVolumeInt()) > abs(target_volume)){
        Serial.println("Volume too high");
        MOTOR_CONTROL_setValue(0);
        time_pressure_reached=1;
      }
      else{
        MOTOR_CONTROL_setValue(Speed);
      }
      // check if we need to change state 
      controller_state = BREATHE_setToEXHALE(END_SWITCH_VALUE_STOP,time_pressure_reached);      
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
      MOTOR_CONTROL_setValue(0);
      // Restart when 1) inhalation detected OR 2) timer passed
      bool isEnhaling;
      if(CurrentPressurePatient < comms_getTS()){
        isEnhaling= true;
        comms_setTRIG(1);
      }
      else{
        isEnhaling= false;
      }
      if (((millis() - exhale_start_time) > target_exhale_time) || isEnhaling){ // TODO: replace true by underpressure
        controller_state = inhale;
        // Get values for plotting
        comms_setBPM(millis() - inhale_start_time);
        
        inhale_start_time = millis();
        start_time_pressure = millis();
        time_pressure_reached=0;

        resetVolume();

        // load 'new' setting values for controller
        target_inhale_time = comms_getInhaleTime();
        target_exhale_time = comms_getExhaleTime();
        target_pressure = comms_getPK();
        target_volume = comms_getVT();
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

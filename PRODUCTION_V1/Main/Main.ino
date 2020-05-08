#include "TimerThree.h"
#include "PINOUT.h"
#include <avr/wdt.h>
// for debuggin purposes: allows to turn off features
#define PYTHON 1
#define HARDWARE 0
#define DEBUGserial Serial3
 
//---------------------------------------------------------------
// VARIABLES
//---------------------------------------------------------------

unsigned long controllerTime = 10000; // us 
unsigned long interrupttime; 

volatile float CurrentPressurePatient = 0;
volatile float CurrentFlowPatient = 0;
volatile float Volume2Patient = 0;
volatile float CurrentVolumePatient = 0;

float fio2 = 0.2;

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
bool sensor_calibration_ok = false;
bool pressure_sens_init_ok = false;
bool flow_sens_init_ok = false;
bool isFlow2PatientRead = false;
bool isPatientPressureCorrect = false;
bool isAngleOK = false;
bool isPythonOK = false;
bool isAmbientPressureCorrect = false;
bool oxygen_init_ok = false;

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

  //--- check mains supply and battery voltage
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
   
  //--- set up flow sensor
  DEBUGserial.println("Setting up flow sensor: ");
  if (FLOW_SENSOR_INIT()) {
    flow_sens_init_ok = true;
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

  //-- set up oxygen control
  oxygen_init_ok = true;

  //-- setup done
  DEBUGserial.println("Setup done");

  //-- check alarms
  isAmbientPressureCorrect = BME_280_UPDATE_AMBIENT();
  temperature_OK = BME_280_CHECK_TEMPERATURE();
  checkSupply(&main_supply, &batt_supply, &battery_SoC, &battery_powered, &battery_above_25);
  checkALARM_init(oxygen_init_ok, pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, 
                  sensor_calibration_ok, fan_OK, battery_powered, battery_SoC, temperature_OK);
    
  //-- set up communication with screen
  if(PYTHON) initCOMM();
  if (!PYTHON) isPythonOK = true;

  //-- wait for calibration of flow and pressure sensors
  flow_sens_init_ok = false;
  pressure_sens_init_ok = false;
  DEBUGserial.println("WAIT FOR CALIBRATION");
  while(!pressure_sens_init_ok || !flow_sens_init_ok){
    recvWithEndMarkerSer0();
    if (PYTHON) doWatchdog();
    
    if (comms_getActive() == -1) {
      comms_resetActive();
      analogWrite(Speaker_PWM, 127);
      delay(200);
      analogWrite(Speaker_PWM, 0);
  
      flow_sens_init_ok = FLOW_SENSOR_CALIBRATE();
      pressure_sens_init_ok = PRESSURE_SENSOR_CALIBRATE();
    }
  }
  DEBUGserial.println("CALIBRATION OK");
  sensor_calibration_ok = true;
  checkALARM_init(oxygen_init_ok, pressure_sens_init_ok, flow_sens_init_ok, motor_sens_init_ok, 
                  sensor_calibration_ok, fan_OK, battery_powered, battery_SoC, temperature_OK);
  
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
  fan_OK = FanPollingRoutine();
  isFlow2PatientRead = FLOW_SENSOR_Measure(&CurrentFlowPatient,maxflowinhale,minflowinhale);
  fan_OK = FanPollingRoutine();
  isPatientPressureCorrect = BME280_readPressurePatient(&CurrentPressurePatient,maxpressureinhale,minpressureinhale);
  fan_OK = FanPollingRoutine();
  isAngleOK = HALL_SENSOR_getVolume(&Volume2Patient);
  fan_OK = FanPollingRoutine();
  
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
  checkALARM(fio2, CurrentPressurePatient, CurrentVolumePatient, controller_state, isPatientPressureCorrect,
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
      controller_state = BREATHE_setToEXHALE(target_pressure);  

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
        target_risetime = updateAutoFlow(target_risetime, target_inhale_time);
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

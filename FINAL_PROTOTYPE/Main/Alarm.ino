#ifndef ALARM_H
#define ALARM_H

#include <Wire.h>

unsigned int ALARM = 0;
unsigned int ALARMMASK = 0x77FF; // give alarm for all states except watchdog and battery power
unsigned int PYTHONMASK = 0xFFFF; // give alarm for all python errors

unsigned long WatchdogTimeRX = 3000; // 3 seconds watchdog
unsigned long WatchdogTimeTX = 1000; // 1 seconds watchdog
unsigned long lastWatchdogTimeRX = millis();
unsigned long lastWatchdogTimeTX = millis();

unsigned long CPU_TIMER = 3000; // 3 seconds watchdog
unsigned long lastCpuTime = millis();

float maxTemperature = 50;

#define ON_REQUEST_DEBOUNCE_CYCLES 40  // alarm is accepted after 400ms or in case of intermittent error, 40 more errors than not errors 
#define OFF_REQUEST_DEBOUNCE_CYCLES 50  // alarm is switched off  after 500ms without alarm request
#define ALARM_OFF 0
#define ALARM_ON 1
unsigned int alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES;
unsigned int debouncedAlarmOnOffState = ALARM_OFF;


//---------------------------------------------------------------
// ALARMS
//---------------------------------------------------------------

void ALARM_Short_Beep() {
  debouncedAlarmOnOffState = ALARM_ON;
  alarmDebounceCounter = 50;
}

void ALARM_init() {
  while (1);
}

// debounceAlarm is to be called every 10ms,
// it treats all alarms set the last 10ms and feeds it to the debounce filter
void ALARM_debounceAlarm()
{
  unsigned int alarmRequested =  (ALARM & ALARMMASK);
  unsigned int alarmStatusFromPython = 0;
  if (debouncedAlarmOnOffState == ALARM_OFF)
  {
    if (alarmRequested > 0)
    {
      alarmDebounceCounter--;
      if (alarmDebounceCounter == 0)
      {
        debouncedAlarmOnOffState = ALARM_ON;
        alarmDebounceCounter = OFF_REQUEST_DEBOUNCE_CYCLES;
      }
    }
    else // (alarmRequested == 0 )
    {
      if (alarmDebounceCounter < ON_REQUEST_DEBOUNCE_CYCLES)
      {
        alarmDebounceCounter++;
      }
    }
  }
  else  // (ALARM_ON == debouncedAlarmOnOffState)
  {
    if (alarmRequested > 0)
    {
      alarmDebounceCounter = OFF_REQUEST_DEBOUNCE_CYCLES;
    }
    else // (alarmRequested == 0 )
    {
      alarmDebounceCounter--;
      if (alarmDebounceCounter == 0)
      {
        debouncedAlarmOnOffState = ALARM_OFF;
        alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES;
      }
    }
  }

  // Check if the buzzer should be triggered, based on input from Python
  alarmStatusFromPython = comms_getAlarmSatusFromPython() & PYTHONMASK;

  // light purely determined by python
  if (alarmStatusFromPython > 0){
    LightOn();
    DEBUGserial.println(ALARM, BIN);
  }
  else{
    LightOff();
  }

  // In ini state, the alarm should not sound (only for start beep)
  if(controller_state != ini){
    if (alarmStatusFromPython > 0){
      if (!comms_getMT()){
        SpeakerOn();
      }
      else{
        SpeakerOff();
      }
    }
    else{
      SpeakerOff();
    }
  }

// OLD CODE: back when arduino was boss over it's own buzzer, good times...    
//  if ( (debouncedAlarmOnOffState == ALARM_ON) ||  (alarmStatusFromPython > 0 ) )
//  {    
//    if (!(comms_getMT() || transientMute)){
//      SpeakerOn();
//    }
//    else{
//      SpeakerOff();
//    }
//    LightOn();
//    DEBUGserial.println(ALARM, BIN);
//  }
//  else
//  {
//    SpeakerOff();
//    LightOff();
//  }
}

//----------------------------------------------------------
// alarm state
//----------------------------------------------------------
void setAlarmState(unsigned int alarm) {
  unsigned int alarmbyte = (0x0001 << alarm);
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;
}

//-----------------------------------------------------
// reset alarm state
//-----------------------------------------------------
void resetAlarmState(unsigned int alarm) {
  unsigned int alarmbyte = (0x0001 << alarm);
  alarmbyte = 0xFFFF ^ alarmbyte;
  // BITWISE AND current alarm with new to RESET
  ALARM &= alarmbyte;
}

//-----------------------------------------------------
// get alarm state
//-----------------------------------------------------
unsigned int ALARM_getAlarmState(void) {
  // return 0 if we are in startup transient
  if (transientMute){
    return 0;
  }
  else{
    return ALARM;
  }
}

//-----------------------------------------------------
// check alarm
//-----------------------------------------------------
void checkALARM_init( bool pressure_sens_init_ok, 
    bool flow_sens_init_ok, bool motor_sens_init_ok, bool hall_sens_init_ok, bool fan_OK, 
    bool battery_powered, float battery_SOC, bool temperature_OK)
    {
  if (temperature_OK == false){
    // check flow sensor temperature measurement
    setAlarmState(5);
  }
  else{
    resetAlarmState(5);
  }
  if (pressure_sens_init_ok==false){
    // Sensor calibration failed pressure
    setAlarmState(7);
  }
  else{
    resetAlarmState(7);
  }

   if (flow_sens_init_ok==false){
    // Sensor calibration failed flow
    setAlarmState(8);
  }
  else{
    resetAlarmState(8);
  }
  
   if (motor_sens_init_ok==false){
    // Motor limit switches check failed
    setAlarmState(9);
  }
  else{
    resetAlarmState(9);
  }
  
   if (hall_sens_init_ok==false){
    // hall sensor initialization failed
    setAlarmState(10);
  }
  else{
    resetAlarmState(10);
  }

  if (battery_powered){
    // switched to battery --> check if externally powered
    setAlarmState(11);
  }
  else{
    resetAlarmState(11);
  }

  if (battery_SoC<0.5){
    // SoC battery <50% -  low
    setAlarmState(12);
  }
  else{
    resetAlarmState(12);
  }

  if (battery_SoC<0.25){
    // SoC battery <25% - critical
    setAlarmState(13);
  }
  else{
    resetAlarmState(13);
  }
}

void checkALARM(float pressure, int volume, controller_state_t state,
    bool isPatientPressureCorrect, bool isFlow2PatientRead, bool fan_OK, 
    bool battery_powered, float battery_SOC, bool isAmbientPressureCorrect, bool temperature_OK)
    {
  if (pressure > comms_getPK() + comms_getADPK()){
  // max pressure exceeded
  setAlarmState(1);
  }
  else{
    resetAlarmState(1);
  }

  if (volume > comms_getVT() + comms_getADVT()){
    // max volume exceeded
    setAlarmState(2);
  }
  else{
    resetAlarmState(2);
  }
  
  if (pressure < comms_getPP() - comms_getADPP() && state != ini){
    // Peep deviation exceeded
    //setAlarmState(3);
  }
  else{
    //resetAlarmState(3);
  }

   if (isPatientPressureCorrect==false || isAmbientPressureCorrect == false){
    // check pressure sensor connected and reacting
    setAlarmState(4);
  }
  else{
    resetAlarmState(4);
  }
  
  if (temperature_OK == false){
    // check flow sensor temperature measurement
    setAlarmState(5);
  }
  else{
    resetAlarmState(5);
  }

  if (isFlow2PatientRead==false){
    // flow sensors sensor connected and reacting
    setAlarmState(6);
  }
  else{
    resetAlarmState(6);
  }   
  // removed initialisation errors to function above
  if (battery_powered){
    // switched to battery --> check if externally powered
    setAlarmState(11);
  }
  else{
    resetAlarmState(11);
  }

  if (battery_SoC<0.5){
    // SoC battery <50% -  low
    setAlarmState(12);
  }
  else{
    resetAlarmState(12);
  }

  if (battery_SoC<0.25){
    // SoC battery <25% - critical
    setAlarmState(13);
  }
  else{
    resetAlarmState(13);
  }

  if (fan_OK==false){
    // Fan not operational
    setAlarmState(14);
  }
  else{
    resetAlarmState(14);
  }

  if (isPythonOK==false){
    // Python not operational
    setAlarmState(15);
  }
  else{
    resetAlarmState(15);
  }
}

//---------------------------------------------------------------
// DEGRADED MODE
//---------------------------------------------------------------

bool checkDegradedMode(bool isFlow2PatientRead, bool isPatientPressureCorrect, bool isAmbientPressureCorrect){
  // if i2c sensors fail ==> disable i2c bus!
  if (!FLOW_SENSOR_CHECK_I2C()){
    DEBUGserial.println("=== RESET I2C SENSORS & GO TO SAFE MODE ===");
    FLOW_SENSOR_DISABLE();
    BME280_DISABLE();
    #ifdef hall_sensor_i2c
      HALL_SENSOR_DISABLE();
    #endif
    // flush i2c
    while(Wire.available()){
      Wire.read();
    }
    // disable i2c
    pinMode(SCL, INPUT);
    pinMode(SDA, INPUT);

    return 1;
  }
  else if(!(isFlow2PatientRead && isPatientPressureCorrect && isAmbientPressureCorrect)){
    DEBUGserial.println("=== GO TO SAFE MODE ===");
    return 1;
  }
  else{
    return 0;
  }
}

//---------------------------------------------------------------
// WATCHDOG
//---------------------------------------------------------------

void doWatchdog(void) {  
  // if watchdog timer has passed AND the communication was OK ==> reset communication
  if (millis() - lastWatchdogTimeRX > WatchdogTimeRX && isPythonOK == true) {
    isPythonOK = false;
    resetComm();      
  }

  // if python communication is gone, send settings and check if OK
  if (getSettings() && isPythonOK == false) {
    isPythonOK = true;   
  }

  // TX alarms
  if (millis() - lastWatchdogTimeTX > WatchdogTimeTX) {
    lastWatchdogTimeTX = millis();
    sendAlarmState();         
  }
}

void updateWatchdog(unsigned long newtime) {
  lastWatchdogTimeRX = newtime;
}

//---------------------------------------------------------------
// CPU watchdog
//---------------------------------------------------------------

unsigned long maxinterrupttime = 0;
unsigned long startinterrupttime = 0;

void CPU_TIMER_reset() {
  maxinterrupttime = 0;
}

void CPU_TIMER_start(unsigned long starttime) {
  startinterrupttime = starttime;
}

void CPU_TIMER_stop(unsigned long stoptime) {
  unsigned long interrupttime = stoptime - startinterrupttime;
  if (interrupttime >= maxinterrupttime) {
    maxinterrupttime = interrupttime;
  }
}

unsigned long CPU_TIMER_get() {
  return (100000 * maxinterrupttime / controllerTime);
}

void doCPU_TIMER() {
  if (millis() - lastCpuTime > CPU_TIMER) {
    sendCPUState();
    CPU_TIMER_reset();
    lastCpuTime = millis();
  }
}

#endif

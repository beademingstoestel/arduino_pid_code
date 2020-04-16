#ifndef ALARM_H
#define ALARM_H

#include <Wire.h>

unsigned int ALARM = 0;

unsigned long WatchdogTimeRX = 3000; // 3 seconds watchdog
unsigned long WatchdogTimeTX = 1000; // 1 seconds watchdog
unsigned long lastWatchdogTimeRX = millis();
unsigned long lastWatchdogTimeTX = millis();

unsigned long CPU_TIMER = 3000; // 3 seconds watchdog
unsigned long lastCpuTime = millis();

float maxTemperature = 50;

#define ON_REQUEST_DEBOUNCE_CYCLES 2  // alarm is accepted after 400ms or in case of intermittent error, 40 more errors than not errors 
#define OFF_REQUEST_DEBOUNCE_CYCLES 50  // alarm is switched off  after 500ms without alarm request
#define ALARM_OFF 0
#define ALARM_ON 1
unsigned int alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES;
unsigned int debouncedAlarmOnOffState = ALARM_OFF;


//---------------------------------------------------------------
// ALARMS
//---------------------------------------------------------------

void ALARM_init() {
  while (1);
}

void ALARM_processAlarm()
{
  // Get alarm status from python: alarms (bits) to be disabled are 1
  unsigned int alarmStatusFromPython = comms_getAlarmSatusFromPython();
  resetAlarmStatePython(alarmStatusFromPython);
  // Check if the buzzer should be triggered, based on bit 0 from python
  unsigned int buzzerStatusFromPython = alarmStatusFromPython << 15;
  
  // light purely determined by python
  if (buzzerStatusFromPython > 0){
    LightOn();
    DEBUGserial.println(ALARM, BIN);
  }
  else{
    LightOff();
  }

  // In ini state, the alarm should not sound (only for start beep)
  if(controller_state != ini){
    if (buzzerStatusFromPython > 0){
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

  // buzzer should always sound if PC is not connected
  if(!isPythonOK){
    SpeakerOn();
  }
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
// reset alarm state from python
//-----------------------------------------------------
void resetAlarmStatePython(unsigned int alarm) {
  // BITWISE AND current ALARM with alarm from python
  unsigned int alarmbyte = ALARM & alarm;
  // BITWISE XOR current ALARM with new to RESET
  ALARM ^= alarmbyte;
  // ignore LSB
  ALARM |= (alarmbyte & 0x0001);
}

//-----------------------------------------------------
// get alarm state
//-----------------------------------------------------
unsigned int ALARM_getAlarmState(void) {
    return ALARM;
}

//-----------------------------------------------------
// check alarm init
//-----------------------------------------------------
void checkALARM_init( bool pressure_sens_init_ok, 
    bool flow_sens_init_ok, bool motor_sens_init_ok, bool hall_sens_init_ok, bool fan_OK, 
    bool battery_powered, float battery_SOC, bool temperature_OK)
    {
  if (temperature_OK == false){
    // check flow sensor temperature measurement
    setAlarmState(5);
  }
  if (pressure_sens_init_ok==false){
    // Sensor calibration failed pressure
    setAlarmState(7);
  }
  if (flow_sens_init_ok==false){
    // Sensor calibration failed flow
    setAlarmState(8);
  }
   if (motor_sens_init_ok==false){
    // Motor limit switches check failed
    setAlarmState(9);
  }
   if (hall_sens_init_ok==false){
    // hall sensor initialization failed
    setAlarmState(10);
  }
  if (battery_powered){
    // switched to battery --> check if externally powered
    setAlarmState(11);
  }
  if (battery_SoC<0.5){
    // SoC battery <50% -  low
    setAlarmState(12);
  }
  if (battery_SoC<0.25){
    // SoC battery <25% - critical
    setAlarmState(13);
  }
}

//-----------------------------------------------------
// check alarm loop
//-----------------------------------------------------
void checkALARM(float pressure, int volume, controller_state_t state,
    bool isPatientPressureCorrect, bool isFlow2PatientRead, bool fan_OK, 
    bool battery_powered, float battery_SOC, bool isAmbientPressureCorrect, bool temperature_OK)
    {
  if (pressure > comms_getPK() + comms_getADPK()){
    // max pressure exceeded
    setAlarmState(1);
  }
  if (abs(volume) > comms_getVT() + comms_getADVT()){
    // max volume exceeded
    setAlarmState(2);
  }
  if (pressure < comms_getPP() - comms_getADPP() && state != ini){
    // Peep deviation exceeded
    //setAlarmState(3);
  }
   if (isPatientPressureCorrect==false || isAmbientPressureCorrect == false){
    // check pressure sensor connected and reacting
    setAlarmState(4);
  }
  if (temperature_OK == false){
    // check flow sensor temperature measurement
    setAlarmState(5);
  }
  if (isFlow2PatientRead==false){
    // flow sensors sensor connected and reacting
    setAlarmState(6);
  }
   
  // removed initialisation errors to function above
  
  if (battery_powered){
    // switched to battery --> check if externally powered
    setAlarmState(11);
  }
  if (battery_SoC<0.5){
    // SoC battery <50% -  low
    setAlarmState(12);
  }
  if (battery_SoC<0.25){
    // SoC battery <25% - critical
    setAlarmState(13);
  }
  if (fan_OK==false){
    // Fan not operational
    setAlarmState(14);
  }
  if (isPythonOK==false){
    // Python not operational
    setAlarmState(15);
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

  // TX alarms if communication is OK
  if (millis() - lastWatchdogTimeTX > WatchdogTimeTX && isPythonOK == true) {
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

int CPU_TIMER_get() {
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

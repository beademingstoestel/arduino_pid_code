#ifndef ALARM_H
#define ALARM_H

unsigned int ALARM = 0;
unsigned int ALARMMASK = 0xFFFF; // give alarm for all states by default

unsigned long Watchdog = 3000; // 3 seconds watchdog
unsigned long lastWatchdogTime = millis();

unsigned long CPU_TIMER = 3000; // 3 seconds watchdog
unsigned long lastCpuTime = millis();

#define ON_REQUEST_DEBOUNCE_CYCLES 10  // alarm is accepted after 100ms or in case of intermittent error, 10 more errors than not errors 
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
void debounceAlarm()
{
  unsigned int alarmRequested =  (ALARM & ALARMMASK);
  unsigned int alarmStatusFromPython = 0;
  if (ALARM_OFF == debouncedAlarmOnOffState)
  {
    if (alarmRequested > 0)
    {
      alarmDebounceCounter--;
      if (0 == alarmDebounceCounter)
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
      if (0 == alarmDebounceCounter)
      {
        debouncedAlarmOnOffState = ALARM_OFF;
        alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES;
      }
    }
  }

  alarmStatusFromPython = comms_getAlarmSatusFromPython();

//    DEBUGserial.print("python alarm status: ");
//    DEBUGserial.println(alarmStatusFromPython);
//    DEBUGserial.print("arduino alarm status: ");
//    DEBUGserial.println(debouncedAlarmOnOffState);
//    DEBUGserial.print("counter alarm: ");
//    DEBUGserial.println(alarmDebounceCounter);
//    DEBUGserial.print("arduino bit alarm: ");
//    DEBUGserial.println(ALARM);
    
  // TODO: ADD MASK FOR PYTHON MESSAGES!
  if ( (ALARM_ON == debouncedAlarmOnOffState) ||  (alarmStatusFromPython > 0 ) )
  {    
    SpeakerOn();
    LightOn();
  }
  else
  {
//    DEBUGserial.print("arduino alarm status: ");
//    DEBUGserial.println(debouncedAlarmOnOffState);
    SpeakerOff();
    LightOff();
  }
}

//----------------------------------------------------------
// alarm state
//----------------------------------------------------------
void setAlarmState(unsigned int alarm, unsigned long timer, controller_state_t state) {
  DEBUGserial.print("SET ALARM: ");
  DEBUGserial.println(alarm);
  DEBUGserial.print("CONTROL STATE: ");
  DEBUGserial.println(state);
  DEBUGserial.print("Timer: ");
  DEBUGserial.println(timer);

  unsigned int alarmbyte = (0x01 << alarm);
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;

  sendAlarmState();
}

//-----------------------------------------------------
// reset alarm state
//-----------------------------------------------------
void resetAlarmState(unsigned int alarm, unsigned long timer, controller_state_t state) {
  DEBUGserial.print("RESET ALARM: ");
  DEBUGserial.println(alarm);
  DEBUGserial.print("CONTROL STATE: ");
  DEBUGserial.println(state);
  DEBUGserial.print("Timer: ");
  DEBUGserial.println(timer);
//  DEBUGserial.print("arduino bit alarm: ");
//  DEBUGserial.println(ALARM);

  unsigned int alarmbyte = (0x01 << alarm);
  alarmbyte = 0xFFFF ^ alarmbyte;
  // BITWISE AND current alarm with new to RESET
  ALARM &= alarmbyte;

  sendAlarmState();
}

//-----------------------------------------------------
// get alarm state
//-----------------------------------------------------
int getAlarmState(void) {
  return ALARM;
}

//-----------------------------------------------------
// check alarm
//-----------------------------------------------------
void checkALARM(float pressure, int volume, unsigned long timer, controller_state_t state, 
    bool isPatientPressureCorrect, bool isFlow2PatientRead, bool pressure_sens_init_ok, 
    bool flow_sens_init_ok, bool motor_sens_init_ok, bool hall_sens_init_ok, bool fan_OK, 
    bool battery_powered, float battery_SOC)
    {
  if (pressure > comms_getPK() + comms_getADPK()){
  // max pressure exceeded
  setAlarmState(1,timer,state);
  }
  else{
    resetAlarmState(1,timer,state);
  }

  if (volume > comms_getVT() + comms_getADVT()){
    // max volume exceeded
    setAlarmState(2,timer,state);
  }
  else{
    resetAlarmState(2,timer,state);
  }
  
  if (pressure < comms_getPP() - comms_getADPP()){
    // Peep deviation exceeded
    setAlarmState(3,timer,state);
  }
  else{
    resetAlarmState(3,timer,state);
  }

   if (isPatientPressureCorrect==false){
    // check pressure sensor connected and reacting
    setAlarmState(4,timer,state);
  }
  else{
    resetAlarmState(4,timer,state);
  }

  if (isFlow2PatientRead==false){
    // flow sensors sensor connected and reacting
    setAlarmState(6,timer,state);
  }
  else{
    resetAlarmState(6,timer,state);
  }   

  if (pressure_sens_init_ok==false){
    // Sensor calibration failed pressure
    setAlarmState(7,timer,state);
  }
  else{
    resetAlarmState(7,timer,state);
  }

   if (flow_sens_init_ok==false){
    // Sensor calibration failed flow
    setAlarmState(8,timer,state);
  }
  else{
    resetAlarmState(8,timer,state);
  }
  
   if (motor_sens_init_ok==false){
    // Motor limit switches check failed
    setAlarmState(9,timer,state);
  }
  else{
    resetAlarmState(9,timer,state);
  }
  
   if (hall_sens_init_ok==false){
    // hall sensor initialization failed
    setAlarmState(10,timer,state);
  }
  else{
    resetAlarmState(10,timer,state);
  }

  if (battery_powered){
    // switched to battery --> check if externally powered
    setAlarmState(11,timer,state);
  }
  else{
    resetAlarmState(11,timer,state);
  }

  if (battery_SoC<0.5){
    // SoC battery <50% -  low
    setAlarmState(12,timer,state);
  }
  else{
    resetAlarmState(12,timer,state);
  }

  if (battery_SoC<0.25){
    // SoC battery <25% - critical
    setAlarmState(13,timer,state);
  }
  else{
    resetAlarmState(13,timer,state);
  }

  if (fan_OK==false){
    // Fan not operational
    setAlarmState(14,timer,state);
  }
  else{
    resetAlarmState(14,timer,state);
  }

  if (isPythonOK==false){
    // Python not operational
    setAlarmState(15,timer,state);
  }
  else{
    resetAlarmState(15,timer,state);
  }
}

//---------------------------------------------------------------
// WATCHDOG
//---------------------------------------------------------------

void doWatchdog(void) {  
  // if watchdog timer has passed AND the communication was OK ==> reset communication
  if (millis() - lastWatchdogTime > Watchdog && isPythonOK == true) {
    isPythonOK = false;
    resetComm();      
  }

  // if python communication is gone, send settings and check if OK
  if (getSettings() && isPythonOK == false) {
    isPythonOK = true;   
  }
}

void updateWatchdog(unsigned long newtime) {
  lastWatchdogTime = newtime;
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

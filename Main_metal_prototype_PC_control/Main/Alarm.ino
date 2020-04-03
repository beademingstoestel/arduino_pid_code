#ifndef ALARM_H
#define ALARM_H

unsigned int ALARM = 0;
unsigned int ALARMMASK = 0xFF; // give alarm for all states by default

unsigned long Watchdog = 3000; // 3 seconds watchdog
unsigned long lastWatchdogTime = millis();

unsigned long CPU_TIMER = 3000; // 3 seconds watchdog
unsigned long lastCpuTime = millis();

#define ON_REQUEST_DEBOUNCE_CYCLES 10  // alarm is accepted after 100ms or in case of intermittent error, 10 more errors than not errors 
#define OFF_REQUEST_DEBOUNCE_CYCLES 50  // alarm is switched off  after 500ms without alarm request
#define ALARM_OFF 1
#define ALARM_ON 1 
unsigned int alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES; 
unsigned int debouncedAlarmOnOffState = ALARM_OFF;

 
//---------------------------------------------------------------
// ALARMS
//---------------------------------------------------------------

void ALARM_init(){
  while(1);
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
        if (0 == alarmDebounceCounter)
        {
           debouncedAlarmOnOffState = ALARM_OFF;
           alarmDebounceCounter = ON_REQUEST_DEBOUNCE_CYCLES;
        }      
      }
    }

    alarmStatusFromPython = comms_getAlarmSatusFromPython();  
    if ( (ALARM_ON == debouncedAlarmOnOffState) ||  (alarmStatusFromPython > 0 ) )  
    {
      // SOUND ALARM
      // digitalWrite(ALARMLED, HIGH);
    }
    else 
    {
      //digitalWrite(ALARMLED, LOW);
    }
}


void setAlarmState(int alarm) {
  byte alarmbyte = 0x01 << alarm;
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;

  sendAlarmState(); // do we need to report this every call of this function? Once each 10ms should be enough?
}

void resetAlarmState(int alarm) {
  debounceAlarm(); // take current alarms into account fro debounce. Could be moved somewhere else.
  // BITWISE AND current alarm with new to RESET
  ALARM &= alarm;
}

int getAlarmState(void) {
  return ALARM;
}

void checkALARM(float pressure, int volume, unsigned long timer, controller_state_t state){
  if (pressure < 5 && state == inhale && timer > 500){
    //no pressure
    setAlarmState(3);
  }
  if (volume < 100 && state == inhale && timer > 500){
    // no flow
    setAlarmState(4);
  }
  if (pressure > comms_getPK() + comms_getADPK()){
    // max pressure exceeded
    setAlarmState(5);
  }
  if (pressure < comms_getPP() - comms_getADPP()){
    // Peep deviation exceeded
    setAlarmState(6);
  }
  if (volume > comms_getVT() + comms_getADVT()){
    // max volume exceeded
    setAlarmState(7);
  }
}

//---------------------------------------------------------------
// WATCHDOG
//---------------------------------------------------------------

void doWatchdog(void) {
  if (millis() - lastWatchdogTime > Watchdog) {
    setAlarmState(2);
    // reset the settings if connection lost
    if (!getSettings()){
      delay(1000);
      resetComm();
    }
  }
  
  // check if all settings are OK
  if(getSettings()){
    // reset watchdog if OK
    resetAlarmState(2);
  }
}

void updateWatchdog(unsigned long newtime){
  lastWatchdogTime = newtime;
}

//---------------------------------------------------------------
// CPU watchdog
//---------------------------------------------------------------

unsigned long maxinterrupttime = 0;
unsigned long startinterrupttime = 0;

void CPU_TIMER_reset(){
  maxinterrupttime = 0;
}

void CPU_TIMER_start(unsigned long starttime){
  startinterrupttime = starttime;
}

void CPU_TIMER_stop(unsigned long stoptime){
  unsigned long interrupttime = stoptime - startinterrupttime;
  if(interrupttime >= maxinterrupttime){
    maxinterrupttime = interrupttime;
  }
}

unsigned long CPU_TIMER_get(){
  return (100000 * maxinterrupttime / controllerTime);
}

void doCPU_TIMER(){
  if (millis() - lastCpuTime > CPU_TIMER) {
    sendCPUState();
    CPU_TIMER_reset();
    lastCpuTime = millis();
  }
}

#endif

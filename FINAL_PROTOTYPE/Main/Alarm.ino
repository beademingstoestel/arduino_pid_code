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
  // TODO: ADD MASK FOR PYTHON MESSAGES!
  if ( (ALARM_ON == debouncedAlarmOnOffState) ||  (alarmStatusFromPython > 0 ) )
  {
    //SpeakerOn();
    LightOn();
  }
  else
  {
    SpeakerOff();
    LightOff();
  }
}

//----------------------------------------------------------
// alarm state
//----------------------------------------------------------
void setAlarmState(int alarm) {
  DEBUGserial.print("SET ALARM: ");
  DEBUGserial.println(alarm);

  byte alarmbyte = 0x01 << alarm;
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;

  sendAlarmState();
}

//-----------------------------------------------------
// reset alarm state
//-----------------------------------------------------
void resetAlarmState(int alarm) {
  DEBUGserial.print("RESET ALARM: ");
  DEBUGserial.println(alarm);

  byte alarmbyte = 0x01 << alarm;
  alarmbyte = 0xFF ^ alarmbyte;
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
// TODO: UPDATE FROM EXCEL
void checkALARM(float pressure, int volume, unsigned long timer, controller_state_t state) {
  if (pressure < 5 && state == inhale && timer > 500) {
    //no pressure
    setAlarmState(3);
  }
  if (volume < 100 && state == inhale && timer > 500) {
    // no flow
    setAlarmState(4);
  }
  if (pressure > comms_getPK() + comms_getADPK()) {
    // max pressure exceeded
    setAlarmState(5);
  }
  if (pressure < comms_getPP() - comms_getADPP()) {
    // Peep deviation exceeded
    setAlarmState(6);
  }
  if (volume > comms_getVT() + comms_getADVT()) {
    // max volume exceeded
    setAlarmState(7);
  }

  // new added
  if (isPatientPressureCorrect == false) {
    // check pressure sensor connected and reacting
    setAlarmState(4);
  }
  if (isFlow2PatientRead == false) {
    // flow sensors sensor connected and reacting
    setAlarmState(6);
  }
  if (pressure_sens_init_ok == false) {
    // Sensor calibration failed pressure
    setAlarmState(8);
  }
  if (flow_sens_init_ok == false) {
    // Sensor calibration failed flow
    setAlarmState(9);
  }
  if (motor_sens_init_ok == false) {
    // Motor limit switches check failed
    setAlarmState(10);
  }
  if (hall_sens_init_ok == false) {
    // hall sensor initialization failed
    setAlarmState(11);
  }
  if (battery_powered) {
    // switched to battery --> check if externally powered tbd
    setAlarmState(12);
  }
  if (battery_SoC < 0.5) {
    // SoC battery <50% -  low
    setAlarmState(13);
  }
  if (battery_SoC < 0.25) {
    // SoC battery <25% - critical
    setAlarmState(14);
  }
  if (fan_OK == false) {
    // Fan not operational
    setAlarmState(16);
  }
}

//---------------------------------------------------------------
// WATCHDOG
//---------------------------------------------------------------

void doWatchdog(void) {
  if (millis() - lastWatchdogTime > Watchdog) {
    setAlarmState(15);
    // reset the settings if connection lost
    if (!getSettings()) {
      delay(1000);
      resetComm();
    }
  }

  // check if all settings are OK
  if (getSettings()) {
    // reset watchdog if OK
    resetAlarmState(15);
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

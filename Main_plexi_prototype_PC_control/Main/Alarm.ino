#ifndef ALARM_H
#define ALARM_H

unsigned int ALARM = 0;
unsigned int ALARMMASK = 0xFF; // give alarm for all states by default

unsigned long Watchdog = 3000; // 3 seconds watchdog
unsigned long lastWatchdogTime = millis();

//---------------------------------------------------------------
// ALARMS
//---------------------------------------------------------------

void setAlarmState(int alarm) {
  byte alarmbyte = 0x01 << alarm;
  // BITWISE OR current alarm with new to SET
  ALARM |= alarmbyte;

  // BITWISE AND current alarm with mask to check if alarm needs to be triggered
  if (ALARM & ALARMMASK) {
    // SOUND ALARM
    // digitalWrite(ALARMLED, HIGH);
  }
  else {
    //digitalWrite(ALARMLED, LOW);
  }
  sendAlarmState();
}

void resetAlarmState(int alarm) {
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
    resetComm();
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

#endif

#ifndef BREATHE_CONTROL_H
#define BREATHE_CONTROL_H

float PRESSURE_INHALE_SETPOINT = 0;
float current_inhale_pressure = 0;
float offset = 250;
float exhale_speed = 80;
float hold_speed = 0;
float delta_time;

bool volumeTriggered = false;
//----------------------------------
float Kp = 10;
float Ki = 0.1;
float Kd = 0;
float r = 0.5;  // forgetting factor for D-action
//-----------------------------------
float error = 0;
float error_t_m_1 = 0;
float dError = 0;
float dError_t_m_1 = 0;
float dError_unfiltered = 0;
//----------------------------------
float PID_value = 0;
float PID_value_P = 0;
float PID_value_I = 0;
float PID_value_D = 0;

int EXHALE_TIME = 0;
//------------------------------------------------------------------------------
void BREATHE_CONTROL_setPointInhalePressure(float setting, float risetime)
{
  delta_time = millis() - inhale_start_time; // time since INTAKE state has started
  if (delta_time < risetime) {
    PRESSURE_INHALE_SETPOINT = setting * delta_time / risetime;
  } else {
    PRESSURE_INHALE_SETPOINT = setting;
  }
  if(controller_state == exhale || controller_state == wait){
    PRESSURE_INHALE_SETPOINT = 0;
  }
}
//------------------------------------------------------------------------------
float BREATHE_CONTROL_getPointInhalePressure()
{
  return PRESSURE_INHALE_SETPOINT;
}
//------------------------------------------------------------------------------
void BREATHE_CONTROL_setInhalePressure(float setting)
{
  current_inhale_pressure = setting;
}
//------------------------------------------------------------------------------
float BREATHE_CONTROL_getInhalePressure()
{
  return current_inhale_pressure;
}
//------------------------------------------------------------------------------
float BREATHE_getPID()
{
  return PID_value;
}

//------------------------------------------------------------------------------
controller_state_t BREATHE_setToEXHALE()
{
  if ((millis() - inhale_start_time) > target_inhale_time) //   || Hall_position>=Hall_position_ref
  {
    PID_value_I = 0;
    PID_value_P = 0;
    exhale_start_time = millis();
    return exhale;
  }
  else {
    return inhale;
  }
}

//------------------------------------------------------------------------------
controller_state_t BREATHE_setToWAIT(int end_switch)
{
  if (((millis() - inhale_start_time) > 2 * target_inhale_time) && end_switch == 1)
  {
    return wait;
  }
  else {
    return exhale;
  }
}

//------------------------------------------------------------------------------
controller_state_t BREATHE_setToINHALE(bool inhale_detected)
{
  bool timepassed = 0;
  // check timer
  if (millis() - exhale_start_time > target_exhale_time) {
    timepassed = 1;
  }

  if (timepassed || inhale_detected)
  {
    return inhale;
  }
  else {
    return wait;
  }
}

bool BREATHE_CONTROL_CheckInhale() {
  bool inhale_detected = 0;
  // check negative flow
  if (CurrentFlowPatient > comms_getTS() && trigger_mode == 0) {
    inhale_detected = 1;
    comms_setTRIG(1);
  }
  // check underpressure
  if (CurrentPressurePatient < comms_getTP() && trigger_mode == 1) {
    inhale_detected = 1;
    comms_setTRIG(1);
  }
  return inhale_detected;
}

//------------------------------------------------------------------------------
float BREATHE_CONTROL_Regulate()
{
  DEBUGserial.print(CurrentFlowPatient);
  DEBUGserial.print(",");
  DEBUGserial.print(CurrentVolumePatient);
  DEBUGserial.print(",");
  DEBUGserial.print(Volume2Patient);
  DEBUGserial.print(",");
  DEBUGserial.print(CurrentPressurePatient);
  DEBUGserial.print(",");
  DEBUGserial.println(PRESSURE_INHALE_SETPOINT);

  float error = current_inhale_pressure - PRESSURE_INHALE_SETPOINT; //Motor direction is flipped clckwise is negative
  //Serial.println(diff);
  if (controller_state == inhale)
  {
    dError = r * dError_t_m_1 + 1000.0 * (1 - r) * (error - error_t_m_1) / time_diff;
    dError_unfiltered =  1000.0 * (error - error_t_m_1) / time_diff;
    PID_value_D = Kd * -dError;

    dError_t_m_1 = dError;
    error_t_m_1 = error;

    PID_value_P = Kp * error;
    PID_value_I += Ki * error;
    PID_value_I = (PID_value_I > offset) ? offset : PID_value_I;
    PID_value_I = (PID_value_I < -offset) ? -offset : PID_value_I;
    PID_value = PID_value_P + PID_value_I + PID_value_D;
    if (PID_value > 0) PID_value = 0;

    return PID_value;//PID_value;
  }
  else if (controller_state == exhale)
  {
    PID_value_I = 0;
    PID_value_P = 0;
    return exhale_speed;
  }
  else if (controller_state == wait) {
    return 0;
  }
}

float BREATHE_CONTROL_Regulate_With_Volume(int end_switch) {
  float Speed = BREATHE_CONTROL_Regulate();
  
  if (controller_state == inhale) {
    //if (abs(Volume2Patient) > abs(target_volume)) {
    if (abs(CurrentVolumePatient) > abs(target_volume) && volumeTriggered == false) {
      volumeTriggered = true;
      return hold_speed;
    }
    else if (end_switch == 1) {
      return hold_speed;
    }
    else {
      return Speed;
    }
  }

  else if (controller_state == exhale) {
    volumeTriggered = false;
    if (end_switch == 1) {
      return hold_speed;
    }
    else {
      return Speed;
    }
  }
}
//------------------------------------------------------------------------------
#endif

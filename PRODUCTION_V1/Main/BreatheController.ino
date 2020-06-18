#ifndef BREATHE_CONTROL_H
#define BREATHE_CONTROL_H

float PRESSURE_INHALE_SETPOINT = 0;
float current_inhale_pressure = 0;
float offset = 50;
float exhale_speed = 150;
float hold_speed = 0;
float delta_time;
float flow_at_switching = 0;
float peep_at_switching = 0;

//---------------------------------
// this is used in degraded mode with a linear scaling on the pressure setpoint
float min_degraded_mode_speed = -35;     

bool volumeTriggered = false;
//----------------------------------
bool endswitchFlag = false;
float preloadspeed0 = -5;
float preloadspeed1 = -20;
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
void BREATHE_CONTROL_setPointInhalePressure(float setting, float risetime, bool min_degraded_mode_ON)
{
  delta_time = millis() - inhale_start_time; // time since INTAKE state has started
  if (delta_time < risetime) {
    PRESSURE_INHALE_SETPOINT = comms_getPP() + (setting - comms_getPP()) * delta_time / risetime;
  } else {
    PRESSURE_INHALE_SETPOINT = setting;
  }
  if(min_degraded_mode_ON){
    PRESSURE_INHALE_SETPOINT = 10;
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

const int numReadingsAutoFlow = 10;
float readingsAutoFlow[numReadingsAutoFlow];  // the readings from the analog input
int readIndexAutoFlow = 0;                    // the index of the current reading
float totalAutoFlow = 0;                      // the running total

const int numReadingsPEEP = 10;
float readingsPEEP[numReadingsPEEP];      // the readings from the analog input
int readIndexPEEP = 0;                    // the index of the current reading
float totalPEEP = 0;                      // the running total
//------------------------------------------------------------------------------
controller_state_t BREATHE_setToEXHALE(unsigned int target_pressure, bool min_degraded_mode_ON)
{
  // keep running average of flow during inhale: for autoflow
  totalAutoFlow = totalAutoFlow - readingsAutoFlow[readIndexAutoFlow];
  readingsAutoFlow[readIndexAutoFlow] = CurrentFlowPatient;
  totalAutoFlow = totalAutoFlow + readingsAutoFlow[readIndexAutoFlow];
  readIndexAutoFlow = readIndexAutoFlow + 1;
  if (readIndexAutoFlow >= numReadingsAutoFlow) readIndexAutoFlow = 0;
  
  // check if inhale time has passed 
  if ((millis() - inhale_start_time) > target_inhale_time)
  {
    flow_at_switching = totalAutoFlow / numReadingsAutoFlow;
    
    PID_value_I = 0;
    PID_value_P = 0;
    exhale_start_time = millis();
    return exhale;
  }
  //OR if patient coughs (overpressure trigger)
  else if(CurrentPressurePatient > target_pressure + comms_getADPK() && !min_degraded_mode_ON){
    PID_value_I = 0;
    PID_value_P = 0;
    exhale_start_time = millis();
    return exhale;
  }
  // Otherwise, stay in inhale
  else {
    return inhale;
  }
}

//------------------------------------------------------------------------------
controller_state_t BREATHE_setToWAIT(int end_switch)
{
  // keep running average of pressure during exhale: for peep adjust
  totalPEEP = totalPEEP - readingsPEEP[readIndexPEEP];
  readingsPEEP[readIndexPEEP] = CurrentPressurePatient;
  totalPEEP = totalPEEP + readingsPEEP[readIndexPEEP];
  readIndexPEEP = readIndexPEEP + 1;
  if (readIndexPEEP >= numReadingsPEEP) readIndexPEEP = 0;
  
  // exhale time should be at least equal to exhale time ...
  if (((millis() - inhale_start_time) > 2 * target_inhale_time))
  {
    //peep_at_switching = totalPEEP / numReadingsPEEP;
    return wait;
  }
  // ... except if we are in APRV mode
  else if(comms_getAPRV() && (millis() - exhale_start_time) > target_exhale_time)
  {
    //peep_at_switching = totalPEEP / numReadingsPEEP;
    return wait;
  }
  // Otherwise, stay in exhale
  else {
    return exhale;
  }
}

//------------------------------------------------------------------------------
controller_state_t BREATHE_setToINHALE(bool inhale_detected)
{
  // keep running average of pressure during wait: for peep adjust
  totalPEEP = totalPEEP - readingsPEEP[readIndexPEEP];
  readingsPEEP[readIndexPEEP] = CurrentPressurePatient;
  totalPEEP = totalPEEP + readingsPEEP[readIndexPEEP];
  readIndexPEEP = readIndexPEEP + 1;
  if (readIndexPEEP >= numReadingsPEEP) readIndexPEEP = 0;
  
  // Check if time has passed, or patient triggered an inhale
  if ((millis() - exhale_start_time) > target_exhale_time || inhale_detected)
  {
    peep_at_switching = totalPEEP / numReadingsPEEP;
    return inhale;
  }
  // Otherwise, stay in wait
  else {
    return wait;
  }
}

bool BREATHE_CONTROL_CheckInhale() {
  bool inhale_detected = 0;
  // check negative flow
  if (CurrentFlowPatient > comms_getTS() && trigger_mode == 1) {
    inhale_detected = 1;
    comms_setTRIG(1);
  }
  // check underpressure
  if (CurrentPressurePatient < comms_getTP() && trigger_mode == 0) {
    inhale_detected = 1;
    comms_setTRIG(1);
  }
  return inhale_detected;
}

//------------------------------------------------------------------------------
//float BREATHE_CONTROL_Regulate()
float BREATHE_CONTROL_Regulate(bool min_degraded_mode_ON)
{
  //  DEBUGserialprint(CurrentFlowPatient);
  //  DEBUGserialprint(",");
  //  DEBUGserialprint(CurrentVolumePatient);
  //  DEBUGserialprint(",");
  //  DEBUGserialprint(Volume2Patient);
  //  DEBUGserialprint(",");
  //  DEBUGserialprint(CurrentPressurePatient);
  //  DEBUGserialprint(",");
  //  DEBUGserialprintln(PRESSURE_INHALE_SETPOINT);

  float error = current_inhale_pressure - PRESSURE_INHALE_SETPOINT; //Motor direction is flipped clckwise is negative
  //DEBUGserialprintln(diff);
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

//------------------------------------------------------------------------
float BREATHE_CONTROL_Regulate_With_Volume(int end_switch, bool min_degraded_mode_ON) {
  float Dead_Time_Volume = 0.25; // td = (0.01^2*pi*1)/(100/60/1000)
  float Speed = BREATHE_CONTROL_Regulate(min_degraded_mode_ON);
  // Return PID speed, unless max volume reached or minimal degraded mode
  if (controller_state == inhale ) {
    endswitchFlag = false;
    if ((CurrentVolumePatient+CurrentFlowPatient*Dead_Time_Volume/60000) > abs(target_volume) && comms_getVolumeLimitControl() && min_degraded_mode_ON == false) {
      volumeTriggered = true;
      return hold_speed;
    }
    else if (end_switch == 1) {
      volumeTriggered = true;
      return hold_speed;
    }
    else if (volumeTriggered){
      return hold_speed;
    }
    else if (min_degraded_mode_ON){
      return -30+min_degraded_mode_speed/comms_getInhaleTime()*1000;
    }
    else {
      return Speed;
    }
  }
  // 1) Move arm upward until endswitch
  // 2) Move down a bit to touch ambubag
  else if (controller_state == exhale) {
    volumeTriggered = false;
    if (endswitchFlag == false){
      if (end_switch == 1) {
        endswitchFlag = true;
        return preloadspeed0 + preloadspeed1/comms_getInhaleTime()*1000;
      }
      else{
        return Speed;
      }
    }
    else if (endswitchFlag == true){
      return preloadspeed0 + preloadspeed1/comms_getInhaleTime()*1000;
    }
  }
  // Keep arm steady in wait
  else if (controller_state == wait) {
    return hold_speed;
  }
}

//------------------------------------------------------------------------
float updateAutoFlow(float risetime, unsigned long target_inhale_time){
  // In autoflow: adjust risetime
  if (comms_getAutoFlow()){
    float delta = 50;
    
    // if the flow was zero too soon: slow down
    if(flow_at_switching < 2){
      risetime += delta;
      if (risetime > target_inhale_time - 200) risetime = target_inhale_time - 200;
    }
    // if the flow is not yet sufficiently LOW: speed up
    else if(flow_at_switching > 5){
      risetime -= delta;
      if (risetime < 200) risetime = 200;
    }
  }
  else{
    risetime = comms_getRP();
  }

  // Otherwise, don't change risetime
  return risetime;
}

//------------------------------------------------------------------------
float PEEP_error = 0;
float PEEP_error_int = 0;
float PEEP_Kp = 1;
float PEEP_Ki = 0;

void PEEP_update(){
    PEEP_error = peep_at_switching - comms_getPP();
    PEEP_error_int += PEEP_error;
    int PEEP_turn_direction = sgn(PEEP_error);
    float PEEP_turn_time = PEEP_Kp * PEEP_error + PEEP_Ki * PEEP_error_int;
    PEEP_turn_motor(PEEP_turn_direction, abs(PEEP_turn_time*1000));

//    Plot 
//    Serial.print(peep_at_switching);
//    Serial.print(",");
//    Serial.println(PEEP_error);
//    Serial.print(peep_at_switching);
//    Serial.print(",");
//    Serial.println(PEEP_error);
//    Serial.print(peep_at_switching);
//    Serial.print(",");
//    Serial.println(PEEP_error);
//    Serial.print(peep_at_switching);
//    Serial.print(",");
//    Serial.println(PEEP_error);
}

static inline int8_t sgn(float val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

#endif

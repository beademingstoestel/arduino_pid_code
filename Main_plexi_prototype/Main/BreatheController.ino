#ifndef BREATHE_CONTROL_H
#define BREATHE_CONTROL_H

// update very 40ms
float PRESSURE_INHALE_SETPOINT = 0;
float VOLUME_SETPOINT = 600; // in ml;
float BREATHING_SPEED_SETPOINT = 10; // in # per second
float current_inhale_pressure = 0;

float Vlung = 0;
bool is_blocking = false;

float MAX_DISPLACEMENT=500;
float offset = 250;
float exhale_speed = 80;

unsigned long current_time = 0;
unsigned long previous_exhale_time = 0;
float bpm = 0;
float delta_time;

//----------------------------------
float Kp = 250;
float Ki = 0;
float Kd = 0.018;
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
//----------------------------------
float PLUNGER_POSITION = 0;
//----------------------------------
int Hall_sensors_positions[12] = {186,193,200,206,211,215,219,221,225,228,231,234};
int Volumn[12] = {60,55,50,45,40,35,30,25,20,15,10,5};
//----------------------------------------


int EXHALE_TIME = 0;
//------------------------------------------------------------------------------
void BREATHE_CONTROL_setPointInhalePressure(float setting, float risetime)
{
  current_time = millis();
  delta_time = -start_time_pressure+current_time;  // time since INTAKE state has started
  if (delta_time < risetime){
    PRESSURE_INHALE_SETPOINT = setting*delta_time/risetime;
  }else{
  PRESSURE_INHALE_SETPOINT = setting;
  }
  //Serial.println(start_time_pressure);
  //Serial.println(current_time);
  //Serial.println(delta_time);

  
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
void BREATHE_setCurrentTime(unsigned long t)
{
  current_time = t;
}
                  
//------------------------------------------------------------------------------
controller_state_t BREATHE_setToEXHALE(int end_switch,bool time_pressure_reached)
{
  if ((end_switch==1 || time_pressure_reached==1)) //   || Hall_position>=Hall_position_ref
  {
    PID_value_I=0;
    PID_value_P=0;
    //-- compute exhale time
    unsigned long time_diff = current_time-previous_exhale_time;
    bpm = 60000.0/time_diff;
    previous_exhale_time = current_time;
    exhale_start_time = millis();
    return exhale;
  } 
  else{
    return inhale;
  }
}


//------------------------------------------------------------------------------
controller_state_t BREATHE_setToWAIT(int end_switch)
{  
    if (end_switch==1)
    {
      return wait;
    }  
    else{
      return exhale;
    }  
}
//------------------------------------------------------------------------------
float BREATHE_CONTROL_Regulate()
{
    float error = current_inhale_pressure-PRESSURE_INHALE_SETPOINT; //Motor direction is flipped clckwise is negative
    //Serial.println(diff);
    if (controller_state==inhale)
    {
      dError = r*dError_t_m_1 + 1000.0*(1-r)*(error-error_t_m_1)/time_diff;
      dError_unfiltered =  1000.0*(error-error_t_m_1)/time_diff;
      PID_value_D = Kd*-dError;
      
      dError_t_m_1 = dError;
      error_t_m_1 = error;
      
      PID_value_P = Kp*error; 
      PID_value_I += Ki*error;
      PID_value_I = (PID_value_I>offset)?offset:PID_value_I;
      PID_value_I = (PID_value_I<-offset)?-offset:PID_value_I;
      PID_value = PID_value_P + PID_value_I + PID_value_D;
      if (PID_value>0) PID_value=0;
      //Serial.println(error);
      //Serial.print(PID_value_P);
      //Serial.print(",");
      //Serial.println(PID_value_D);
      //Serial.print(",");
      //Serial.println(dError_unfiltered);
      //Serial.println("----");
      return PID_value;//PID_value;
    }
    else if (controller_state==exhale)
    {       
      PID_value_I=0;
      PID_value_P=0;     
      return exhale_speed;
    }    
}
//------------------------------------------------------------------------------
#endif

int Volumn2Position(int Volumn_ref)
{
  if (Volumn_ref>Volumn[1])
  {
    Serial.println("Reference too high, caping it to max value");
    return Hall_sensors_positions[1];
  }
  else if (Volumn_ref<Volumn[12])
  {
    Serial.println("Reference too high, caping it to min value");
    return Hall_sensors_positions[12];
  }
  else
  {
    int index_vol=0;
    while (Volumn_ref>Volumn[12-index_vol])
    {
      index_vol ++;
    }
    return Hall_sensors_positions[12-index_vol];
  }
}

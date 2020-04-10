#ifdef MOTORDRIVER_BTS7960
#include <BTS7960.h>

const uint8_t R_EN = R_EN_PIN;     //unsigned int 8 bit 
const uint8_t L_EN = L_EN_PIN;
const uint8_t L_PWM = L_PWM_PIN;
const uint8_t R_PWM = R_PWM_PIN;
BTS7960 motorController(R_EN, L_EN, L_PWM, R_PWM); //initialiseren van de motorcontroller
//----------------------------------------------------------------------------------
bool MOTOR_CONTROL_setup(int ENDSWITCH_PUSH, int ENDSWITCH_FULL){
  pinMode(ENDSWITCH_FULL,INPUT_PULLUP);
  pinMode(ENDSWITCH_PUSH,INPUT_PULLUP);
  
  MOTOR_CONTROL_setp();

  unsigned long starttime = millis();
  unsigned long maxsetuptime = 10000;
 
  // if already at bottom: go up until we clear the switch
  if(digitalRead(ENDSWITCH_FULL)){
    MOTOR_CONTROL_setValue(50);
    while(!digitalRead(ENDSWITCH_FULL)){
      if(millis() - starttime > maxsetuptime){
        MOTOR_CONTROL_setValue(0); 
        return false;
      }
    }
    delay(100);
    MOTOR_CONTROL_setValue(0);
  }
  // Go down to bottom switch
  MOTOR_CONTROL_setValue(-80);
  while(!digitalRead(ENDSWITCH_FULL)){
    if(millis() - starttime > maxsetuptime){
      MOTOR_CONTROL_setValue(0); 
      return false;
    }
  }
  MOTOR_CONTROL_setValue(0);
  // Go up to top endswitch
  MOTOR_CONTROL_setValue(50);
  while(!digitalRead(ENDSWITCH_PUSH)){
    if(millis() - starttime > maxsetuptime){
      MOTOR_CONTROL_setValue(0); 
      return false;
    }
  }
  MOTOR_CONTROL_setValue(0);

  return true;
}

//----------------------------------------------------------------------------------
void MOTOR_CONTROL_setp()
{
  motorController.Enable();
  //digitalWrite(R_EN,HIGH);
  //digitalWrite(L_EN,HIGH);
}
//----------------------------------------------------------------------------------
void MOTOR_CONTROL_setValue(int value)
{
  if (value>=0)
  {
      value = value>255?255:value;
      motorController.TurnLeft(value); //0->255
  }
  else
  {
      value = -value;
      value = value>255?255:value;
      motorController.TurnRight(value); //0->255
  }    
}
//----------------------------------------------------------------------------------


#endif

#ifdef MOTORDRIVER_VNH3SP30
  const int MOTOR_IN_A = Motor_IN_A_PIN;
  const int MOTOR_EN_A = Motor_EN_A_PIN;
  const int MOTOR_EN_B = Motor_EN_B_PIN;
  const int MOTOR_PWM = Motor_PWM_PIN;
  const int MOTOR_IN_B = Motor_IN_B_PIN;

//----------------------------------------------------------------------------------
bool MOTOR_CONTROL_setup(int ENDSWITCH_PUSH, int ENDSWITCH_FULL){
  pinMode(ENDSWITCH_FULL,INPUT_PULLUP);
  pinMode(ENDSWITCH_PUSH,INPUT_PULLUP);
  
  pinMode(MOTOR_IN_A, OUTPUT);
  digitalWrite(MOTOR_IN_A, LOW);
  pinMode(MOTOR_IN_B, OUTPUT);
  digitalWrite(MOTOR_IN_B, LOW);
  pinMode(MOTOR_PWM, OUTPUT);
  digitalWrite(MOTOR_PWM, LOW);
  //NOT USED, MOTOR DRIVER ERROR CONDITION IS secondary to all other failures
 // pinMode(Motor_EN_A, INPUT);  
 // pinMode(Motor_EN_B, INPUT);

  unsigned long starttime = millis();
  unsigned long maxsetuptime = 10000;
 
  // if already at bottom: go up until we clear the switch
  if(read_endswitch_stop()){
    MOTOR_CONTROL_setValue(50);
    while(!read_endswitch_stop()){
      if(millis() - starttime > maxsetuptime){
        MOTOR_CONTROL_setValue(0); 
        return false;
      }
    }
    delay(100);
    MOTOR_CONTROL_setValue(0);
  }
  // Go down to bottom switch
  MOTOR_CONTROL_setValue(-80);
  while(!read_endswitch_stop()){
    if(millis() - starttime > maxsetuptime){
      MOTOR_CONTROL_setValue(0); 
      return false;
    }
  }
  MOTOR_CONTROL_setValue(0);
  // Go up to top endswitch
  MOTOR_CONTROL_setValue(50);
  while(!read_endswitch_start()){
    if(millis() - starttime > maxsetuptime){
      MOTOR_CONTROL_setValue(0); 
      return false;
    }
  }
  MOTOR_CONTROL_setValue(0);

  return true;
}


//----------------------------------------------------------------------------------
void MOTOR_CONTROL_setValue(int value)
{
  if (value>=0)
  {
      value = value>255?255:value;
      Motor_TurnLeft(value); //0->255
  }
  else
  {
      value = -value;
      value = value>255?255:value;
      Motor_TurnRight(value); //0->255
  }    
}
//----------------------------------------------------------------------------------
void Motor_TurnLeft(uint8_t PWMValue){
   //analogWrite(MOTOR_PWM, PWMValue);
   digitalWrite(MOTOR_IN_A, HIGH);
   digitalWrite(MOTOR_IN_B, LOW);
   analogWrite(MOTOR_PWM, PWMValue);
}
void Motor_TurnRight(uint8_t PWMValue){
   //analogWrite(MOTOR_PWM, PWMValue);
   digitalWrite(MOTOR_IN_A, LOW);
   digitalWrite(MOTOR_IN_B, HIGH);
   analogWrite(MOTOR_PWM, PWMValue);
}

#endif

#include <BTS7960.h>
#include "PINOUT.h"

const uint8_t EN = EN_PIN;     
const uint8_t L_PWM = L_PWM_PIN;
const uint8_t R_PWM = R_PWM_PIN;
BTS7960 motorController(EN, L_PWM, R_PWM); //initialiseren van de motorcontroller
//----------------------------------------------------------------------------------
void MOTOR_CONTROL_setp()
{
  motorController.Enable();
}
//----------------------------------------------------------------------------------
void MOTOR_CONTROL_setValue(int value)
{
  if (value>=0)
  {
      value = value>255?255:value;
      motorController.TurnRight(value); //0->255
  }
  else
  {
      value = -value;
      value = value>255?255:value;
      motorController.TurnLeft(value); //0->255
  }    
}
//----------------------------------------------------------------------------------

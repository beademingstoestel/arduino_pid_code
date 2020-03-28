#include <BTS7960.h>

const uint8_t EN = 4;     //unsigned int 8 bit 
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 7;
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

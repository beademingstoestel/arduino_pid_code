#include "TimerThree.h"
 
void setup()
{
  pinMode(13, OUTPUT);
  Timer3.initialize(1000000);         // initialize timer3 in us, set 100 ms timing
  Timer3.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}
 
void callback()
{
  digitalWrite(13, digitalRead(13) ^ 1);
}
 
void loop()
{
  // your program here...
}
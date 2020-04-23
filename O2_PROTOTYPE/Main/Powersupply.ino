#ifndef POWERSUPPLY_H
#define POWERSUPPLY_H



void checkSupply(float *main_supply, float *batt_supply, float *battery_SoC, 
  bool *battery_powered, bool *battery_above_25)
{
  // main power supply
  *main_supply = MainSupplyVoltage()/1000;
  // battery voltage
  *batt_supply = PSUSupplyVoltage()/1000;
  // battery status percentage
  *battery_SoC = *batt_supply/25;
  // check if powered from battery only or if main supply is connected
  if(*main_supply > 23) *battery_powered = false;
  else *battery_powered = true;
  // check if battery is critically low
  if (*battery_SoC > 0.25) *battery_above_25 = true;  
  else *battery_above_25 = false;  
}

#endif

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
  // 1 battery: 13V = 100%, 10V = 0%
  *battery_SoC = (*batt_supply/2 - 10)*(0.333);
  
  // check if powered from battery only or if main supply is connected
  if(*main_supply > 23){
    *battery_powered = false;
  }
  else{
    *battery_powered = true;
    DEBUGserialprintln("NO PSU - RUNNING ON BATTERY POWER");
  }
  // check if battery is critically low
  if (*battery_SoC > 0.25){
    *battery_above_25 = true;  
  }
  else{
    *battery_above_25 = false; 
    DEBUGserialprintln("FAILED: BATTERY LOW"); 
  }
}

#endif

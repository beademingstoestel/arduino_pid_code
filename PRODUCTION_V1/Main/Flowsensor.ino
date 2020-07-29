#include <Wire.h>
#include "sdpsensor.h"
bool IS_FLOW_SENSOR_TUBE_INITIALIZED = false;
bool IS_FLOW_SENSOR_O2_INITIALIZED = false;
float Volume_l;
int Volume_ml;
float totalFlow = 0;
float Volume_l_O2 = 0;
int Volume_ml_O2 = 0;
float totalFlow_O2 = 0;
unsigned long deltaT;
bool resetAllowed = true;
float K_O2;

float Cp_O2 = 2.0/25;
float Ci_O2 = 0.0/25;
float o2air = 0.20;
float o2oxy = 1.00;
float wantedoxygenvolume = 0;
float maxvolumeoxygenaveraged = 0;
float fio2max = 1.02;
float fio2min = 0.25;
float valvetime;
float Vo2_error = 0;
float Vo2_cum_error = 0;

const int numReadingsO2 = 5;
float readingsO2[numReadingsO2] = {0,0,0,0,0};      
int readIndexO2 = 0;               
float totalO2 = 0;  

float density_air = 1.225; //kg/m3
float density_o2 = 1.429; //kg/m3

int flowsensordirection_tube = -1;
int flowsensordirection_O2 = -1;
float calibration_offset_tube = 0;
float calibration_offset_O2 = 0;
int sensorHealthyCounter = 0;
int sensorHealthyCounterO2 = 0;
int maxsensorHealthyCounter = 3;
//----------------------------------------------------------------------------------------------------------------
// SDP3x on the default I2C address of 0x21:
SDP3XSensor sdp_tube = SDP3XSensor(0x21);
SDP3XSensor sdp_O2 = SDP3XSensor(0x22);
//lookup table for forward current (mA) vs. normalized radiant flux. Key-value pair: {current, flux}
const double DP_VS_LPM[][2] = //RECALIBRATION
  //DP, Flow
  {{0.0, 0.0},
  {0.23 ,1.85},
  {0.28 ,2.21},
  {0.98 ,5.45},
  {2.48 ,10.24},
  {2.53 ,10.46},
  {4.1 ,14.2},
  {7.77 ,21.17},
  {8.3 ,22.7},
  {12.8 ,29.14},
  {16.1 ,32.28},
  {20.4 ,38.01},
  {24.6 ,42.86},
  {25.08 ,43.15},
  {34.18 ,50.34},
  {42.62 ,56.14},
  {47.4 ,60.21},
  {48.2 ,61.13},
  {55.1 ,65.6},
  {56.6 ,66.66},
  {60.9 ,70.07},
  {64.13 ,72},
  {70.8 ,76.2},
  {73.3 ,77.01},
  {81.9 ,82.18},
  {84.2 ,84.77},
  {96.3 ,91.02},
  {97.6 ,90.7},
  {109.4 ,96.68},
  {115.6 ,99.82},
  {119.8 ,101.4},
  {125 ,105.5},
  {129 ,106.1},
  {139.8 ,111.2},
  {152.6 ,117.8},
  {154.9 ,118.1},
  {162 ,122.3},
  {166.5 ,123.6},
  {178.5 ,128.5},
  {185.5 ,130.3},
  {193.2 ,133.9},
  {199 ,136.7},
  {209.8 ,140.8},
  {218 ,144.2},
  {220 ,146.1},
  {232 ,150},
  {242.2 ,152.6},
  {244.8 ,154.8},
  {268.5 ,161.9},
  {280 ,167.1},
  {281 ,167.6},
  {300 ,173},
  {302.5 ,173.5},
  {317 ,179},
  {322.6 ,178.8},
  {340 ,185.8},
  {343.5 ,185.4},
  {355.5 ,192.1},
  {370 ,194.9},
  {379.5 ,199.1},
  {391.5 ,199.9},
  {394.5 ,204.7},
  {423 ,210.9},
  {426 ,213.4},
  {459.5 ,219.5},
  {468 ,226.6},
  {499 ,232.3},
  {502 ,235.2},
  {536 ,243.7},
  {536.4 ,247.7},
  {546.05 ,253.9}};

//lookup table for forward current (mA) vs. normalized radiant flux. Key-value pair: {current, flux}

const double DP_vs_SLM[][2] =
  {{0.0  ,0.0},
  {0.15 ,1.1},
  {0.32 ,2.1},
  {0.54 ,3.2},
  {0.79 ,4.3},
  {1.07 ,5.4},
  {1.39 ,6.4},
  {1.75 ,7.5},
  {2.13 ,8.6},
  {2.52 ,9.7},
  {2.93 ,10.7},
  {4.08 ,12.9},
  {5.32 ,15},
  {6.65 ,17.2},
  {8.09 ,19.3},
  {9.65 ,21.5},
  {11.35  ,23.6},
  {13.18  ,25.8},
  {15.15  ,27.9},
  {17.26  ,30.1},
  {19.51  ,32.2},
  {21.92  ,34.3},
  {24.43  ,36.5},
  {27.07  ,38.6},
  {29.84  ,40.8},
  {32.71  ,42.9},
  {35.7 ,45.1},
  {38.76  ,47.2},
  {41.94  ,49.4},
  {45.21  ,51.5},
  {48.59  ,53.7},
  {52.06  ,55.8},
  {55.62  ,58},
  {59.24  ,60.1},
  {62.92  ,62.2},
  {66.67  ,64.4},
  {70.48  ,66.5},
  {74.38  ,68.7},
  {78.35  ,70.8},
  {82.38  ,73},
  {86.52  ,75.1},
  {97.57  ,80.5},
  {109.38 ,85.9},
  {121.82 ,91.2},
  {134.98 ,96.6},
  {148.59 ,102},
  {162.57 ,107.3},
  {183.3  ,115.1},
  {197.59 ,120.3},
  {212.03 ,125.6},
  {226.9  ,130.8},
  {242.51 ,136},
  {258.42 ,141.3},
  {274.74 ,146.5},
  {304.68 ,155.6},
  {322.5  ,161},
  {340.83 ,166.3},
  {359.43 ,171.7},
  {378.05 ,177.1},
  {397.91 ,182.4},
  {419.05 ,187.8},
  {439.32 ,193.2},
  {461.5  ,198.5},
  {484.33 ,203.9},
  {507.99 ,209.3}};
//----------------------------------------------------------------------------------------------------------------
char SLM[10];
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_INIT()
{
  FLOW_SENSOR_setDeltaT(controllerTime);
  Wire.begin();  
  delay(1000); // let serial console settle
  // reset the i2c communication
  int returnCode_reset = sdp_tube.resetI2C();
  // initialize sensors
  int returnCode = sdp_tube.init();
  if(returnCode == 0){
    IS_FLOW_SENSOR_TUBE_INITIALIZED=true;
  }
  if (OXYGENFLOWSENSOR){
    int returnCode_O2 = sdp_O2.init();
    if(returnCode_O2 == 0){
      IS_FLOW_SENSOR_O2_INITIALIZED=true;
    }
  }

  if (IS_FLOW_SENSOR_TUBE_INITIALIZED && (IS_FLOW_SENSOR_O2_INITIALIZED || !OXYGENFLOWSENSOR)) 
  {
    return true;
  } 
  else 
  {
    return false;
  }  
}

//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_CALIBRATE()
{
    bool flowcalOK = true;
    float currentVal;
    float sum = 0;
    float difference = 0;
    float thresholdcalOK = 0.3;
    float diff_error = 0;

    if(IS_FLOW_SENSOR_TUBE_INITIALIZED){
      // calibrate tube flow sensor
      sum = 0;
      for(int i=0;i<100;i++)
      {
        FLOW_SENSOR_Measure(0, &currentVal);
        sum+=currentVal;
        delay(50);
        // additional check based on differences
        difference = currentVal-(sum/(i+1));
        // returns false value if current flow sensor readout deviates from average by set threshold
        if (abs(difference)>thresholdcalOK && i>10) 
        {
           diff_error = difference;
           flowcalOK=false;
        }
      }    
      calibration_offset_tube =sum/100.0;
          
      DEBUGserialprint("Flow sensor offset: ");
      DEBUGserialprintln(calibration_offset_tube);
    }

    if (IS_FLOW_SENSOR_O2_INITIALIZED){
      // calibrate O2 flow sensor
      sum = 0;
      for(int i=0;i<100;i++)
      {
        FLOW_SENSOR_Measure(1, &currentVal);
        sum+=currentVal;
        delay(50);
        // additional check based on differences
        difference = currentVal-(sum/(i+1));
        if (abs(difference)>thresholdcalOK) // returns false value if current flow sensor readout deviates from average by set threshold
        {
           diff_error = difference;
           flowcalOK=false;
        }
      }    
      calibration_offset_O2 =sum/100.0;    
      DEBUGserialprint("Oxygen Flow sensor offset: ");
      DEBUGserialprintln(calibration_offset_O2);
    }
 
    return flowcalOK; // init successfull or not;
}
//----------------------------------------------------------------------------------------------------------------
// MEASURE
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_Measure(bool sensortype, float* value)
{
  if ((IS_FLOW_SENSOR_TUBE_INITIALIZED && !sensortype) || (IS_FLOW_SENSOR_O2_INITIALIZED && sensortype))
  {
    int ret;
    float DP; 

    if(!sensortype){
      ret = sdp_tube.readcont();
      if (ret == 0){    
        DP = sdp_tube.getDifferentialPressure();
        sensorHealthyCounter--;
        if (sensorHealthyCounter < 0){
          sensorHealthyCounter = 0;
        }
      }
      else{
        sensorHealthyCounter++;
        return true;
      }
    }
    else{
      ret = sdp_O2.readcont();
      if (ret == 0){    
        DP = sdp_O2.getDifferentialPressure();
        sensorHealthyCounterO2--;
        if (sensorHealthyCounterO2 < 0){
          sensorHealthyCounterO2 = 0;
        }
      }
      else{
        sensorHealthyCounterO2++;
        return true;
      }
    }

    bool neg = (DP<0?true:false);
    double x = abs(DP);
    double y, x0, x1, y0, y1;
    for (int i = 0; i < sizeof(DP_vs_SLM) / (sizeof(DP_vs_SLM[0][0]) * 2); i++)
    {
      if (x >= DP_VS_LPM[i][0] && x <= DP_VS_LPM[i + 1][0])
      {
        y0 = DP_VS_LPM[i][1];  //lower bound
        y1 = DP_VS_LPM[i + 1][1]; //upper bound
        x0 = DP_VS_LPM[i][0];
        x1 = DP_VS_LPM[i + 1][0];
        if(neg==false){
          y = abs(y0 + abs(abs((abs(x) - x0)) * ((y1 - y0) / (x1 - x0))));
        }
        else{
          y = -(y0 + abs(abs((abs(x) - x0)) * ((y1 - y0) / (x1 - x0))));
        }
      }
    }
    if(!sensortype){*value = (y*flowsensordirection_tube) - calibration_offset_tube;}
    if(sensortype){*value = (y*flowsensordirection_O2) - calibration_offset_O2;}
  } 
  return true;
}

bool FLOW_SENSOR_MeasurePatient(float *value, float maxflowinhale, float minflowinhale){
  bool SensorHealthy = FLOW_SENSOR_Measure(0, value);
  // Check if min-max is healthy:
  if (abs(maxflowinhale-minflowinhale)<0.5){
     SensorHealthy = false;
  }
  return SensorHealthy;
}

bool FLOW_SENSOR_MeasureO2(float *value, float maxflowinhale, float minflowinhale){
  bool SensorHealthy = FLOW_SENSOR_Measure(1, value);
  // Check if valve hasn't failed
  if (maxvolumeoxygenaveraged > 1000){
    SensorHealthy = false;
  }
  // Check if min-max is healthy:
  if (abs(maxflowinhale-minflowinhale)<0.5){
     SensorHealthy = false;
  }
  return SensorHealthy;
}

bool FLOW_SENSOR_MeasureO2(float *value){
  bool SensorHealthy = FLOW_SENSOR_Measure(1, value);
  // Check if valve hasn't failed
  if (maxvolumeoxygenaveraged > 1000){
    SensorHealthy = false;
  }
  return SensorHealthy;
}

//----------------------------------------------------------------------------------------------------------------
// VOLUME PATIENT
//----------------------------------------------------------------------------------------------------------------
int maxvolumepatient = 0;

void FLOW_SENSOR_hardresetVolume(){
    Volume_l = 0;
    totalFlow = 0;
    maxvolumepatient = 0;
}

void FLOW_SENSOR_resetVolume(){
  resetAllowed = true;
}

void FLOW_SENSOR_resetVolume_flowtriggered(){
  if(CurrentFlowPatient > 1 && resetAllowed){
    resetAllowed = false;
    Volume_l = 0;
    totalFlow = 0;
    maxvolumepatient = 0;
  }
}

void FLOW_SENSOR_updateVolume(float flow){ //flow = liter/min
  totalFlow += flow;
  Volume_l = totalFlow * ((float)deltaT / 60000);
  float densitycorrection = 1;//density_air/((FLOW_SENSOR_getFIO2()*density_o2)+((1-FLOW_SENSOR_getFIO2())*density_air));
  Volume_ml = (int)(Volume_l*1000.0/densitycorrection);
  if (Volume_ml > maxvolumepatient){
    maxvolumepatient = Volume_ml; 
  }  
}

int FLOW_SENSOR_getTotalVolumeInt(){ // Volume = ml
  return Volume_ml;
}

bool FLOW_SENSOR_getVolume(float *value){
  if (IS_FLOW_SENSOR_TUBE_INITIALIZED){
    *value = Volume_ml;
    return true;
  }
  return false;
}

int FLOW_SENSOR_getMaxVolume(){
  return maxvolumepatient;
}

//----------------------------------------------------------------------------------------------------------------
// VOLUME O2
//----------------------------------------------------------------------------------------------------------------
int maxvolumeoxygen = 0;

void FLOW_SENSOR_resetVolumeO2(){
    Volume_l_O2 = 0;
    totalFlow_O2 = 0;
    maxvolumeoxygen = 0;
}

void FLOW_SENSOR_updateVolumeO2(float flow_O2){ //flow = liter/min
  totalFlow_O2 += flow_O2;
  Volume_l_O2 = totalFlow_O2 * ((float)deltaT / 60000);
  float densitycorrection = 1;//density_air/density_o2;
  Volume_ml_O2 = (int)(Volume_l_O2*1000.0/densitycorrection); // TODO: CHECK IF THIS WORKS! DENSITY CORRECTION
  if (Volume_ml_O2 > maxvolumeoxygen){
    maxvolumeoxygen = Volume_ml_O2; 
  }
}

void FLOW_SENSOR_updateVolumeO2init(float flow_O2){ //flow = liter/min
  totalFlow_O2 += flow_O2;
  Volume_l_O2 = totalFlow_O2 / 60000;
  Volume_ml_O2 = (int)(Volume_l_O2*1000);
}

int FLOW_SENSOR_getTotalVolumeIntO2(){ // Volume = ml
  return Volume_ml_O2;
}

bool FLOW_SENSOR_getVolumeO2(float *value){
  if (IS_FLOW_SENSOR_O2_INITIALIZED){
    *value = Volume_ml_O2;
    return true;
  }
  return false;
}

int FLOW_SENSOR_getMaxVolumeO2(){
  return maxvolumeoxygen;
}

//----------------------------------------------------------------------------------------------------------------
// EXTRA FEATURES
//----------------------------------------------------------------------------------------------------------------

// set time interval
void FLOW_SENSOR_setDeltaT(unsigned long deltat){
  deltaT = deltat/1000;
}

void FLOW_SENSOR_DISABLE(){
  IS_FLOW_SENSOR_TUBE_INITIALIZED = 0;
  IS_FLOW_SENSOR_O2_INITIALIZED = 0;
}

bool FLOW_SENSOR_CHECK_I2C(){
  // check consecutive failures
  if ((sensorHealthyCounter >= maxsensorHealthyCounter) || (sensorHealthyCounterO2 >= maxsensorHealthyCounter)){
    return false;
  }else{
    return true;
  }
}

float FLOW_SENSOR_GET_TEMP(){
  return sdp_tube.getTemperature();
}

bool FLOW_SENSOR_CHECK_TEMP(){
  if (FLOW_SENSOR_GET_TEMP() > maxTemperature){
    return false;
  }
  else{
    return true;
  }
}

//----------------------------------------------------------------------------------------------------------------
// OXYGEN PID
//----------------------------------------------------------------------------------------------------------------               

void FLOW_SENSOR_setK_O2(float k_O2){
  K_O2 = k_O2;
}

unsigned long FLOW_SENSOR_getTime(float fio2){
  // don't add oxygen below 25% requested
  if (fio2 < fio2min){
    return 0;
  }
  // prevent inflation of bag
  if(fio2 > fio2max){
    fio2 = fio2max;
  }
  // calculate wanted oxygen volume
  wantedoxygenvolume = maxvolumepatient * (fio2-o2air)/(o2oxy-o2air);
  // calulate corresponding time to open valve
  valvetime = K_O2 * wantedoxygenvolume;
  
  // don't return negative valve time
  if (valvetime < 0){
    valvetime = 0;
  }
  return valvetime;
}

void FLOW_SENSOR_updateK_O2(){
  // calculate running average of supplied oxygen volume
  totalO2 = totalO2 - readingsO2[readIndexO2];
  readingsO2[readIndexO2] = maxvolumeoxygen;
  totalO2 = totalO2 + readingsO2[readIndexO2];
  readIndexO2 = readIndexO2 + 1;
  if (readIndexO2 >= numReadingsO2) readIndexO2 = 0;
  maxvolumeoxygenaveraged = totalO2 / numReadingsO2;

  // calculate error and update K_O2
  if(wantedoxygenvolume == 0) wantedoxygenvolume = 1;
  Vo2_cum_error += Vo2_error;
  Vo2_error = (wantedoxygenvolume - maxvolumeoxygenaveraged)/wantedoxygenvolume;
  K_O2 = K_O2 + Cp_O2 * Vo2_error + Ci_O2 * Vo2_cum_error;
  if(K_O2 < 0){
    K_O2 = 0;
  }
  
//  DEBUGserialprint("error: ");
//  DEBUGserialprintln(Vo2_error);
//  DEBUGserialprint("cumulative error: ");
//  DEBUGserialprintln(Vo2_cum_error*wantedoxygenvolume);
//  DEBUGserialprint("K: ");
//  DEBUGserialprintln(K_O2);
//  DEBUGserialprint("valveTime: ");
//  DEBUGserialprintln(valvetime);
//  DEBUGserialprint("Vpatient: ");
//  DEBUGserialprintln(maxvolumepatient);
//  DEBUGserialprint("Voxygenwanted: ");
//  DEBUGserialprintln(wantedoxygenvolume);
//  DEBUGserialprint("V02: ");
//  DEBUGserialprintln(maxvolumeoxygen);
//  DEBUGserialprint("FIO2: ");
//  DEBUGserialprintln(FLOW_SENSOR_getFIO2());
}

float FLOW_SENSOR_getFIO2(){
  if (maxvolumepatient == 0) maxvolumepatient = 10; // avoid divide by zero
  float fio2measured = ((o2oxy-o2air)*maxvolumeoxygenaveraged/maxvolumepatient)+o2air;

  if(fio2measured>1){
    fio2measured = 1;
  }
  if(fio2measured<0){
    fio2measured = 0;
  }
  return fio2measured;
}

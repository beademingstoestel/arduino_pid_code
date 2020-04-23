#include <Wire.h>
#include "sdpsensor.h"
bool IS_FLOW_SENSOR_INITIALIZED = false;
float Volume_l;
int Volume_ml;
float totalFlow = 0;
float Volume_l_O2 = 0;
int Volume_ml_O2 = 0;
float totalFlow_O2 = 0;
unsigned long deltaT;
bool resetAllowed = true;
float K_O2;

int flowsensordirection_tube = -1;
int flowsensordirection_O2 = -1;
float calibration_offset_tube = 0;
float calibration_offset_O2 = 0;
int sensorHealthyCounter = 0;
int maxsensorHealthyCounter = 3;
//----------------------------------------------------------------------------------------------------------------
// SDP3x on the default I2C address of 0x21:
SDP3XSensor sdp_tube = SDP3XSensor(0x21);
SDP3XSensor sdp_O2 = SDP3XSensor(0x22);
//lookup table for forward current (mA) vs. normalized radiant flux. Key-value pair: {current, flux}
const double DP_vs_SLM[][2] = {{0.0  ,0.0},{0.15 ,1.1},{0.32 ,2.1},{0.54 ,3.2},{0.79 ,4.3},
{1.07 ,5.4},{1.39 ,6.4},{1.75 ,7.5},{2.13 ,8.6},{2.52 ,9.7},{2.93 ,10.7},{4.08 ,12.9},{5.32 ,15},
{6.65 ,17.2},{8.09 ,19.3},{9.65 ,21.5},{11.35  ,23.6},{13.18  ,25.8},{15.15  ,27.9},{17.26  ,30.1},{19.51  ,32.2},
{21.92  ,34.3},{24.43  ,36.5},{27.07  ,38.6},{29.84  ,40.8},{32.71  ,42.9},{35.7 ,45.1},{38.76  ,47.2},{41.94  ,49.4},
{45.21  ,51.5},{48.59  ,53.7},{52.06  ,55.8},{55.62  ,58},{59.24  ,60.1},{62.92  ,62.2},{66.67  ,64.4},{70.48  ,66.5},
{74.38  ,68.7},{78.35  ,70.8},{82.38  ,73},{86.52  ,75.1},{97.57  ,80.5},{109.38 ,85.9},{121.82 ,91.2},{134.98 ,96.6},
{148.59 ,102},{162.57 ,107.3},{183.3  ,115.1},{197.59 ,120.3},{212.03 ,125.6},{226.9  ,130.8},{242.51 ,136},{258.42 ,141.3},
{274.74 ,146.5},{304.68 ,155.6},{322.5  ,161},{340.83 ,166.3},{359.43 ,171.7},{378.05 ,177.1},{397.91 ,182.4},{419.05 ,187.8},
{439.32 ,193.2},{461.5  ,198.5},{484.33 ,203.9},{507.99 ,209.3}};
//----------------------------------------------------------------------------------------------------------------
char SLM[10];
//----------------------------------------------------------------------------------------------------------------
// CALIBRATION
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_INIT()
{
  Wire.begin();  
  delay(1000); // let serial console settle
  // reset the i2c communication
  int returnCode_reset = sdp_tube.resetI2C();
  // initialize sensors
  int returnCode = sdp_tube.init();
  int returnCode_O2 = sdp_O2.init();
  // check if sensor initalisation OK & start calibration
  if (returnCode_reset == 0 && returnCode == 0 && returnCode_O2 == 0) 
  {
    IS_FLOW_SENSOR_INITIALIZED=true;
    delay(100);
    float currentVal;
    float sum;
    float difference = 0;
    bool flowcalOK = true;
    float thresholdcalOK = 0.3;
    float diff_error = 0;

    // calibrate tube flow sensor
    sum = 0;
    for(int i=0;i<100;i++)
    {
      FLOW_SENSOR_Measure(0, &currentVal);
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
    calibration_offset_tube =sum/100.0;    
    DEBUGserial.print("Flow sensor offset: ");
    DEBUGserial.println(calibration_offset_tube);

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
    DEBUGserial.print("Flow sensor offset: ");
    DEBUGserial.println(calibration_offset_O2);
    
    return flowcalOK; // init successfull or not;
  } 
  else 
  {
    return false; // init failed;
  }  
}
//----------------------------------------------------------------------------------------------------------------
// MEASURE
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_Measure(bool sensortype, float* value)
{
  bool SensorHealthy = false; //
  if (IS_FLOW_SENSOR_INITIALIZED)
  {
    int ret;
    float DP; 
    
    if(!sensortype){ret = sdp_tube.readcont();}
    if(sensortype){ret = sdp_O2.readcont();}
    
    if (ret == 0){    
      if(!sensortype){DP = sdp_tube.getDifferentialPressure();}
      if(sensortype){DP = sdp_O2.getDifferentialPressure();}
      
      sensorHealthyCounter--;
      if (sensorHealthyCounter < 0){
        sensorHealthyCounter = 0;
      }
    } 
    else{
      sensorHealthyCounter++;
      return true;
    }
    bool neg = (DP<0?true:false);
    double x = abs(DP);
    double y, x0, x1, y0, y1;
    for (int i = 0; i < sizeof(DP_vs_SLM) / (sizeof(DP_vs_SLM[0][0]) * 2); i++)
    {
      if (x >= DP_vs_SLM[i][0] && x <= DP_vs_SLM[i + 1][0])
      {
        y0 = DP_vs_SLM[i][1];  //lower bound
        y1 = DP_vs_SLM[i + 1][1]; //upper bound
        x0 = DP_vs_SLM[i][0];
        x1 = DP_vs_SLM[i + 1][0];
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
  return SensorHealthy;
}


bool FLOW_SENSOR_MeasurePatient(float *value, float maxflowinhale, float minflowinhale){
  bool SensorHealthy = FLOW_SENSOR_Measure(0, value);
  // Check if min-max is healthy:
  if (abs(maxflowinhale-minflowinhale)>0.01){
     SensorHealthy = true;
  }
  return SensorHealthy;
}

bool FLOW_SENSOR_MeasureO2(float *value){
  bool SensorHealthy = FLOW_SENSOR_Measure(1, value);
  // Check if min-max is healthy: TODO
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
  Volume_ml = (int)(Volume_l*1000);
  if (Volume_ml > maxvolumepatient){
    maxvolumepatient = Volume_ml; 
  }  
}

int FLOW_SENSOR_getTotalVolumeInt(){ // Volume = ml
  return Volume_ml;
}

bool FLOW_SENSOR_getVolume(float *value){
  if (IS_FLOW_SENSOR_INITIALIZED){
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
  Volume_ml_O2 = (int)(Volume_l_O2*1000);
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
  if (IS_FLOW_SENSOR_INITIALIZED){
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
  IS_FLOW_SENSOR_INITIALIZED = 0;
}

bool FLOW_SENSOR_CHECK_I2C(){
  // check consecutive failures
  if (sensorHealthyCounter >= maxsensorHealthyCounter){
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

float PID_K_O2 = 0.002;

void FLOW_SENSOR_setK_O2(float k_O2){
  K_O2 = k_O2;
}

unsigned long FLOW_SENSOR_getTime(float fio2){
  return K_O2 * maxvolumepatient * (fio2-0.2)/0.8;
}

void FLOW_SENSOR_updateK_O2(){
//  float error = (maxvolumeoxygen + peakoffset) - (maxvolumepatient * (fio2-0.2)/0.8);
  float error = (maxvolumeoxygen) - (maxvolumepatient * (fio2-0.2)/0.8);
  K_O2 = K_O2 - PID_K_O2 * error;
  DEBUGserial.print("error: ");
  DEBUGserial.println(error);
  DEBUGserial.print("K: ");
  DEBUGserial.println(K_O2);
}

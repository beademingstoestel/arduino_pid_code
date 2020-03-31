#include <Wire.h>
#include "sdpsensor.h"

#define CALIBRATION_SEQ 500
bool IS_FLOW_SENSOR_INITIALIZED = false;

float Volume;
float totalFlow = 0;
unsigned long numberoftriggers = 0;
unsigned long deltaT;                   // time between 2 triggers in ms
float DPoffset = 0;
//----------------------------------------------------------------------------------------------------------------
// SDP3x on the default I2C address of 0x21:
SDP3XSensor sdp;
// Density international standard atmosphere at 1013.25 hPa and 15 degrees centigrade
const float rho_isa = 1.225; // kg/m^3
const float A_tube = 1;
const float A_orifice = 0.9; // Fill in area of orifice! This value is CRUCIAL.
float A_rat_squared = 0;
const float R_spec = 287.058; // [J/(kg.K)]
//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_INIT()
{
  Wire.begin();
  delay(1000); // let serial console settle
  int returnCode = sdp.init();
  if (returnCode == 0)
  {
    IS_FLOW_SENSOR_INITIALIZED = true;
    // Calculate ratio beforehand for speed reasons
    A_rat_squared = (A_tube / A_orifice) * (A_tube / A_orifice);

    delay(100);
    FLOW_SENSOR_Calibrate(&DPoffset);
    Serial.print("OFFSET: ");
    Serial.println(DPoffset);
    
    return true; // init successfull;
  }
  else
  {
    return false; // init failed;
  }
}
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_Calibrate(float* offset)
{
  //int ret = sdp.readcont(); // check if we can measure data
  int ret = 0;
  float sum;          // measured sum of differential pressures [Pa]

  // Get differential pressure from sensor. Continuous read was enabled by Thomas. Every 0.5 ms a new sample is
  // ready. Average value will be returned when sample frequency is lower.

  if (ret == 0) {
    Serial.println("calib");
    for (int i = 0; i < CALIBRATION_SEQ; i++) {
      sdp.readcont();
      sum += sdp.getDifferentialPressure();
      Serial.println(sdp.getDifferentialPressure());
    }
    *offset = sum / (float) CALIBRATION_SEQ;
    return true;
  } else {
    // We cannot measure data, return without success.
    return false;
  }
  return true;
}
//----------------------------------------------------------------------------------------------------------------
bool FLOW_SENSOR_Measure(float *m_dot)
{
  if (IS_FLOW_SENSOR_INITIALIZED)
  {

    int ret = sdp.readcont(); // check if we can measure data
    float DP;					// measured differential pressure [Pa]
    //float m_dot;				// mass flow m_dot [kg/s]
    float pres;					// measured pressure [Pa]
    float temp;					// measured temperature [degrees C]
    float rho;

    // Get differential pressure from sensor. Continuous read was enabled by Thomas. Every 0.5 ms a new sample is
    // ready. Average value will be returned when sample frequency is lower.

    if (ret == 0) {
      // Fetch sensor data
      DP = (sdp.getDifferentialPressure() - DPoffset);
      Serial.println(DP);
      temp = (sdp.getTemperature());
      pres = 101325;

    } else {
      // We cannot measure data, return without success.
      return false;

    }

    // Sensor info for breathing support
    //
    // Sensor in use is SDP31. Scale factor therefore is 60 Pa-1 (p.7/14 + p.10/14 datasheet)
    // Differential pressure returned in DP, units Pascal.
    // Temperature returned is a value in degrees centigrade.

    // 1alculate density at this temperature
    //
    // rho = pres/(R_spec*(temp+278.15))
    //
    // 1) Calculate new rho
    //

    rho = pres / (R_spec * (temp + 278.15));


    // 2) Calculation of mass flow rate based on Venturi
    //
    // A1 = [m2] surface area of tube
    // A2 = [m2] surface area of orifice
    // m_dot = A1*sqrt( 2/rho * DP/((A1/A2)^2 - 1) )

    if (DP >= 0) {
      *m_dot = A_tube * sqrt( (2 / rho) * (DP / (A_rat_squared - 1)) );
    } else {
      *m_dot = -A_tube * sqrt( (2 / rho) * (-DP / (A_rat_squared - 1)) );
    }

    // Exit with success!
    return true;

  } else {

    // Initialize first, but I cannot return an error code :(
    return false;
  }
}

void resetVolume() {
  Volume = 0;
  totalFlow = 0;
  numberoftriggers = 0;
}

void updateVolume(float flow) { //flow = liter/min
  totalFlow += flow;
  numberoftriggers += 1;
}

float getTotalVolume() { // Volume = ml
  Volume = totalFlow * ((float)deltaT / 6000) * numberoftriggers;
  return Volume;
}

int getTotalVolumeInt() {
  getTotalVolume();
  int intVolume = (int)Volume;
  return intVolume;
}

// set time interval
void setDeltaT(unsigned long deltat) {
  deltaT = deltat / 1000;
}

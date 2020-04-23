#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_BMP280.h>

//-----------------------------------------------------------------------------------------------
Adafruit_BME280 bme1;//(0x77: Tube sensor);
Adafruit_BME280 bme2(BME_SPI_CS);  //(Ambient sensor over SPI);
Adafruit_Sensor *bme_pressure_patient1 = bme1.getPressureSensor();
Adafruit_Sensor *bme_pressure_ambient = bme2.getPressureSensor();
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();

bool PRESSURE_SENSOR1_INITIALIZED = false;
bool PRESSURE_SENSOR2_INITIALIZED = false;
bool PRESSURE_SENSOR3_INITIALIZED = false;

float PRESSURE_INIT_VALUE_BME = 0;
float PRESSURE_INIT_VALUE_MPL = 0;
float PRESSURE_INIT_VALUE_AMBIENT = 0;
float sum = 0;

#define hPa2cmh2o_scale 1.0197442889221
#define Pa2cmh2o_scale 0.010197442889221

//-----------------------------------------------------------------------------------------------
const int numReadings = 100;
const int updaterate = 10;
int updatecounter = 0;
float readings[numReadings];      // the readings from the analog input
int readIndex = 0;                // the index of the current reading
float total = 0;                  // the running total
//-----------------------------------------------------------------------------------------------
bool PRESSURE_SENSOR_INIT(){
  bool bme1ok = 0;
  bool bme2ok = 0;
  bool mplok = 0;

  bme1ok = BME1_Setup();
  bme2ok = BME2_Setup();
  mplok = MPL_Setup();

  return ((bme1ok || !BME_tube) && (bme2ok || !BME_ambient) && (mplok || !MPL_tube));
}
//-----------------------------------------------------------------------------------------------
bool BME1_Setup()
{
  if (!bme1.begin(0x77))
  {
    DEBUGserial.println("BME280 sensor for tube pressure not found");
    return false;
  }
  else
  {
    PRESSURE_SENSOR1_INITIALIZED = true;
    bme1.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BME280::SAMPLING_NONE,     /* Temp. oversampling */
                     Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                     Adafruit_BME280::SAMPLING_NONE,     /* Hum. oversampling */
                     Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                     Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */
    // calibrate
    delay(50);
    sum = 0;
    for (int i = 0; i < 50; i++)
    {
      sum += BME280_readpressure_cmH2O();
    }
    PRESSURE_INIT_VALUE_BME = sum / 50;
  }
  return true;
}
bool BME2_Setup()
{
  if (!bme2.begin())
  {
    DEBUGserial.println("BME280 sensor for ambient pressure not found");
    return false;
  }
  else
  {
    PRESSURE_SENSOR2_INITIALIZED = true;
    bme2.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BME280::SAMPLING_NONE,     /* Temp. oversampling */
                     Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                     Adafruit_BME280::SAMPLING_NONE,     /* Hum. oversampling */
                     Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                     Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */
    // calibrate
    delay(50);
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = BME280_readPressureAmbient();
      total += readings[thisReading];
    }
    PRESSURE_INIT_VALUE_AMBIENT = total / numReadings;
  }
  return true;
}
bool MPL_Setup()
{
  if (! mpl3115a2.begin()) {
    DEBUGserial.println("MPL sensor for tube pressure not found");
    return false;
  }
  else {
    delay(100);
    PRESSURE_SENSOR3_INITIALIZED = true;
    sum = 0;
    for (int i = 0; i < 50; i++)
    {
      sum += MPL3115A2_readpressure_cmH2O();
      delay(40); // add small delay: this sensor has long sampling time!
    }
    PRESSURE_INIT_VALUE_MPL = sum / 50;
  }
  return true;
}
//-----------------------------------------------------------------------------------------------
float BME280_readpressure_cmH2O()
{
  if (PRESSURE_SENSOR1_INITIALIZED)
  {
    sensors_event_t  pressure_event1;
    bme_pressure_patient1->getEvent(&pressure_event1);
    if(PRESSURE_SENSOR2_INITIALIZED){
      return (pressure_event1.pressure * hPa2cmh2o_scale) - PRESSURE_INIT_VALUE_AMBIENT;
    }
    else{
      return (pressure_event1.pressure * hPa2cmh2o_scale) - PRESSURE_INIT_VALUE_BME;
    }
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
float BME280_readPressureAmbient()
{
  if (PRESSURE_SENSOR2_INITIALIZED)
  {
    sensors_event_t  pressure_event_ambient;
    bme_pressure_ambient->getEvent(&pressure_event_ambient);
    return  (pressure_event_ambient.pressure * hPa2cmh2o_scale);
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
float MPL3115A2_readpressure_cmH2O() {
  if (PRESSURE_SENSOR3_INITIALIZED)
  {
    if(PRESSURE_SENSOR2_INITIALIZED){
      return (mpl3115a2.getPressureTHOMASVDD() * Pa2cmh2o_scale) - PRESSURE_INIT_VALUE_AMBIENT;
    }
    else{
      return (mpl3115a2.getPressureTHOMASVDD() * Pa2cmh2o_scale) - PRESSURE_INIT_VALUE_MPL;
    }
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
bool BME280_readPressurePatient(float *value,float maxpressureinhale, float minpressureinhale)
{
  float sensor1 = BME280_readpressure_cmH2O();
  bool SensorHealthy = false;

  //DEBUGserial.println(sensor1);

  *value = sensor1;

  if (abs(maxpressureinhale-minpressureinhale)>0.01){
      SensorHealthy= true;
  }
  return SensorHealthy;
}
//-----------------------------------------------------------------------------------------------

void BME280_DISABLE(){
    PRESSURE_SENSOR1_INITIALIZED = 0;
    PRESSURE_SENSOR2_INITIALIZED = 0;
    PRESSURE_SENSOR3_INITIALIZED = 0;
}

//-----------------------------------------------------------------------------------------------
bool BME_280_UPDATE_AMBIENT(){
  // don't recalculate every loop
  if (updatecounter == updaterate){
    updatecounter = 0;

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = BME280_readPressureAmbient();
    // check if the value is reasonable: between 650 and 1150 cmH2O
    if(readings[readIndex] > 1150 || readings[readIndex] < 650){
      return false;
    }
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
  
    // if we're at the end of the array wrap around to the beginning:
    if (readIndex >= numReadings) readIndex = 0;
  
    // calculate the average:
    PRESSURE_INIT_VALUE_AMBIENT = (total / numReadings);
  }
  updatecounter++;
  
  return true;
}

//-----------------------------------------------------------------------------------------------
bool BME_280_CHECK_TEMPERATURE(){
  if(PRESSURE_SENSOR2_INITIALIZED){
      if (bme2.getTemperature() > maxTemperature){
        return false;
      }
      else{
        return true;
      }
  }
  return false;
}

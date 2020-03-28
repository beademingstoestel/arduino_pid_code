/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2652

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_BMP280.h>

#define BME_SPI_SCK 52
#define BME_SPI_MISO 50
#define BME_SPI_MOSI 51 
#define BME_SPI_CS 48
//-----------------------------------------------------------------------------------------------
Adafruit_BME280 bme1;//(0x76);
Adafruit_BME280 bmp2;//(0x77); // use I2C interface
Adafruit_BME280 bme3(BME_SPI_CS); // hardware SPI
Adafruit_Sensor *bme_pressure_patient1 = bme1.getPressureSensor();
Adafruit_Sensor *bmp_pressure_patient2 = bmp2.getPressureSensor();
Adafruit_Sensor *bme_pressure_ref = bme3.getPressureSensor();

bool PRESSURE_SENSOR1_INITIALIZED = false;
bool PRESSURE_SENSOR2_INITIALIZED = false;
bool PRESSURE_SENSOR3_INITIALIZED = false;

#define PRESSURE_RA_SEQ_LENGTH 5
float PRESSURE_INIT_VALUE = 0;
float PRESSURE_RA_SEQ[PRESSURE_RA_SEQ_LENGTH];
int PRESSURE_RA_SEQ_WR_INDEX = 0;
float PRESSURE_SUM = 0;

Adafruit_MPL3115A2 redundant = Adafruit_MPL3115A2();

#define hPa2cmh2o_scale 1.0197442889221
//-----------------------------------------------------------------------------------------------
bool BME280_Setup() 
{
    if (!bme1.begin(0x77))
    {   
        Serial.println("BME280 sensor 1 not found"); 
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
    }
    delay(50); // wait here to settle;
   /* if (!bme2.begin(0x77)) 
    {    
        Serial.println("BMP280 sensor 2 not found"); 
        return false;        
    }
    else
    {
        PRESSURE_SENSOR2_INITIALIZED = true;
        bme2.setSampling(Adafruit_BME280::MODE_NORMAL,
          Adafruit_BME280::SAMPLING_NONE,
          Adafruit_BME280::SAMPLING_X1,
          Adafruit_BME280::SAMPLING_NONE,
          Adafruit_BME280::FILTER_OFF,
          Adafruit_BME280::STANDBY_MS_0_5);
}   */
    delay(50);
    /*if (!bme3.begin()) 
    {    
        Serial.println("BMP280 sensor 3 not found"); 
        return false;          
    }
    else
    {
        PRESSURE_SENSOR3_INITIALIZED = true;
        bme3.setSampling(Adafruit_BME280::MODE_NORMAL,
          Adafruit_BME280::SAMPLING_NONE,
          Adafruit_BME280::SAMPLING_X1,
          Adafruit_BME280::SAMPLING_NONE,
          Adafruit_BME280::FILTER_OFF,
          Adafruit_BME280::STANDBY_MS_0_5);
    }
    */
  //init buffer of running average filter
    for (int i=0;i<PRESSURE_RA_SEQ_LENGTH;i++)
    {
      PRESSURE_RA_SEQ[i]=0;  
    }
    
    float current_value;
    float sum = 0;
    for (int i=0;i<PRESSURE_RA_SEQ_LENGTH;i++)
    {
      BME280_readPressurePatient(&current_value);
    }
    
    for(int i=0;i<50;i++)
    {
      BME280_readPressurePatient(&current_value);
      sum+=current_value;
      //Serial.println (CurrentPressurePatient);
    }
    PRESSURE_INIT_VALUE = sum/50;

    
    return true;
}
//-----------------------------------------------------------------------------------------------
float BME280_readpressure1_cmH2O()
{
  if (PRESSURE_SENSOR1_INITIALIZED)
  {
    sensors_event_t  pressure_event1;
    bme_pressure_patient1->getEvent(&pressure_event1);
    return  pressure_event1.pressure*hPa2cmh2o_scale;
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
float BME280_readpressure2_cmH2O()
{
  if (PRESSURE_SENSOR2_INITIALIZED)
  {
    sensors_event_t  pressure_event2;
    bmp_pressure_patient2->getEvent(&pressure_event2);
    return  pressure_event2.pressure*hPa2cmh2o_scale;
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
float BME280_readpressure_ref_cmH2O()
{
  if (PRESSURE_SENSOR3_INITIALIZED)
  {
    sensors_event_t  pressure_event;
    bme_pressure_ref->getEvent(&pressure_event);
    return pressure_event.pressure*hPa2cmh2o_scale;
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
bool BME280_readPressurePatient(float *value) 
{
    float sensor1 = BME280_readpressure1_cmH2O();

    /*bme_pressure_patient2->getEvent(&pressure_event2);
    sensor2 =  pressure_event2.pressure*hPa2cmh2o_scale;

    if (abs(sensor1-sensor2)<100)
    {
      *value=(sensor1+sensor2)/2 - BME280_readPressureRef() + 38.9;
      return true;
    }
    return false;*/
    //*value=sensor1-1044.60;
    *value=sensor1-PRESSURE_INIT_VALUE;
    return true;
}
//-----------------------------------------------------------------------------------------------

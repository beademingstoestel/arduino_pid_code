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
#define BME_SPI_CS 47

//-----------------------------------------------------------------------------------------------
Adafruit_BME280 bme1;//(0x77: Tube sensor);
Adafruit_BME280 bme2(BME_SPI_CS);  //(Ambient sensor over SPI);
Adafruit_Sensor *bme_pressure_patient1 = bme1.getPressureSensor();
Adafruit_Sensor *bme_pressure_ambient = bme2.getPressureSensor();

bool PRESSURE_SENSOR1_INITIALIZED = false;
bool PRESSURE_SENSOR2_INITIALIZED = false;
bool PRESSURE_SENSOR3_INITIALIZED = false;

#define PRESSURE_RA_SEQ_LENGTH 5
float PRESSURE_INIT_VALUE_BME = 0;
float PRESSURE_INIT_VALUE_MLP = 0;
float PRESSURE_INIT_VALUE_AMBIENT = 0;
float PRESSURE_RA_SEQ[PRESSURE_RA_SEQ_LENGTH];
int PRESSURE_RA_SEQ_WR_INDEX = 0;
float PRESSURE_SUM = 0;

Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();

#define hPa2cmh2o_scale 1.0197442889221
#define Pa2cmh2o_scale 0.010197442889221
//-----------------------------------------------------------------------------------------------
bool BME280_Setup() 
{
    if (!bme1.begin(0x77))
    {   
        Serial.println("BME280 sensor for tube pressure not found"); 
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

    if (! mpl3115a2.begin()) {
      Serial.println("Couldnt find MPL sensor for tube pressure");
      return false;
    }
    else{
      PRESSURE_SENSOR2_INITIALIZED = true;
    }

    if (!bme2.begin())
    {   
        Serial.println("BME280 sensor for ambient pressure not found"); 
        return false;        
    }
    else
    {
        PRESSURE_SENSOR3_INITIALIZED = true;
        bme2.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BME280::SAMPLING_NONE,     /* Temp. oversampling */
                  Adafruit_BME280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BME280::SAMPLING_NONE,     /* Hum. oversampling */
                  Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */
    }

    
    delay(50); // wait here to settle;

    //init buffer of running average filter
    for (int i=0;i<PRESSURE_RA_SEQ_LENGTH;i++)
    {
      PRESSURE_RA_SEQ[i]=0;  
    }
    
    float sum = 0;
    for(int i=0;i<50;i++)
    {
      sum+= BME280_readpressure_cmH2O();
    }
    PRESSURE_INIT_VALUE_BME = sum/50;

    sum = 0;
    for(int i=0;i<50;i++)
    {
      sum+= MPL3115A2_readpressure_cmH2O();
    }
    PRESSURE_INIT_VALUE_MLP = sum/50;

    sum = 0;
    for(int i=0;i<50;i++)
    {
      sum+= BME280_readPressureAmbient();
    }
    PRESSURE_INIT_VALUE_AMBIENT = sum/50;

    
    return true;
}
//-----------------------------------------------------------------------------------------------
float BME280_readpressure_cmH2O()
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

float MPL3115A2_readpressure_cmH2O(){
  if (PRESSURE_SENSOR2_INITIALIZED)
  {
    return mpl3115a2.getPressure()*Pa2cmh2o_scale;
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
float BME280_readPressureAmbient()
{
  if (PRESSURE_SENSOR3_INITIALIZED)
  {
    sensors_event_t  pressure_event_ambient;
    bme_pressure_ambient->getEvent(&pressure_event_ambient);
    return  pressure_event_ambient.pressure*hPa2cmh2o_scale;
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------
bool BME280_readPressurePatient(float *value) 
{
    float sensor1 = BME280_readpressure_cmH2O();
    float sensor2 = MPL3115A2_readpressure_cmH2O();

    /*bme_pressure_patient2->getEvent(&pressure_event2);
    sensor2 =  pressure_event2.pressure*hPa2cmh2o_scale;

    if (abs(sensor1-sensor2)<100)
    {
      *value=(sensor1+sensor2)/2 - BME280_readPressureRef() + 38.9;
      return true;
    }
    return false;*/
    //*value=sensor1-1044.60;
    *value=sensor1-PRESSURE_INIT_VALUE_BME;
    
    return true;
}
//-----------------------------------------------------------------------------------------------

#ifdef hall_sensor_i2c
#include <AS5601.h>

//-----------------------------------------------------------------------------------------------------------
volatile AS5601 hall_encoder;     // I2C
volatile bool HALL_SENSOR_INITIALIZED = false;
int startangle = 0;
//-----------------------------------------------------------------------------------------------------------
bool HALL_SENSOR_INIT()
{
  //Serial.println("Setting up encoder");
  if (!hall_encoder.magnetDetected())
  {
    Serial.println("Error setting up AS5601 Hall sensor encoder, check magnet or encoder position.");
    return false;
  }
  HALL_SENSOR_INITIALIZED = true;
  return true;
}
//-----------------------------------------------------------------------------------------------------------
unsigned int HALL_SENSOR_read()
{
  if (HALL_SENSOR_INITIALIZED)
  {
    return hall_encoder.getRawAngle();
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------------------
bool HALL_SENSOR_readHall(unsigned int *value)
{
  if (HALL_SENSOR_INITIALIZED) {
    *value = hall_encoder.getRawAngle();
    return 1;
  }
  else {
    return 0;
  }
}

bool HALL_SENSOR_getVolume(float *value) {
  if (HALL_SENSOR_INITIALIZED) {
    int angle = hall_encoder.getRawAngle();
    if (angle - startangle > 500) {
      angle = (angle - startangle) - 4095;

    }
    else {
      angle = angle - startangle;
      //Serial.println(angle);
    }

    // Convert angle to value with calibration from excel file
    *value = 0.1*((float)angle * (float)angle * 0.0757 + (float)angle * 2.2519 + 0.0023);
    if (*value < 0) {
      *value = 0;
    }
    return 1;
  }
  else {
    return 0;
  }
}

bool HALL_SENSOR_calibrateHall()
{
  if (HALL_SENSOR_INITIALIZED) {
    startangle = hall_encoder.getRawAngle();
    startangle = startangle;
    return 1;
  }
  else {
    return 0;
  }
}

#else
#include <AS5040.h>

//-----------------------------------------------------------------------------------------------------------
volatile AS5040 hall_encoder(AS_SPI_SCK, AS_SPI_CS, AS_SPI_MISO);
volatile bool HALL_SENSOR_INITIALIZED = false;
int startangle = 0;
//-----------------------------------------------------------------------------------------------------------
bool HALL_SENSOR_INIT()
{
  //Serial.println("Setting up encoder");
  if (!hall_encoder.begin())
  {
    Serial.println("Error setting up AS5040 Hall sensor encoder");
    return false;
  }
  HALL_SENSOR_INITIALIZED = true;
  return true;
}
//-----------------------------------------------------------------------------------------------------------
unsigned int HALL_SENSOR_read()
{
  if (HALL_SENSOR_INITIALIZED)
  {
    return hall_encoder.read();
  }
  return 0;
}
//-----------------------------------------------------------------------------------------------------------
bool HALL_SENSOR_readHall(unsigned int *value)
{
  if (HALL_SENSOR_INITIALIZED) {
    *value = hall_encoder.read();
    return 1;
  }
  else {
    return 0;
  }
}

bool HALL_SENSOR_getVolume(float *value) {
  if (HALL_SENSOR_INITIALIZED) {
    int angle = hall_encoder.read();
    if (angle - startangle > 500) {
      angle = (angle - startangle) - 1023;

    }
    else {
      angle = angle - startangle;
      //Serial.println(angle);
    }

    // Convert angle to value with calibration from excel file
    *value = (float)(angle + 276) * 12.857 - 3979;
    if (*value < 0) {
      *value = 0;
    }
    return 1;
  }
  else {
    return 0;
  }
}

bool HALL_SENSOR_calibrateHall()
{
  if (HALL_SENSOR_INITIALIZED) {
    startangle = hall_encoder.read();
    startangle = startangle;
    return 1;
  }
  else {
    return 0;
  }
}

#endif

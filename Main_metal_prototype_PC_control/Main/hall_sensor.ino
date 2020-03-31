#include <AS5040.h>

//-----------------------------------------------------------------------------------------------------------
volatile AS5040 hall_encoder(AS_SPI_SCK, AS_SPI_CS, AS_SPI_MISO);
volatile bool HALL_SENSOR_INITIALIZED = false;
//-----------------------------------------------------------------------------------------------------------
bool HALL_SENSOR_INIT()
{
  //Serial.println("Setting up encoder");
  if (!hall_encoder.begin())
  {
    Serial.println("Error setting up AS5040 Hall sensor encoder");
    return false; 
  }
  HALL_SENSOR_INITIALIZED=true;
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
    if (HALL_SENSOR_INITIALIZED){
      *value = hall_encoder.read();
      return 1;
    }
    else{
      return 0;
    }
}

bool HALL_SENSOR_getVolume(float *value){
  if (HALL_SENSOR_INITIALIZED){
    unsigned int angle = hall_encoder.read();
    *value = (float)angle * 12.857 - 3979;
    if(*value < 0){
      *value = 0;
    }
    return 1;
  }
  else{
    return 0;
  }
}

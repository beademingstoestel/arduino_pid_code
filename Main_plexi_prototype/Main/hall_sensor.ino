#include <AS5040.h>

#define AS_SPI_SCK 52   // clk
#define AS_SPI_MISO 50  // DO pin
#define AS_SPI_MOSI 51  // DI pin
#define AS_SPI_CS 47    // CS for this device

/*#define CSpin   10
#define CLKpin  11
#define DOpin   12*/
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

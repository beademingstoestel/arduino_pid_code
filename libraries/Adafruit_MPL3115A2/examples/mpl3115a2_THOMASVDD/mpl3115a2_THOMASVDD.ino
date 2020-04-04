/**************************************************************************/
/*!
    @file     Adafruit_MPL3115A2.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Example for the MPL3115A2 barometric pressure sensor

    This is a library for the Adafruit MPL3115A2 breakout
    ----> https://www.adafruit.com/products/1893

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit_MPL3115A2 test!");
  Serial.println("Made awesome by THOMASVDD!");
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
}

float conversion = 0.010197162129779282; // pascal to mbar

void loop() {
  float pascals = baro.getPressureTHOMASVDD();
  Serial.println(pascals*conversion);
  delay(10);
}
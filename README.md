Put all the libraries contained in the 'libraries' folder into your local arduino library folder.
You can find the location of this folder from File>Preferences>Sketchbook location.

Board: Arduino Mega
Processor: atmega2560

Current version: metal prototype with pc control

Settings to be checked for prototype machines:

TURN PYTHON ON (1) OR OFF (0): 
	#define PYTHON 1
	
WHEN INITIALISATION FAILS, STOP MACHINE (1) OR CONTINUE ANYWAYS (0)
	#define HARDWARE 1
	
DEFINE DEBUG SERIAL PORT: Serial3 BY DEFAULT
	#define DEBUGserial Serial3	

USE I2C (UNCOMMENT LINE) OR SPI (COMMENT LINE) HALL SENSOR
	#define hall_sensor_i2c  
	
TURN PRESSURE SENSOR IN TUBE ON (1) OR OFF (0): 
	#define BME_tube 1
	
TURN AMBIENT PRESSURE SENSOR IN CASE ON (1) OR OFF (0): 
	#define BME_ambient 0



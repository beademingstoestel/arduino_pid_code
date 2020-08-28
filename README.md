




Put all the libraries contained in the 'libraries' folder into your local arduino library folder.
You can find the location of this folder from File>Preferences>Sketchbook location.

Board: Arduino Mega
Processor: atmega2560

______________________________________________________________

Use code under 'PRODUCTION V1' , all other versions are archives.

In Pinout.h select type of machine:
Either #define AUDI_V2 for motherboard rev2
Or  #define SHIELD_V3 for motherboard rev3

______________________________________________________________


If machine gives error code or does not connect, set PYTHON=0, you can then connect over Serial Port with machine.

TURN PYTHON ON (1) OR OFF (0):

	#define PYTHON 1


_____________________________________________________________
	
WHEN INITIALISATION FAILS, STOP MACHINE (1) OR CONTINUE ANYWAYS (0):

	#define HARDWARE 1
	


USE I2C (UNCOMMENT LINE) OR SPI (COMMENT LINE) HALL SENSOR:

	#define hall_sensor_i2c  
	
TURN PRESSURE SENSOR IN TUBE ON (1) OR OFF (0): 

	#define BME_tube 1
	
TURN AMBIENT PRESSURE SENSOR IN CASE ON (1) OR OFF (0): 

	#define BME_ambient 0



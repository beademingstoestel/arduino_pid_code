//#define plexi
//#define metal
//#define FINAL_PROTOTYPE
//#define AUDI_V1
#define AUDI_V2

#ifdef plexi
  #define ENDSWITCH_FULL_PIN 3 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 2 // exhale: upper switch

  #define EN_PIN 4
  #define L_PWM_PIN 6
  #define R_PWM_PIN 7

  #define AS_SPI_SCK 52   // clk
  #define AS_SPI_MISO 50  // DO pin
  #define AS_SPI_MOSI 51  // DI pin
  #define AS_SPI_CS 47    // CS for this device

  #define BME_SPI_SCK 52
  #define BME_SPI_MISO 50
  #define BME_SPI_MOSI 51 
  #define BME_SPI_CS 48

#endif

#ifdef metal
  #define ENDSWITCH_FULL_PIN 3 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 2 // exhale: upper switch

  #define MOTORDRIVER_BTS7960
  #define R_EN_PIN 3
  #define L_EN_PIN 26
  #define L_PWM_PIN 4
  #define R_PWM_PIN 7

  #define AS_SPI_SCK 52   // clk
  #define AS_SPI_MISO 50  // DO pin
  #define AS_SPI_MOSI 51  // DI pin
  #define AS_SPI_CS 47    // CS for this device

  #define BME_SPI_SCK 52
  #define BME_SPI_MISO 50
  #define BME_SPI_MOSI 51 
  #define BME_SPI_CS 48
#endif

#ifdef FINAL_PROTOTYPE
  #define hall_sensor_i2c

  #define BME_tube 1
  #define BME_ambient 1
  #define MPL_tube 1

  #define ENDSWITCH_FULL_PIN 3 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 2 // exhale: upper switch

  #define MOTORDRIVER_BTS7960
  #define R_EN_PIN 5
  #define L_EN_PIN 26
  #define L_PWM_PIN 6
  #define R_PWM_PIN 7

  #define AS_SPI_SCK 52   // clk
  #define AS_SPI_MISO 50  // DO pin
  #define AS_SPI_MOSI 51  // DI pin
  #define AS_SPI_CS 47    // CS for this device

  #define BME_SPI_SCK 52
  #define BME_SPI_MISO 50
  #define BME_SPI_MOSI 51 
  #define BME_SPI_CS 48

  #define Speaker_PWM 12
  #define Light_PWM 11
  #define Fan_PWM 45
  #define main_supply_voltage A4 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 5 //Scaling factor for resistor divider 20K over 5.1K ==> *5
  #define PSU_supply_voltage A3 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 5 //Scaling factor for resistor divider 20K over 5.1K ==> *5
  #define fan_speed A4

  #define FANPWMSETTING 100
  #define BEEPLENGTH 2000
#endif

#ifdef AUDI_V1
  #define hall_sensor_i2c

  #define BME_tube 1
  #define BME_ambient 1
  #define MPL_tube 1
  
  #define endswitches_inverted
  #define ENDSWITCH_FULL_PIN 7 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 6 // exhale: upper switch
  
  #define MOTORDRIVER_VNH3SP30
  #define Motor_IN_A_PIN 12
  #define Motor_EN_A_PIN 11
  #define Motor_EN_B_PIN 10
  #define Motor_PWM_PIN 9
  #define Motor_IN_B_PIN 8
  
  //SPI -> Fixed pins
  #define BME_SPI_SCK 52 // clk
  #define BME_SPI_MISO 50 // DO
  #define BME_SPI_MOSI 51 // DI
  #define BME_SPI_CS 19 // CS for external barometer

  //  #define Speaker_PWM 5 // original pinout PCB
  //  #define Light_PWM 4   // original pinout PCB
  
  #define Speaker_PWM 4 // ADJUSTED
  #define Light_PWM 5   // ADJUSTED
  #define Fan_PWM 3
  #define fan_speed A4
  
  #define main_supply_voltage A3 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define PSU_supply_voltage A2 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11

  // TX3 and RX3 to staggered debug header
  // (pinout debug header: 5V - TX - RX - GND, Square pad = pin1)
  
  #define FANPWMSETTING 100
  #define BEEPLENGTH 2000

  #define automatic_peep
#endif

#ifdef AUDI_V2
  #define hall_sensor_i2c

  #define BME_tube 1
  #define BME_ambient 1
  #define MPL_tube 1
  
  #define endswitches_inverted
  #define ENDSWITCH_FULL_PIN 7 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 6 // exhale: upper switch
  
  #define MOTORDRIVER_VNH3SP30
  #define Motor_IN_A_PIN 12
  #define Motor_EN_A_PIN 11
  #define Motor_EN_B_PIN 10
  #define Motor_PWM_PIN 9
  #define Motor_IN_B_PIN 8
  
  //SPI -> Fixed pins
  #define BME_SPI_SCK 52 // clk
  #define BME_SPI_MISO 50 // DO
  #define BME_SPI_MOSI 51 // DI
  #define BME_SPI_CS 19 // CS for external barometer

  //  #define Speaker_PWM 5 // original pinout PCB
  //  #define Light_PWM 4   // original pinout PCB
  
  #define Speaker_PWM 45 // ADJUSTED
  #define Light_PWM 13   // O2
  #define O2_valve 4
  #define O2_safety_valve 5
  #define Fan_PWM 3
  #define fan_speed A4
  
  #define main_supply_voltage A3 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define PSU_supply_voltage A2 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11

  // TX3 and RX3 to staggered debug header
  // (pinout debug header: 5V - TX - RX - GND, Square pad = pin1)
  
  #define FANPWMSETTING 100
  #define BEEPLENGTH 2000

  //#define automatic_peep
  #define oxygen_inhale_serial Serial3
  #define oxygen_exhale_serial Serial2

#endif

//#define plexi
//#define metal
//#define FINAL_PROTOTYPE
//#define AUDI_V1
//#define AUDI_V2
#define SHIELD_V3

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

  #define flowsensordirection_tube -1
  #define flowsensordirection_O2 -1
  
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

  #define OXYGENCONTROL 0
  #define OXYGENSENSORINHALE 0
  #define OXYGENSENSOREXHALE 0
  #define OXYGENFLOWSENSOR 0

  // correction factor for oxygen sensors: add 8%
  #define OXYGEN_CORRECTION 1.08

  //#define automatic_peep
  #define oxygen_inhale_serial Serial2
  #define oxygen_exhale_serial Serial3

  #define preloadspeed0 -5
  #define preloadspeed1 -20
  #define exhale_speed 80
  #define min_degraded_mode_speed -35
  
#endif

#ifdef AUDI_V2
  #define hall_sensor_i2c

  #define BME_tube 1
  #define BME_ambient 1
  #define MPL_tube 1

  #define flowsensordirection_tube -1
  #define flowsensordirection_O2 -1
  
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
  #define Light_PWM 13   // O2
  #define O2_valve 5
//  #define Fan_PWM 3
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
  #define Motor_PEEP_CW 44
  #define Motot_PEEP_CCW 46
  
  #define oxygen_inhale_serial Serial3
  #define oxygen_exhale_serial Serial2

  #define OXYGENCONTROL 0
  #define OXYGENSENSORINHALE 1
  #define OXYGENSENSOREXHALE 0
  #define OXYGENFLOWSENSOR 1

  // correction factor for oxygen sensors: add 8%
  #define OXYGEN_CORRECTION 1.08

  #define preloadspeed0 0
  #define preloadspeed1 0
  #define exhale_speed 150
  #define min_degraded_mode_speed -35

#endif

#ifdef SHIELD_V3
  #define hall_sensor_i2c

  #define BME_tube 1
  #define BME_ambient 1
  #define MPL_tube 1

  #define flowsensordirection_tube 1
  #define flowsensordirection_O2 1
  
  #define endswitches_inverted
  #define ENDSWITCH_FULL_PIN A9 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN A8 // exhale: upper switch
  
  #define MOTORDRIVER_VNH3SP30
  #define Motor_IN_A_PIN 12
  #define Motor_EN_A_PIN 11
  #define Motor_EN_B_PIN 10
  #define Motor_PWM_PIN 9
  #define Motor_IN_B_PIN 8
  
  //SPI -> Fixed pins
  #define BME_SPI_SCK 53 // clk
  #define BME_SPI_MISO 51 // DO -these have been switched to correct for mistake on board
  #define BME_SPI_MOSI 50 // DI
  #define BME_SPI_CS A0 // CS for external barometer

  //  #define Speaker_PWM 5 // original pinout PCB
  //  #define Light_PWM 4   // original pinout PCB
  
  #define Speaker_PWM 4 // ADJUSTED
  #define Light_PWM 5   // O2
  #define O2_valve A11
 #define Fan_PWM 3
  #define fan_speed A4
  
  #define main_supply_voltage A3 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define PSU_supply_voltage A2 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11

  // TX3 and RX3 to staggered debug header
  // (pinout debug header: 5V - TX - RX - GND, Square pad = pin1)
  
  #define FANPWMSETTING 75
  #define BEEPLENGTH 2000

  //#define automatic_peep
  #define Motor_PEEP_CW 44
  #define Motot_PEEP_CCW 46
  
  #define oxygen_inhale_serial Serial3 // this is the one being used
  #define oxygen_exhale_serial Serial2

  #define OXYGENCONTROL 0
  #define OXYGENSENSORINHALE 1
  #define OXYGENSENSOREXHALE 0
  #define OXYGENFLOWSENSOR 1

  // correction factor for oxygen sensors: add 8%
  #define OXYGEN_CORRECTION 1.08

  #define preloadspeed0 0
  #define preloadspeed1 0
  #define exhale_speed 150
  #define min_degraded_mode_speed -55

#endif

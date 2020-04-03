//#define plexi
//#define metal
#define protoseven

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

#ifdef protoseven
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
  #define BME_SPI_CS 47

  #define Speaker_PWM 12
  #define Light_PWM 11
  #define Fan_PWM 45
  #define main_supply_voltage A4 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define PSU_supply_voltage A3 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define fan_speed A4
  #define BEEPLENGTH 2000
#endif

#ifdef protoGroteSerie
  #define ENDSWITCH_FULL_PIN 7 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 6 // exhale: upper switch
  //Motor driver: VNH3SP30-E

  #define MOTORDRIVER_VNH3SP30
  #define Motor_IN_A 12
  #define Motor_EN_A 11
  #define Motor_EN_B 10
  #define Motor_PWM 9
  #define Motor_IN_B 8
  
  //SPI -> Fixed pins
  #define BME_SPI_SCK 52 // clk
  #define BME_SPI_MISO 50 // DO
  #define BME_SPI_MOSI 51 // DI
  #define BME_SPI_CS 19 // CS for external barometer
  
  #define Speaker_PWM 5
  #define Light_PWM 4
  #define Fan_PWM 3
  #define main_supply_voltage A2 // ~batt voltage if no PSU [ANALOG]
  #define MainSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define PSU_supply_voltage A3 // detect if PSU is unplugged [ANALOG OR DIGITAL]
  #define PSUSupplyVoltageScaling 11 //Scaling factor for resistor divider 10K over 1K ==> *11
  #define fan_speed A4
  // TX3 and RX3 to staggered debug header
  // (pinout debug header: 5V - TX - RX - GND, Square pad = pin1)
  #define BEEPLENGTH 2000
#endif

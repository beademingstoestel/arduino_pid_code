//#define plexi
//#define metal
#define protoseven

#ifdef plexi
  #define ENDSIWTCH_FULL_PIN 3 // inhale: lower switch
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
  #define ENDSIWTCH_FULL_PIN 3 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 2 // exhale: upper switch

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
  #define ENDSIWTCH_FULL_PIN 3 // inhale: lower switch
  #define ENDSWITCH_PUSH_PIN 2 // exhale: upper switch

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
#endif

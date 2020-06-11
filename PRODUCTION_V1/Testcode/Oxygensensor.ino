char read_oxygen[4] = {0x11, 0x01, 0x01, 0xED};

int I_output[] = {0,0,0,0,0,0,0,0,0,0,0,0}; //initialize output
int i = 0;  //initialize counter

int E_output[] = {0,0,0,0,0,0,0,0,0,0,0,0}; //initialize output
int j = 0;  //initialize counter


bool OXYGEN_SENSOR_INIT(){
  Serial2.begin(9600);
  Serial3.begin(9600);

  OXYGEN_SENSOR_READ_INHALE();
  OXYGEN_SENSOR_READ_EXHALE();
  delay(1000);
  float init_inhale = OXYGEN_SENSOR_GET_INHALE();
  float init_exhale = OXYGEN_SENSOR_GET_EXHALE();

  if(init_inhale<19 || init_inhale>22) return false;
  if(init_exhale<19 || init_exhale>22) return false;

  return true;
}

void OXYGEN_SENSOR_READ_INHALE(){
  Serial2.write(read_oxygen);
}

void OXYGEN_SENSOR_READ_EXHALE(){
  Serial3.write(read_oxygen);
}

float OXYGEN_SENSOR_GET_INHALE(){
  OXYGEN_SENSOR_READ_INHALE();
  i = 0;
  while (Serial2.available()) {
    int I_inByte = Serial2.read();
    I_output[i] = I_inByte;
    i++;
  }
  float I_concentration = (I_output[3]*256.0 + I_output[4])/10.0; //vol%
  float I_temperature = (I_output[7]*256.0 + I_output[8])/10.0; //Deg C
  Serial.print("Sensor inhale: ");
  Serial.println(I_concentration);
  return I_concentration;
}

float OXYGEN_SENSOR_GET_EXHALE(){
  OXYGEN_SENSOR_READ_EXHALE();
  j = 0;
  while (Serial3.available()) {
    int E_inByte = Serial3.read();
    E_output[j] = E_inByte;
    j++;
  }
  float E_concentration = (E_output[3]*256.0 + E_output[4])/10.0; //vol%
  float E_temperature = (E_output[7]*256.0 + E_output[8])/10.0; //Deg C
  Serial.print("Sensor exhale: ");
  Serial.println(E_concentration);
  return E_concentration;
}

// humidty inhale = humi ambient sensor * (Vtot - Voxygen)/Vtot + humi oxygen * Voxygen/Vtot
//                = humi ambient sensor * (Vtot - Voxygen)
// humidty exhale = humi tube sensor

unsigned long lastOxygenTime = millis();
unsigned long OXYGEN_TIMER = 1000; // 1 second

bool OXYGEN_SENSOR_MEASURE(){
  if (millis() - lastOxygenTime > OXYGEN_TIMER) {
    lastOxygenTime = millis();
    // get oxygen levels
    float oxygen_inhale = OXYGEN_SENSOR_GET_INHALE();
    float oxygen_exhale = OXYGEN_SENSOR_GET_EXHALE();
    // get humidity levels
    float humidity_patient = BME_280_GET_HUMIDTY_PATIENT();
    float humidity_ambient = BME_280_GET_HUMIDTY_AMBIENT();
    // convert humidty
    float humidity_inhale = humidity_ambient*(FLOW_SENSOR_getMaxVolume() - FLOW_SENSOR_getMaxVolumeO2())/FLOW_SENSOR_getMaxVolume();
    float humidity_exhale = humidity_patient;
    // MAP THE 2 VALUES!
  }
}

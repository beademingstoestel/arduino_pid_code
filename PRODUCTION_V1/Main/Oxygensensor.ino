bool OXYGEN_SENSORS_INITIALIZED = false;

char read_oxygen[4] = {0x11, 0x01, 0x01, 0xED};

int I_output[] = {0,0,0,0,0,0,0,0,0,0,0,0}; //initialize output
int i = 0;  //initialize counter

int E_output[] = {0,0,0,0,0,0,0,0,0,0,0,0}; //initialize output
int j = 0;  //initialize counter

int len_I = 0;
int len_E = 0;

byte sum_I = 0;
byte sum_E = 0;

float I_concentration = 0;
float I_temperature;

float E_concentration = 0;
float E_temperature;

bool OXYGEN_SENSOR_INIT(){
  if(OXYGENSENSORS){
    oxygen_inhale_serial.begin(9600);
    oxygen_exhale_serial.begin(9600);
  
    OXYGEN_SENSOR_READ_INHALE();
    OXYGEN_SENSOR_READ_EXHALE();
    while(OXYGEN_SENSOR_GET_INHALE() == 0 || OXYGEN_SENSOR_GET_EXHALE() == 0){
      delay(100);
    }
    float init_inhale = OXYGEN_SENSOR_GET_INHALE();
    float init_exhale = OXYGEN_SENSOR_GET_EXHALE();
  
    if(init_inhale<19 || init_inhale>25) return false;
    if(init_exhale<19 || init_exhale>25) return false;

    OXYGEN_SENSORS_INITIALIZED = true;
    return true;
  }
  else{
    OXYGEN_SENSORS_INITIALIZED = false;
    return true;
  }
}

void OXYGEN_SENSOR_READ_INHALE(){
  oxygen_inhale_serial.write(read_oxygen);
}

void OXYGEN_SENSOR_READ_EXHALE(){
  oxygen_exhale_serial.write(read_oxygen);
}

float OXYGEN_SENSOR_GET_INHALE(){
  if (oxygen_inhale_serial.available()){
    int I_inByte = oxygen_inhale_serial.read();  
    sum_I = sum_I + I_inByte;

    //start byte
    if (I_inByte == 0x16 && i >= len_I){
      i = 0;
      sum_I = I_inByte;
    }
    // length
    if (i==1){
      len_I = I_inByte;
    }
    // data
    if (i>=2 && i<11){
      I_output[i] = I_inByte;
    }
    // checksum
    if (i==11){
      sum_I = sum_I - I_inByte;
      if (I_inByte == 256 - sum_I){
        // end of correct message
        I_concentration = (I_output[3]*256.0 + I_output[4])/10.0; //vol%
        I_temperature = (I_output[7]*256.0 + I_output[8])/10.0; //Deg C
        DEBUGserialprint("O2 in: ");
        DEBUGserialprintln(I_concentration);
      }
    }   
    i++;
  }
  return I_concentration;
}

float OXYGEN_SENSOR_GET_EXHALE(){
  if (oxygen_exhale_serial.available()){
    int E_inByte = oxygen_exhale_serial.read();  
    sum_E = sum_E + E_inByte;

    //start byte
    if (E_inByte == 0x16 && j >= len_E){
      j = 0;
      sum_E = E_inByte;
    }
    // length
    if (j==1){
      len_E = E_inByte;
    }
    // data
    if (j>=2 && j<11){
      E_output[j] = E_inByte;
    }
    // checksum
    if (j==11){
      sum_E = sum_E - E_inByte;
      if (E_inByte == 256 - sum_E){
        // end of correct message
        E_concentration = (E_output[3]*256.0 + E_output[4])/10.0; //vol%
        E_temperature = (E_output[7]*256.0 + E_output[8])/10.0; //Deg C
        DEBUGserialprint("O2 ex: ");
        DEBUGserialprintln(E_concentration);
      }
    }   
    j++;
  }
  return E_concentration;
}

// humidty inhale = humi ambient sensor * (Vtot - Voxygen)/Vtot + humi oxygen * Voxygen/Vtot
//                = humi ambient sensor * (Vtot - Voxygen)
// humidty exhale = humi tube sensor

unsigned long lastOxygenTime = millis();
unsigned long OXYGEN_TIMER = 1000; // 1 second
bool OXYGEN_SWITCH = true;

bool OXYGEN_SENSOR_MEASURE(){
  if(OXYGEN_SENSORS_INITIALIZED){
    OXYGEN_SENSOR_GET_INHALE();
    OXYGEN_SENSOR_GET_EXHALE();
    if (millis() - lastOxygenTime > OXYGEN_TIMER) {
      lastOxygenTime = millis();
      // get oxygen levels
      if(OXYGEN_SWITCH==true){
        OXYGEN_SENSOR_READ_INHALE();
        OXYGEN_SWITCH = false;
      }
      else{
        OXYGEN_SENSOR_READ_EXHALE();
        OXYGEN_SWITCH = true;
      }
  
      comms_setFIO2inhale(I_concentration);
      comms_setFIO2exhale(E_concentration);
      DEBUGserialprint("O2 Flow: ");
      DEBUGserialprintln(FIO2*100);
    }
  }
}

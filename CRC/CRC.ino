char str[20] = {};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello");

  getCRC(str);
  Serial.println(str);
  Serial.println(checkCRC(str));
}

void loop() {
  // put your main code here, to run repeatedly:

}

int getCRC(char* str) {
    char checksum = 0;
    char i;
    int test = 500;
    
    sprintf(str, "BPM=%d=", test);
    // calculate CRC with bitwise XOR of each character
    for (i = 0; i < strlen(str); i++) {
        checksum ^= str[i];
    }
    // append CRC (1 byte) to string
    sprintf(str + strlen(str), "%c", checksum);
    
    return 1;
}

int checkCRC(char* str){
  char checksum = 0;
  char i;
  // compute bitwise XOR of full string ==> should be 0!
  for (i = 0; i < strlen(str); i++) {
        checksum ^= str[i];
  }
  return checksum;
}

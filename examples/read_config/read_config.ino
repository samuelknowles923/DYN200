#include <DYN200.h>

// Pin definitions - For useing MAX485 for communication 
const int MAX485ControlPin = 4; // Control pin for both DE/RE on MAX485

// Default DYN-200 slave ID
const uint8_t slaveAddress = 0x01; // Default, change as needed

// Instantiate DYN200 object
DYN200 dyn200(slaveAddress, Serial2, MAX485ControlPin); //  Define device at "slaveAddress" using "Serial2" with "MAX485ControlPin" for MAX485 send recive mode

void setup() {

  Serial.begin(9600); // For debugging
  
  dyn200.begin();
  Serial.println("");
  Serial.println(F("DYN-200 Initialized"));
  
  delay(250);
  String config = dyn200.getConfig();
  Serial.println(config);

  Serial.print("Last error message: ");
  Serial.println(dyn200.getLastError());

}

void loop() {
  /*
  // Read values
  //float torque, rpm, power; 
  //int32_t torque; uint32_t rpm, power;
  int16_t torque; uint16_t rpm, power;

  if (dyn200.readTorque(torque)) {
    Serial.print(F("Torque: ")); 
    Serial.println(torque);
  } else {
    Serial.print(F("Torque read failed: "));
    Serial.println(dyn200.getLastError());
  }

  if (dyn200.readRPM(rpm)) {
    Serial.print(F("RPM: ")); 
    Serial.println(rpm);
  } else {
    Serial.print(F("RPM read failed: "));
    Serial.println(dyn200.getLastError());
  }

  if (dyn200.readPower(power)) {
    Serial.print(F("Power: ")); 
    Serial.println(power);
  } else {
    Serial.print(F("Power read failed: "));
    Serial.println(dyn200.getLastError());
  }


  delay(1000); // Wait before next read
  //*/
}

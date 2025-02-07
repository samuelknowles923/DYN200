#include "DYN200.h"
#include <limits.h>

// Initialize the static member variable
uint8_t DYN200::_MAX485ControlPin = 0;  // Default value
DYN200* DYN200::_instance = nullptr;   // Initialize instance pointer

// Constructor
DYN200::DYN200(uint8_t slaveID, HardwareSerial& serial, uint8_t MAX485ControlPin)
  : _slaveID(slaveID), _serial(serial), _lastErrorCode(0) {
  _node.begin(slaveID, serial);
  _instance = this;  // Set the static instance pointer

  // Set the static MAX485 control pin if not already set
  if (_MAX485ControlPin == 0) {  // Only set it if not already initialized
    _MAX485ControlPin = MAX485ControlPin;
  }
}

// Initialize Modbus communication
bool DYN200::begin(uint32_t baudRate = 19200) {
  pinMode(_MAX485ControlPin, OUTPUT);
  digitalWrite(_MAX485ControlPin, LOW); // Start in receive mode

  // Check if the baud rate is valid
  switch (baudRate) {
    case 9600:
    case 14400:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
      DYN200::_baudRate = baudRate; // Set baud rate if valid
      break;
    default:
      _lastErrorCode = DYN200::invalidBaud;
      return false; // Invalid baud rate
  }
  
  // Start Serial Communications
  _serial.end();
  _serial.begin(baudRate);

  // Set up Modbus transmission callbacks
  _node.preTransmission(preTransmission);
  _node.postTransmission(postTransmission);

  // Verify connection to device by reading registers
  uint8_t result = _node.readHoldingRegisters(0x00, 6);
  if(result != _node.ku8MBSuccess){
    _lastErrorCode = result;
    return false;
  }

  return true; // Return true if successful

}

// Read torque as float
bool DYN200::readTorque(float& torque) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_TORQUE_VALUE, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    int32_t torqueRaw = combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    torque = float(torqueRaw) / 100.0; // Scale appropriately
    return true;
  }
  return false;
}

// Read torque as int32_t
bool DYN200::readTorque(int32_t& torque) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_TORQUE_VALUE, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    torque = combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    return true;
  }
  return false;
}

// Read torque as int16_t
bool DYN200::readTorque(int16_t& torque) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_TORQUE_VALUE, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    int32_t torqueRaw = combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    // Check if the value fits into a 16-bit signed integer
    if (torqueRaw < INT16_MIN || torqueRaw > INT16_MAX){
        _lastErrorCode = DYN200::dataOversize;
        return false; // Value does not fit
    }
    torque = static_cast<int16_t>(torqueRaw);
    return true;    // Return true if value fits
  }
  return false;
}

// Read RPM as float
bool DYN200::readRPM(float& rpm) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_SPEED, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    int32_t rpmRaw = combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    rpm = rpmRaw / 10.0; // Scale appropriately
    return true;    
  }
  return false;
}

// Read RPM as int32_t
bool DYN200::readRPM(uint32_t& rpm) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_SPEED, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    rpm = static_cast<int32_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    return true;
  }
  return false;
}

// Read RPM as int16_t
bool DYN200::readRPM(uint16_t& rpm) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_SPEED, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    uint32_t rpmRaw = static_cast<int32_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    // Check if the value fits into a 16-bit signed integer
    if (rpmRaw > UINT16_MAX){
        _lastErrorCode = DYN200::dataOversize;
        return false; // Value does not fit
    }
    rpm = static_cast<uint16_t>(rpmRaw);
    return true;    // Return true if value fits
  }
  return false;
}

// Read power as float
bool DYN200::readPower(float& power) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_POWER, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    power = combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    return true; // No scaling needed
  }
  return false;
}

// Read power as int32_t
bool DYN200::readPower(uint32_t& power) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_POWER, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    power = static_cast<uint32_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    return true; // No scaling needed
  }
  return false;
}

// Read power as int16_t
bool DYN200::readPower(uint16_t& power) {
  uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_POWER, 2);
  _lastErrorCode = result; // Store the error code
  if (result == _node.ku8MBSuccess) {
    uint32_t powerRaw = static_cast<uint32_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    if (powerRaw > UINT16_MAX){
        _lastErrorCode = DYN200::dataOversize;
        return false; // Value does not fit
    }
    power = static_cast<uint16_t>(powerRaw);
    return true; // No scaling needed
  }
  return false;
}

// Read digital filter value
uint8_t DYN200::readDigitalFilering(){
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_DIGITAL_FILTERING, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
    return static_cast<uint8_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Read radix point (Decimal point placement)
uint8_t DYN200::readRadixPoint(){
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_RADIX_POINT, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
    return static_cast<uint8_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Zero on boot?
bool DYN200::readBootZero(){
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_BOOT_ZERO, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
    return static_cast<bool>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Read full degree value (See manual for description)
uint16_t DYN200::readFullDegree(){
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_FULL_DEGREE, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
    return static_cast<uint16_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Torque reversed?
bool DYN200::readTorqueDirection(){
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_TORQUE_DIRECTION, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
    return static_cast<bool>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Read speed filter value
uint8_t DYN200::readSpeedFilter() { 
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_SPEED_FILTER, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
        return static_cast<uint8_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// Read speed decimal value
uint8_t DYN200::readSpeedDecimal() {
    uint8_t result = _node.readHoldingRegisters(DYN200::_ADDR_SPEED_DECIMAL, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
        return static_cast<uint8_t>(combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1)));
    }
    return NAN; // Return NaN if the read fails
}

// General read register
uint32_t DYN200::readRegister(uint8_t registerAddress){
    uint8_t result = _node.readHoldingRegisters(registerAddress, 2);
    _lastErrorCode = result; // Store the error code
    if (result == _node.ku8MBSuccess) {
        return combineWords(_node.getResponseBuffer(0), _node.getResponseBuffer(1));
    }
    return NAN; // Return NaN if the read fails
}

// Get config output
String DYN200::getConfig() {
    String config = F("DYN-200 Configuration\n");

    // Read and append each configuration value
    config += F("Machine ID: ");
    config += String(DYN200::_slaveID);
    config += F("\n");

    config += F("Baud Rate: ");
    config += String(DYN200::_baudRate);
    config += F("\n");

    uint8_t digitalFiltering = readDigitalFilering();
    config += F("Digital Filtering: ");
    config += String(digitalFiltering);
    config += F("\n");

    uint8_t radixPoint = readRadixPoint();
    config += F("Radix Point: ");
    config += String(radixPoint);
    config += F("\n");

    bool bootZero = readBootZero();
    config += F("Zero on Boot: ");
    config += (bootZero ? F("True") : F("False"));
    config += F("\n");

    uint16_t fullDegree = readFullDegree();
    config += F("Full Degree: ");
    config += String(fullDegree);
    config += F("\n");

    bool torqueDirection = readTorqueDirection();
    config += F("Torque Direction: ");
    config += (torqueDirection ? F("Default") : F("Reversed"));
    config += F("\n");

    uint8_t speedFilter = readSpeedFilter();
    config += F("Speed Filter: ");
    config += String(speedFilter);
    config += F("\n");

    uint8_t speedDecimal = readSpeedDecimal();
    config += F("Speed Decimal: ");
    config += String(speedDecimal);
    config += F("\n");

    return config;
}

// Write digital filter value
bool DYN200::writeDigitalFiltering(uint8_t value = 50) {
    if(value > 99 || value < 1){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_DIGITAL_FILTERING, static_cast<uint32_t>(value));
}

// Write radix point value
bool DYN200::writeRadixPoint(uint8_t value = 2) {
    if(value > 4 || value < 0){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_RADIX_POINT, static_cast<uint32_t>(value));
}

// Write boot zero
bool DYN200::writeBootZero(bool bootZero = true) {
    return writeRegister(DYN200::_ADDR_BOOT_ZERO, static_cast<uint32_t>(bootZero));
}

// Zero or set torque value
bool DYN200::writeSendZero(uint16_t torqueValue = 0) {
    if(torqueValue > 16384 || torqueValue < 0){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_SEND_ZERO, static_cast<uint32_t>(torqueValue));
}

// Write full degree value
bool DYN200::writeFullDegree(uint16_t value = 15599) {
    if(value > 16384 || value < 100){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_FULL_DEGREE, static_cast<uint32_t>(value));
}

// Write torque direction
bool DYN200::writeTorqueDirection(bool reversed = false) {
    return writeRegister(DYN200::_ADDR_TORQUE_DIRECTION, static_cast<bool>(reversed));
}

// Write communication baud rate - Limited set options
bool DYN200::writeBaudRate(uint32_t value = 19200) {
    // Set baud index directly if possible
    if(value>= 1 && value <= 6){
        return writeRegister(DYN200::_ADDR_BAUD_RATE, static_cast<uint32_t>(value));    
    }
    
    uint32_t index;
    switch (value){ // Check for valid baud
    case 9600:
        index = 1;
        break;
    case 14400:
        index = 2;
        break;
    case 19200:
        index = 3;
        break;
    case 38400:
        index = 4;
        break;
    case 57600:
        index = 5;
        break;
    case 115200:
        index = 6;
        break;
    default:
        _lastErrorCode = DYN200::invalidBaud;
        return false;
        break;
    }
    return writeRegister(DYN200::_ADDR_BAUD_RATE, index);   // Write baud index
}

// Write machine ID, Modbus Address 
bool DYN200::writeMachineID(uint8_t ID = 1) {
    if(ID > 120 || ID < 1){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_MACHINE_SLAVE_ID, static_cast<uint32_t>(ID));
}

// Write speed filter value
bool DYN200::writeSpeedFilter(uint8_t value = 50) {
    if(value > 99 || value < 0){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_SPEED_FILTER, static_cast<uint32_t>(value));
}

// Write speed decimal value
bool DYN200::writeSpeedDecimal(uint8_t value = 1) {
    if(value > 3 || value < 0){
        _lastErrorCode = DYN200::invalidWriteValue;
        return false;
    }
    return writeRegister(DYN200::_ADDR_SPEED_DECIMAL, static_cast<uint32_t>(value));
}


// General write register
bool DYN200::writeRegister(uint16_t registerAddress, uint32_t value) {
    // set word 0 of TX buffer to most-significant word of counter (bits 31..16)
    _node.setTransmitBuffer(0, highWord(value));
    // set word 1 of TX buffer to least-significant word of counter (bits 15..0)
    _node.setTransmitBuffer(1, lowWord(value));
    uint8_t result = _node.writeMultipleRegisters(registerAddress, 2);
    _lastErrorCode = result; // Store the result for debugging
    return result == _node.ku8MBSuccess;
}

// Get the last error as a descriptive string
String DYN200::getLastError() {
    switch (_lastErrorCode) {
      case ModbusMaster::ku8MBSuccess:
          return F("Success");
      case ModbusMaster::ku8MBIllegalFunction:
          return F("Illegal function");
      case ModbusMaster::ku8MBIllegalDataAddress:
          return F("Illegal data address");
      case ModbusMaster::ku8MBIllegalDataValue:
          return F("Illegal data value");
      case ModbusMaster::ku8MBSlaveDeviceFailure:
          return F("Slave device failure");
      case ModbusMaster::ku8MBInvalidSlaveID:
          return F("Invalid slave ID");
      case ModbusMaster::ku8MBInvalidFunction:
          return F("Invalid function");
      case ModbusMaster::ku8MBResponseTimedOut:
          return F("Response timed out");
      case ModbusMaster::ku8MBInvalidCRC:
          return F("Invalid CRC");
      case DYN200::dataOversize:
          return F("Data Oversize, Use float or int32_t method");
      case DYN200::invalidWriteValue:
          return F("Invalid write value");
      case DYN200::invalidBaud:
          return F("Invalid baudrate, ");
      default:
          return F("Unknown error");
    }
}

// Set torque value to zero
bool DYN200::setZero(){
    return writeRegister(0x00, 0xFF00);
}

// Reset all values to factory defaults
bool DYN200::factoryReset(){
    return writeRegister(0x02, 0xFF00);
}

// Private helper: Combine high and low words
int32_t DYN200::combineWords(int16_t high, uint16_t low) {
  return (int32_t(high) << 16) | low;
}

// Static methods for Modbus transmission
void DYN200::preTransmission() {
  digitalWrite(_MAX485ControlPin, HIGH);  // Set the control pin for transmission
}

void DYN200::postTransmission() {
  digitalWrite(_MAX485ControlPin, LOW);   // Set the control pin for reception
}
#ifndef DYN200_H
#define DYN200_H

#include <Arduino.h>
#include <ModbusMaster.h>

class DYN200 {
public:

    // Constructor
    DYN200(uint8_t slaveID, HardwareSerial& serial, uint8_t MAX485ControlPin);

    // Initialize the Modbus communication
    bool begin(uint32_t baudRate = 19200);

    // General Read method
    uint32_t readRegister(uint8_t registerAddress);

    // General write method
    bool writeRegister(uint16_t registerAddress, uint32_t value);

    // Read Methods
    // Read Torque / RPM / Power
    bool readTorque(float& torque);    // Read torque as float
    bool readTorque(int32_t& torque);  // Read torque as int32_t
    bool readTorque(int16_t& torque);  // Read torque as int16_t
    bool readRPM(float& rpm);          // Read RPM as float
    bool readRPM(uint32_t& rpm);       // Read RPM as int32_t
    bool readRPM(uint16_t& rpm);       // Read RPM as int16_t
    bool readPower(float& power);      // Read power as float
    bool readPower(uint32_t& power);   // Read power as int32_t
    bool readPower(uint16_t& power);   // Read power as int16_t
    // Read Config Data
    uint8_t readDigitalFilering();     // Read digital filter value
    uint8_t readRadixPoint();          // Read Radix Value (Torque decimal placement)
    bool readBootZero();               // Zero On Boot?
    uint16_t readFullDegree();         // Read full degree value
    bool readTorqueDirection();        // Torque direction reversed?
    uint8_t readSpeedFilter() ;        // Read speed filter value
    uint8_t readSpeedDecimal();        // Read speed decimal value

    // Write Config Data
    bool writeDigitalFiltering(uint8_t value = 50);
    bool writeRadixPoint(uint8_t value = 2);
    bool writeBootZero(bool boot_zero = true);
    bool writeSendZero(uint16_t torque_value = 0);
    bool writeFullDegree(uint16_t value = 15599);
    bool writeTorqueDirection(bool reversed = false);
    bool writeBaudRate(uint32_t value = 19200);
    bool writeMachineID(uint8_t ID = 1);
    bool writeSpeedFilter(uint8_t value = 50);
    bool writeSpeedDecimal(uint8_t value = 1);
    
    // Set zero torque
    bool setZero();
    
    // Reset factory defaults
    bool factoryReset();

    // Read all config data
    String getConfig();   // Returns large string containing all config values

    // Error handling
    String getLastError();  // Returns string of last error
    
    
private:
    // Private Values
    uint8_t _slaveID;        // Modbus Slave Address
    HardwareSerial& _serial; // Hardware serial port used by communication
    ModbusMaster _node;      // Modbus Master object for communication
    uint8_t _lastErrorCode;  // Last error code received
    uint32_t _baudRate = 19200; // Default baudrate

    // Constant Error Message Codes
    static const uint8_t dataOversize = 0xD1;
    static const uint8_t invalidWriteValue = 0xD2; 
    static const uint8_t invalidBaud = 0xD3; 

    // Register Addresses 
    const uint16_t _ADDR_TORQUE_VALUE      = 0x00;
    const uint16_t _ADDR_SPEED             = 0x02;
    const uint16_t _ADDR_POWER             = 0x04;
    const uint16_t _ADDR_DIGITAL_FILTERING = 0x06;
    const uint16_t _ADDR_RADIX_POINT       = 0x08;
    const uint16_t _ADDR_BOOT_ZERO         = 0x0A;
    const uint16_t _ADDR_SEND_ZERO         = 0x0C;
    const uint16_t _ADDR_FULL_DEGREE       = 0x0E;
    const uint16_t _ADDR_TORQUE_DIRECTION  = 0x12;
    const uint16_t _ADDR_BAUD_RATE         = 0x14;
    const uint16_t _ADDR_MACHINE_SLAVE_ID  = 0x16;
    const uint16_t _ADDR_SPEED_FILTER      = 0x1E;
    const uint16_t _ADDR_SPEED_DECIMAL     = 0x20;
    
    // Pre and Post transmission functions for MAX485
    static void preTransmission();
    static void postTransmission();

    // Helper function
    int32_t combineWords(int16_t high, uint16_t low);   // Combines low and high registers

    // Static pin for MAX485 control
    static uint8_t _MAX485ControlPin;

    // Static member for instance pointer
    static DYN200* _instance;

};  

#endif // DYN200_H

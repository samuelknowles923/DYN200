/**
 * @file DYN200.h
 * @brief Header file for DYN200 Modbus Communication Library
 *
 * @details This header defines the DYN200 class for interfacing with DYN200 series 
 * torque transducer using Modbus communication protocol. The class provides methods for 
 * reading and writing various device parameters, including torque, RPM, power, 
 * and device configuration.
 *
 * @author J. Deuter
 * @date March 3, 2025
 *
 * @version 1.0.0
 * 
 * @note Requires Arduino framework and ModbusMaster library
 *
 * @warning Ensure proper hardware connection and MAX485 transceiver setup
 *
 * @see https://github.com/J-Deuter/DYN200
 */

#ifndef DYN200_H
#define DYN200_H

#include <Arduino.h>
#include <ModbusMaster.h>

/**
 * @class DYN200
 * @brief A class for interfacing with DYN-200 series torque transducers via Modbus communication
 * @details This class provides comprehensive control and communication with DYN-200 series 
 * devices, supporting various read and write operations for torque, RPM, power, 
 * and device configuration.
 */
class DYN200 {
public:

    /**
     * @brief Constructor for the DYN200 class
     * 
     * @param slaveID Modbus slave address of the device
     * @param serial Hardware serial port for communication
     * @param MAX485ControlPin Control pin for MAX485 transceiver
     */
    DYN200(uint8_t slaveID, HardwareSerial& serial, uint8_t MAX485ControlPin);

    /**
     * @brief Initialize Modbus communication
     * 
     * @param baudRate Communication baud rate (default 19200)
     * @return bool True if initialization is successful, false otherwise
     */
    bool begin(uint32_t baudRate = 19200);

    /**
     * @brief Read a register value
     * 
     * @param registerAddress Address of the register to read
     * @return uint32_t Value read from the register
     */
    uint32_t readRegister(uint8_t registerAddress);

    /**
     * @brief Write a value to a register
     * 
     * @param registerAddress Address of the register to write
     * @param value Value to write to the register
     * @return bool True if write is successful, false otherwise
     */
    bool writeRegister(uint16_t registerAddress, uint32_t value);

    // ----- Torque Read Overloads ----- //

    /**
     * @brief Read torque as a float
     * @param torque Reference to store the torque value
     * @return bool True if read is successful
     */
    bool readTorque(float& torque);
    
    /**
     * @brief Read torque as a 32-bit integer
     * @param torque Reference to store the torque value
     * @return bool True if read is successful
     */
    bool readTorque(int32_t& torque);
    
    /**
     * @brief Read torque as a 16-bit integer
     * @param torque Reference to store the torque value
     * @return bool True if read is successful
     */
    bool readTorque(int16_t& torque);

    // ----- RPM Read Overloads ----- //

    /**
     * @brief Read RPM as a float
     * @param rpm Reference to store the RPM value
     * @return bool True if read is successful
     */
    bool readRPM(float& rpm);
    
    /**
     * @brief Read RPM as a 32-bit integer
     * @param rpm Reference to store the RPM value
     * @return bool True if read is successful
     */
    bool readRPM(uint32_t& rpm);
    
    /**
     * @brief Read RPM as a 16-bit integer
     * @param rpm Reference to store the RPM value
     * @return bool True if read is successful
     */
    bool readRPM(uint16_t& rpm);

    // ----- Power Read Overloads ----- //
    /**
     * @brief Read power as a float
     * @param power Reference to store the power value
     * @return bool True if read is successful
     */
    bool readPower(float& power);
    
    /**
     * @brief Read power as a 32-bit integer
     * @param power Reference to store the power value
     * @return bool True if read is successful
     */
    bool readPower(uint32_t& power);
    
    /**
     * @brief Read power as a 16-bit integer
     * @param power Reference to store the power value
     * @return bool True if read is successful
     */
    bool readPower(uint16_t& power);

    // ----- Configuration: Read Methods ----- //
    /**
     * @brief Read digital filter value
     * @return uint8_t Digital filter setting
     */
    uint8_t readDigitalFilering();
    
    /**
     * @brief Read radix point (decimal placement) for torque
     * @return uint8_t Radix point value
     */
    uint8_t readRadixPoint();
    
    /**
     * @brief Check if zero is set on boot
     * @return bool True if zero is set on boot
     */
    bool readBootZero();
    
    /**
     * @brief Read full degree value
     * @return uint16_t Full degree setting
     */
    uint16_t readFullDegree();
    
    /**
     * @brief Check if torque direction is reversed
     * @return bool True if torque direction is reversed
     */
    bool readTorqueDirection();
    
    /**
     * @brief Read speed filter value
     * @return uint8_t Speed filter setting
     */
    uint8_t readSpeedFilter();
    
    /**
     * @brief Read speed decimal value
     * @return uint8_t Speed decimal setting
     */
    uint8_t readSpeedDecimal();

    // ----- Configuration Write Methods ----- //

    /**
     * @brief Write digital filtering value
     * @param value Digital filtering setting (default 50)
     * @return bool True if write is successful
     */
    bool writeDigitalFiltering(uint8_t value = 50);
    
    /**
     * @brief Write radix point for torque
     * @param value Radix point setting (default 2)
     * @return bool True if write is successful
     */
    bool writeRadixPoint(uint8_t value = 2);
    
    /**
     * @brief Configure zero on boot
     * @param boot_zero Enable zero on boot (default true)
     * @return bool True if write is successful
     */
    bool writeBootZero(bool boot_zero = true);
    
    /**
     * @brief Send zero torque value
     * @param torque_value Zero torque value 
     * @return bool True if write is successful
     */
    bool writeSendZero(uint16_t torque_value);
    
    /**
     * @brief Write full degree value
     * @param value Full degree setting (default 15599)
     * @return bool True if write is successful
     */
    bool writeFullDegree(uint16_t value = 15599);
    
    /**
     * @brief Set torque direction
     * @param reversed Reverse torque direction (default false)
     * @return bool True if write is successful
     */
    bool writeTorqueDirection(bool reversed = false);
    
    /**
     * @brief Write baud rate
     * @param value Baud rate (default 19200)
     * @return bool True if write is successful
     * @note New value will take affect after power cycle, communication will need reestablished at new baud rate.
     */
    bool writeBaudRate(uint32_t value = 19200);
    
    /**
     * @brief Write machine/slave ID
     * @param ID Machine ID (default 1)
     * @return bool True if write is successful
     * @note New value will take affect after power cycle, communication will need reestablished at new slave ID.
     */
    bool writeMachineID(uint8_t ID = 1);
    
    /**
     * @brief Write speed filter value
     * @param value Speed filter setting (default 50)
     * @return bool True if write is successful
     */
    bool writeSpeedFilter(uint8_t value = 50);
    
    /**
     * @brief Write speed decimal value
     * @param value Speed decimal setting (default 1)
     * @return bool True if write is successful
     */
    bool writeSpeedDecimal(uint8_t value = 1);

    /**
     * @brief Set zero torque
     * @return bool True if zero setting is successful
     */
    bool setZero();
    
    /**
     * @brief Reset device to factory defaults
     * @return bool True if factory reset is successful
     */
    bool factoryReset();

    /**
     * @brief Retrieve full device configuration as a string
     * @return String containing all configuration values
     */
    String getConfig();

    /**
     * @brief Get the last error encountered
     * @return String describing the last error
     */
    String getLastError();
    
private:
    // Private member variables with brief descriptions
    uint8_t _slaveID;        ///< Modbus Slave Address
    HardwareSerial& _serial; ///< Hardware serial port used for communication
    ModbusMaster _node;      ///< Modbus Master object for communication
    uint8_t _lastErrorCode;  ///< Last error code received
    uint32_t _baudRate = 19200; ///< Default baudrate

    // Constant Error Message Codes
    /// Data oversize error code
    static const uint8_t dataOversize = 0xD1;
    /// Invalid write value error code
    static const uint8_t invalidWriteValue = 0xD2; 
    /// Invalid baud rate error code
    static const uint8_t invalidBaud = 0xD3; 

    // Register Addresses 
    // Const register address definitions with brief descriptions
    const uint16_t _ADDR_TORQUE_VALUE      = 0x00; ///< Torque value register address
    const uint16_t _ADDR_SPEED             = 0x02; ///< Speed register address
    const uint16_t _ADDR_POWER             = 0x04; ///< Power register address
    const uint16_t _ADDR_DIGITAL_FILTERING = 0x06; ///< Digital filtering register address
    const uint16_t _ADDR_RADIX_POINT       = 0x08; ///< Radix point register address
    const uint16_t _ADDR_BOOT_ZERO         = 0x0A; ///< Boot zero register address
    const uint16_t _ADDR_SEND_ZERO         = 0x0C; ///< Send zero register address
    const uint16_t _ADDR_FULL_DEGREE       = 0x0E; ///< Full degree register address
    const uint16_t _ADDR_TORQUE_DIRECTION  = 0x12; ///< Torque direction register address
    const uint16_t _ADDR_BAUD_RATE         = 0x14; ///< Baud rate register address
    const uint16_t _ADDR_MACHINE_SLAVE_ID  = 0x16; ///< Machine/Slave ID register address
    const uint16_t _ADDR_SPEED_FILTER      = 0x1E; ///< Speed filter register address
    const uint16_t _ADDR_SPEED_DECIMAL     = 0x20; ///< Speed decimal register address
    
    /**
     * @brief Pre-transmission function for MAX485 control
     */
    static void preTransmission();
    
    /**
     * @brief Post-transmission function for MAX485 control
     */
    static void postTransmission();

    /**
     * @brief Combine high and low words into a 32-bit integer
     * @param high High 16-bit word
     * @param low Low 16-bit word
     * @return int32_t Combined 32-bit integer
     */
    int32_t combineWords(int16_t high, uint16_t low);

    // Static pin for MAX485 control
    static uint8_t _MAX485ControlPin; ///< Control pin for MAX485 transceiver

    // Static member for instance pointer
    static DYN200* _instance; ///< Pointer to class instance
};  

#endif // DYN200_H
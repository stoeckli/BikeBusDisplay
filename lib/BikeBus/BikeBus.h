#ifndef BIKEBUS_H
#define BIKEBUS_H

#include <Arduino.h>

// BikeBus Addresses
#define BIKEBUS_ADDR_OPERATOR_PANEL         1
#define BIKEBUS_ADDR_OPERATOR_PANEL_SLAVE   2
#define BIKEBUS_ADDR_MOTOR                  16
#define BIKEBUS_ADDR_BRAKE                  24
#define BIKEBUS_ADDR_BATTERY                32
#define BIKEBUS_ADDR_BATTERY_OPTIONAL       33
#define BIKEBUS_ADDR_LIGHT                  48
#define BIKEBUS_ADDR_SERVICE_TOOL           240

// BikeBus Tokens (GOD Display Slave Mode - Address 2)
#define TOKEN_AUTO_OFF_TIME         68
#define TOKEN_AUTO_OFF_TIME_WRITE   69
#define TOKEN_IMPERIAL              70
#define TOKEN_IMPERIAL_WRITE        71
#define TOKEN_WHEEL_CIRCUMFERENCE   72
#define TOKEN_WHEEL_CIRCUMFERENCE_WRITE 73
#define TOKEN_EBIKE_DEMAND          74
#define TOKEN_EBIKE_DEMAND_WRITE    75
#define TOKEN_BRAKE_DEMAND          76
#define TOKEN_BRAKE_DEMAND_WRITE    77
#define TOKEN_LED_BRIGHTNESS        80
#define TOKEN_LED_BRIGHTNESS_WRITE  81
#define TOKEN_TOTAL_MILES           132
#define TOKEN_COUNTER               134
#define TOKEN_TOTAL_KM              136
#define TOKEN_TOTAL_TIME            140
#define TOKEN_TOTAL_ENERGY          144
#define TOKEN_MOTOR_ERR_BITS        196
#define TOKEN_MOTOR_ERR_CODE        198
#define TOKEN_BATTERY_ERRORS        200
#define TOKEN_BATTERY_SAFETY_STATUS 202

// BikeBus Timing
#define BIKEBUS_MASTER_INTERVAL     20  // ms - Master sends every 20ms when client responds
#define BIKEBUS_RETRY_INTERVAL      30  // ms - Retry after 30ms if no response
#define BIKEBUS_RESPONSE_TIMEOUT    10  // ms - Client responds 10ms after master telegram
#define BIKEBUS_MOTOR_TIMEOUT       100 // ms - Motor goes to safe state if no MainMotorControl

// BikeBus Error Codes (GOD Display)
#define ERROR_TXERRORS              0x20  // Transmission error
#define ERROR_TXBUSY                0x21  // Bus busy
#define ERROR_TXBYTETIMEOUT         0x22  // Transmission time exceeded
#define ERROR_WRONGADDRECHO         0x23  // Wrong address responds
#define ERROR_WRONGTOKENECHO        0x24  // Wrong token responds
#define ERROR_WRONGVALLOECHO        0x25  // Wrong response low byte value
#define ERROR_WRONGVALHIECHO        0x26  // Wrong response high byte value

// Telegram structure
struct BikeBusTelegram {
    uint8_t address;
    uint8_t token;
    uint8_t valueLow;
    uint8_t valueHigh;
    uint8_t checksum;
};

class BikeBus {
public:
    BikeBus(HardwareSerial &serial);
    
    // Initialize BikeBus communication
    void begin(uint32_t baudRate = 9600);
    
    // Send a telegram to a slave device
    bool sendTelegram(uint8_t slaveAddr, uint8_t token, uint16_t value = 0);
    
    // Receive response from slave device
    bool receiveResponse(BikeBusTelegram &telegram, uint32_t timeout = BIKEBUS_RESPONSE_TIMEOUT);
    
    // Read a value from a slave device
    bool readValue(uint8_t slaveAddr, uint8_t token, uint16_t &value);
    
    // Write a value to a slave device
    bool writeValue(uint8_t slaveAddr, uint8_t token, uint16_t value);
    
    // Calculate checksum for a telegram
    static uint8_t calculateChecksum(const BikeBusTelegram &telegram);
    
    // Validate received telegram
    bool validateTelegram(const BikeBusTelegram &telegram);
    
    // Get last error code
    uint8_t getLastError() const { return _lastError; }
    
    // Clear error
    void clearError() { _lastError = 0; }
    
    // Check if bus is available
    bool isBusAvailable();
    
private:
    HardwareSerial &_serial;
    uint8_t _lastError;
    uint32_t _lastSendTime;
    
    // Send raw bytes
    void sendRawTelegram(const BikeBusTelegram &telegram);
    
    // Receive raw bytes
    bool receiveRawTelegram(BikeBusTelegram &telegram, uint32_t timeout);
};

#endif // BIKEBUS_H

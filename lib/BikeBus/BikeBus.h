/**
 * @file BikeBus.h
 * @brief BikeBus v1.8 Protocol Library - Header File
 * 
 * This header file defines the BikeBus class and protocol constants for communicating
 * with BikeBus v1.8 compatible e-bike systems. The library provides a comprehensive
 * interface for controlling motors, monitoring batteries, and managing lighting systems
 * over a half-duplex UART connection.
 * 
 * Protocol Features:
 * - Motor control (5 support levels, 3 regeneration levels, push assist)
 * - Battery monitoring (voltage, current, SOC, temperature)
 * - Light control (on/off, high beam, brake lights)
 * - Automatic telegram sequencing with 20ms timing
 * - Checksum validation for data integrity
 * - Debug output for protocol analysis
 * 
 * @author Markus Stoeckli
 * @email support@stoeckli.net
 * @date January 5, 2026
 * @version 1.0.0
 * 
 * @copyright Copyright (C) 2026 Markus Stoeckli
 * 
 * @license GNU General Public License v3.0
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BIKEBUS_H
#define BIKEBUS_H

#include <Arduino.h>

// ============================================================================
// BikeBus Protocol Constants
// Based on BikeBus-v1.8 specification
// ============================================================================

// BikeBus Addresses
#define BIKEBUS_ADDR_MASTER         1
#define BIKEBUS_ADDR_DISPLAY_SLAVE  2
#define BIKEBUS_ADDR_MOTOR          16
#define BIKEBUS_ADDR_BRAKE          24
#define BIKEBUS_ADDR_BATTERY1       32
#define BIKEBUS_ADDR_BATTERY2       33
#define BIKEBUS_ADDR_LIGHT          48
#define BIKEBUS_ADDR_SERVICE_TOOL   240

// Motor Tokens
#define MOTOR_TOKEN_ERROR_BITS      2
#define MOTOR_TOKEN_MAIN_CONTROL    3
#define MOTOR_TOKEN_REVOLUTIONS     4
#define MOTOR_TOKEN_RESET_REV       5
#define MOTOR_TOKEN_FW_VERSION      6
#define MOTOR_TOKEN_ERROR_CODE      8
#define MOTOR_TOKEN_ERROR_BITS_ALT  10
#define MOTOR_TOKEN_EXT_ERRORS      11
#define MOTOR_TOKEN_TORQUE_SENSOR   12
#define MOTOR_TOKEN_SPEED           14
#define MOTOR_TOKEN_TEMPERATURE     16
#define MOTOR_TOKEN_SERIAL_LOW      28
#define MOTOR_TOKEN_SERIAL_HIGH     30
#define MOTOR_TOKEN_CONFIG_WORD_R   32
#define MOTOR_TOKEN_CONFIG_WORD_W   33
#define MOTOR_TOKEN_MILEAGE_R       60
#define MOTOR_TOKEN_MILEAGE_W       61
#define MOTOR_TOKEN_CURRENT_LIMIT_R 68
#define MOTOR_TOKEN_CURRENT_LIMIT_W 69

// Battery Tokens (SMBus-based: Token = 2 * (SMBus + 1) + ACTION)
#define BATTERY_TOKEN_MANUF_ACCESS  2
#define BATTERY_TOKEN_MODE          8
#define BATTERY_TOKEN_TEMP          18
#define BATTERY_TOKEN_VOLTAGE       20
#define BATTERY_TOKEN_CURRENT       22
#define BATTERY_TOKEN_AVG_CURRENT   24
#define BATTERY_TOKEN_MAX_ERROR     26
#define BATTERY_TOKEN_SOC           28
#define BATTERY_TOKEN_REMAINING_CAP 32
#define BATTERY_TOKEN_TIME_TO_EMPTY 36
#define BATTERY_TOKEN_AVG_TIME_EMPTY 38
#define BATTERY_TOKEN_AVG_TIME_FULL 40
#define BATTERY_TOKEN_CURRENT_LIMIT_R 44
#define BATTERY_TOKEN_STATUS        46
#define BATTERY_TOKEN_CYCLE_COUNT   48
#define BATTERY_TOKEN_DESIGN_CAP    50
#define BATTERY_TOKEN_DESIGN_VOLT   52
#define BATTERY_TOKEN_MANUF_DATE    56
#define BATTERY_TOKEN_SERIAL        58
#define BATTERY_TOKEN_CELL1         122
#define BATTERY_TOKEN_CELL2         124
#define BATTERY_TOKEN_CELL3         126
#define BATTERY_TOKEN_CELL4         128
#define BATTERY_TOKEN_CELL5         130
#define BATTERY_TOKEN_CELL6         132
#define BATTERY_TOKEN_CELL7         134
#define BATTERY_TOKEN_CELL8         136
#define BATTERY_TOKEN_CELL9         138
#define BATTERY_TOKEN_CELL10        140
#define BATTERY_TOKEN_EXTENDED_DATA 164
#define BATTERY_TOKEN_FLAGS_R       200
#define BATTERY_TOKEN_FLAGS_W       201

// Light Tokens
#define LIGHT_TOKEN_STATUS_R        2
#define LIGHT_TOKEN_CONTROL_W       3

// Light Control Values
#define LIGHT_OFF                   0x00
#define LIGHT_DRIVE                 0x01
#define LIGHT_HIGH_BEAM             0x04
#define LIGHT_BRAKE                 0x08

// Motor Control Values (Main Motor Control)
#define MMC_REGEN_3                 0x0500  // Regeneration level -3
#define MMC_REGEN_2                 0x0600  // Regeneration level -2
#define MMC_REGEN_1                 0x0700  // Regeneration level -1
#define MMC_NEUTRAL                 0x0800  // Neutral
#define MMC_LEVEL_1                 0x0900  // Support level 1
#define MMC_LEVEL_2                 0x0A00  // Support level 2
#define MMC_LEVEL_3                 0x0B00  // Support level 3
#define MMC_LEVEL_4                 0x0C00  // Support level 4
#define MMC_LEVEL_5                 0x0D00  // Support level 5
#define MMC_PUSH_ASSIST             0x0899  // Push assist (Schiebehilfe)

// Config Word bits
#define CONFIG_PEDELEC_MODE         0x02
#define CONFIG_EBIKE_MODE           0x00

// Timing constants (in milliseconds)
#define BIKEBUS_TX_INTERVAL         20
#define BIKEBUS_TX_RETRY_INTERVAL   30
#define BIKEBUS_RX_TIMEOUT          25
#define MOTOR_SAFETY_TIMEOUT        100

// ============================================================================
// BikeBus Communication Class
// ============================================================================

class BikeBus {
private:
    HardwareSerial* serial;
    uint8_t txBuffer[5];
    uint8_t rxBuffer[5];
    unsigned long lastTxTime;
    unsigned long lastMotorControlTime;
    bool waitingForResponse;
    uint8_t currentAddress;
    uint8_t currentToken;
    bool debugEnabled;
    uint16_t wheelCircumference;
    
    // Communication tracking
    unsigned long lastSuccessfulRx;  // Last time we received a valid response
    
    // Telegram sequence state
    int sequenceIndex;
    
    // Motor state
    uint16_t motorControlValue;
    int16_t motorSpeed;           // in RPM (can be negative for reverse)
    uint16_t motorRevolutions;
    uint16_t motorTemperature;    // in Kelvin
    uint16_t motorErrorBits;
    uint8_t motorErrorCode;
    uint16_t motorCurrentLimit;   // in Amperes
    bool motorConfigured;
    
    // Battery state
    uint16_t batteryVoltage;      // in mV
    int16_t batteryCurrent;       // in mA
    int16_t batteryAvgCurrent;    // in mA
    uint8_t batterySOC;           // State of Charge in %
    uint16_t batteryTemp;         // in 0.1K
    uint16_t batteryStatus;
    uint16_t batteryTimeToEmpty;  // in minutes
    
    // Light state
    uint8_t lightStatus;
    
    // Calculate checksum
    uint8_t calculateChecksum(uint8_t* data);
    
    // Send telegram
    bool sendTelegram(uint8_t address, uint8_t token, uint16_t value);
    
    // Receive telegram
    bool receiveTelegram(unsigned long timeout);
    
    // Process received response
    void processResponse();
    
public:
    BikeBus(HardwareSerial* ser, uint16_t wheelCircumferenceMm = 2222);
    
    void begin(unsigned long baud = 9600);
    void setDebug(bool enable);
    void setWheelCircumference(uint16_t circumferenceMm);
    
    // Initialize motor (set to Pedelec mode)
    void initializeMotor();
    
    // Query motor current limit (read immediately)
    void queryMotorCurrentLimit();
    
    // Query motor speed (read immediately)
    void queryMotorSpeed();

    // Set motor current limit
    void setMotorCurrentLimit(uint16_t currentA);

    // Send motor control telegram immediately
    void sendMotorControl();
    
    // Set motor control value (support level, regeneration, or push assist)
    void setMotorControl(uint16_t value);
    
    // Set support level (1-5)
    void setSupportLevel(uint8_t level);
    
    // Set regeneration level (1-3)
    void setRegenerationLevel(uint8_t level);
    
    // Enable/disable push assist
    void setPushAssist(bool enable);
    
    // Set current limit (in Amperes)
    void setCurrentLimit(uint16_t currentA);
    
    // Control light
    void setLight(uint8_t mode);
    
    // Main update function - executes telegram sequence
    void update();
    
    // Bus monitor function - listens to all bus traffic and reports to Serial
    void busMonitor();
    
    // Communication status
    unsigned long getLastSuccessfulRxTime();  // Get timestamp of last successful reception
    bool isBusActive(unsigned long timeoutMs = 5000);  // Check if bus has been active recently
    
    // Motor getters
    int16_t getMotorSpeed();              // RPM (can be negative)
    float getSpeedKmh();                  // Speed in km/h
    uint16_t getMotorRevolutions();       // Total revolutions
    uint16_t getMotorTemperature();       // Temperature in Kelvin
    float getMotorTempC();                // Temperature in Celsius
    uint16_t getMotorErrorBits();         // Error bits
    uint8_t getMotorErrorCode();          // Error code
    uint16_t getMotorCurrentLimit();      // Max current in Amperes
    
    // Battery getters
    uint16_t getBatteryVoltage();         // Voltage in mV
    float getBatteryVoltageV();           // Voltage in V
    int16_t getBatteryCurrent();          // Current in mA
    float getBatteryCurrentA();           // Current in A
    uint8_t getBatterySOC();              // State of Charge (0-100%)
    uint16_t getBatteryTemp();            // Temperature in 0.1K
    float getBatteryTempC();              // Temperature in Celsius
    uint16_t getBatteryStatus();          // Status word
    uint16_t getBatteryTimeToEmpty();     // Time to empty in minutes
};

#endif // BIKEBUS_H

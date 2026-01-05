# BikeBus Library

A comprehensive Arduino library for communicating with BikeBus v1.8 compatible e-bike systems.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Hardware Setup](#hardware-setup)
- [API Reference](#api-reference)
  - [Constructor](#constructor)
  - [Initialization Methods](#initialization-methods)
  - [Motor Control](#motor-control)
  - [Light Control](#light-control)
  - [Data Retrieval](#data-retrieval)
- [Protocol Constants](#protocol-constants)
- [Examples](#examples)
- [Protocol Details](#protocol-details)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

The BikeBus library implements the BikeBus v1.8 protocol, enabling communication with e-bike motor controllers, battery management systems, and lighting systems. It provides a clean, high-level API for controlling and monitoring e-bike components over a half-duplex UART connection.

## Features

- ✅ **Complete BikeBus v1.8 protocol implementation**
- ✅ **Motor control**: 5 support levels, 3 regeneration levels, push assist mode
- ✅ **Battery monitoring**: Voltage, current, state of charge (SOC), temperature
- ✅ **Light control**: On/off, high beam, brake lights
- ✅ **Automatic telegram sequencing** with 20ms timing
- ✅ **Built-in debug output** for protocol analysis
- ✅ **Configurable wheel circumference** for accurate speed calculation
- ✅ **Half-duplex bus support** with automatic echo cancellation
- ✅ **Checksum validation** for data integrity
- ✅ **Automatic retry** on communication timeout

## Installation

This library is designed for PlatformIO projects.

### For PlatformIO

The library is located in the project's `lib/` directory and will be automatically detected. No additional configuration is needed in `platformio.ini`.

### For Arduino IDE

Copy the `BikeBus` folder to your Arduino `libraries` directory (typically `Documents/Arduino/libraries/`).

## Quick Start

```cpp
#include <Arduino.h>
#include <BikeBus.h>

// Create BikeBus instance on Serial2 with 2222mm wheel circumference
BikeBus bikeBus(&Serial2, 2222);

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    
    // Initialize BikeBus at 9600 baud
    bikeBus.begin(9600);
    bikeBus.setDebug(true);  // Enable debug output
    
    // Initialize motor (sets Pedelec mode)
    bikeBus.initializeMotor();
    
    // Set initial support level
    bikeBus.setSupportLevel(3);
}

void loop() {
    // Update BikeBus communication (call every loop)
    bikeBus.update();
    
    // Read and display data
    float speed = bikeBus.getSpeedKmh();
    uint8_t soc = bikeBus.getBatterySOC();
    float voltage = bikeBus.getBatteryVoltageV();
    
    Serial.printf("Speed: %.1f km/h, Battery: %d%%, Voltage: %.1fV\n", 
                  speed, soc, voltage);
    
    delay(200);
}
```

## Hardware Setup

### Wiring

The BikeBus protocol uses a half-duplex UART connection at 9600 baud, 8N1.

**M5Stack Core2 Example:**
- TX Pin: GPIO 14 (Serial2)
- RX Pin: GPIO 13 (Serial2)
- GND: Common ground with BikeBus system

**Important Notes:**
- The BikeBus uses half-duplex communication on a single wire
- A 1kΩ resistor may be needed between TX and RX for proper half-duplex operation
- Ensure proper voltage levels (typically 5V or 3.3V depending on your system)

### Timing Requirements

- **Update Frequency**: Call `bikeBus.update()` in your main loop
- **Telegram Interval**: 20ms (handled automatically)
- **Response Timeout**: 25ms (handled automatically)
- **Retry Interval**: 30ms (handled automatically)

## API Reference

### Constructor

#### `BikeBus(HardwareSerial* ser, uint16_t wheelCircumferenceMm = 2222)`

Creates a new BikeBus instance.

**Parameters:**
- `ser`: Pointer to a HardwareSerial object (e.g., `&Serial2`)
- `wheelCircumferenceMm`: Wheel circumference in millimeters (default: 2222)

**Example:**
```cpp
BikeBus bikeBus(&Serial2, 2222);  // 28" wheel ≈ 2222mm circumference
```

### Initialization Methods

#### `void begin(unsigned long baud = 9600)`

Initializes the serial communication.

**Parameters:**
- `baud`: Baud rate (default: 9600, as per BikeBus spec)

**Example:**
```cpp
bikeBus.begin(9600);
```

#### `void setDebug(bool enable)`

Enables or disables debug output to Serial.

**Parameters:**
- `enable`: `true` to enable debug output, `false` to disable

**Debug Output Includes:**
- TX: Transmitted telegrams in hex format
- RX: Received telegrams in hex format
- Checksum errors with expected vs received values
- Timeout information

**Example:**
```cpp
bikeBus.setDebug(true);  // Enable debug output
```

#### `void setWheelCircumference(uint16_t circumferenceMm)`

Sets the wheel circumference for speed calculation.

**Parameters:**
- `circumferenceMm`: Wheel circumference in millimeters

**Common Wheel Sizes:**
- 26" wheel: ~2070 mm
- 27.5" wheel: ~2150 mm
- 28" wheel: ~2222 mm
- 29" wheel: ~2300 mm

**Example:**
```cpp
bikeBus.setWheelCircumference(2150);  // 27.5" wheel
```

#### `void initializeMotor()`

Initializes the motor controller by setting it to Pedelec mode. **Must be called once during setup.**

**Example:**
```cpp
bikeBus.initializeMotor();
```

### Motor Control

#### `void setSupportLevel(uint8_t level)`

Sets the motor support level (power assistance).

**Parameters:**
- `level`: Support level from 1 (lowest) to 5 (highest)

**Example:**
```cpp
bikeBus.setSupportLevel(3);  // Set to medium assistance
```

#### `void setRegenerationLevel(uint8_t level)`

Sets the regenerative braking level.

**Parameters:**
- `level`: Regeneration level from 1 (gentle) to 3 (aggressive)

**Example:**
```cpp
bikeBus.setRegenerationLevel(2);  // Medium regeneration
```

#### `void setPushAssist(bool enable)`

Enables or disables push assist mode (Schiebehilfe).

**Parameters:**
- `enable`: `true` to enable push assist, `false` to disable

**Important Notes:**
- Switches motor to eBike mode when enabled
- Sends neutral command 8 times when disabled (as per spec)
- Automatically switches back to Pedelec mode when disabled

**Example:**
```cpp
bikeBus.setPushAssist(true);   // Enable push assist
delay(5000);                   // Push for 5 seconds
bikeBus.setPushAssist(false);  // Disable push assist
```

#### `void setMotorControl(uint16_t value)`

Sets the raw motor control value. **Advanced users only.**

**Parameters:**
- `value`: Raw MMC (Main Motor Control) value

**Available Constants:**
- `MMC_REGEN_3` (0x0500): Regeneration level -3
- `MMC_REGEN_2` (0x0600): Regeneration level -2
- `MMC_REGEN_1` (0x0700): Regeneration level -1
- `MMC_NEUTRAL` (0x0800): Neutral
- `MMC_LEVEL_1` to `MMC_LEVEL_5` (0x0900-0x0D00): Support levels
- `MMC_PUSH_ASSIST` (0x0899): Push assist mode

**Example:**
```cpp
bikeBus.setMotorControl(MMC_LEVEL_4);  // Same as setSupportLevel(4)
```

#### `void setCurrentLimit(uint16_t currentA)`

Sets the battery current limit.

**Parameters:**
- `currentA`: Current limit in Amperes

**Example:**
```cpp
bikeBus.setCurrentLimit(20);  // Limit to 20A
```

### Light Control

#### `void setLight(uint8_t mode)`

Controls the e-bike lights.

**Parameters:**
- `mode`: Light mode (can be combined with bitwise OR)

**Available Constants:**
- `LIGHT_OFF` (0x00): All lights off
- `LIGHT_DRIVE` (0x01): Normal driving lights
- `LIGHT_HIGH_BEAM` (0x04): High beam
- `LIGHT_BRAKE` (0x08): Brake lights

**Examples:**
```cpp
bikeBus.setLight(LIGHT_OFF);                          // Turn off
bikeBus.setLight(LIGHT_DRIVE);                        // Normal lights
bikeBus.setLight(LIGHT_DRIVE | LIGHT_HIGH_BEAM);     // High beam on
bikeBus.setLight(LIGHT_DRIVE | LIGHT_BRAKE);         // Braking
```

### Data Retrieval

#### Motor Data

##### `int16_t getMotorSpeed()`

Returns motor speed in RPM (can be negative for reverse).

**Returns:** Motor speed in revolutions per minute

**Example:**
```cpp
int16_t rpm = bikeBus.getMotorSpeed();
```

##### `float getSpeedKmh()`

Returns vehicle speed in km/h, calculated from motor RPM and wheel circumference.

**Returns:** Speed in kilometers per hour

**Example:**
```cpp
float speed = bikeBus.getSpeedKmh();
Serial.printf("Speed: %.1f km/h\n", speed);
```

##### `uint16_t getMotorRevolutions()`

Returns total motor revolutions since last reset.

**Returns:** Total revolution count

**Example:**
```cpp
uint16_t revs = bikeBus.getMotorRevolutions();
```

##### `uint16_t getMotorTemperature()`

Returns motor temperature in Kelvin.

**Returns:** Temperature in Kelvin

**Example:**
```cpp
uint16_t tempK = bikeBus.getMotorTemperature();
```

##### `float getMotorTempC()`

Returns motor temperature in Celsius.

**Returns:** Temperature in degrees Celsius

**Example:**
```cpp
float tempC = bikeBus.getMotorTempC();
Serial.printf("Motor temp: %.1f°C\n", tempC);
```

##### `uint16_t getMotorErrorBits()`

Returns motor error status bits.

**Returns:** 16-bit error status word

**Example:**
```cpp
uint16_t errors = bikeBus.getMotorErrorBits();
if (errors != 0) {
    Serial.printf("Motor error: 0x%04X\n", errors);
}
```

##### `uint8_t getMotorErrorCode()`

Returns specific motor error code.

**Returns:** 8-bit error code

**Example:**
```cpp
uint8_t errorCode = bikeBus.getMotorErrorCode();
```

#### Battery Data

##### `uint16_t getBatteryVoltage()`

Returns battery voltage in millivolts.

**Returns:** Voltage in mV

**Example:**
```cpp
uint16_t voltage_mV = bikeBus.getBatteryVoltage();
```

##### `float getBatteryVoltageV()`

Returns battery voltage in volts.

**Returns:** Voltage in V

**Example:**
```cpp
float voltage = bikeBus.getBatteryVoltageV();
Serial.printf("Battery: %.1fV\n", voltage);
```

##### `int16_t getBatteryCurrent()`

Returns battery current in milliamperes (positive = discharging, negative = charging).

**Returns:** Current in mA

**Example:**
```cpp
int16_t current_mA = bikeBus.getBatteryCurrent();
```

##### `float getBatteryCurrentA()`

Returns battery current in amperes.

**Returns:** Current in A

**Example:**
```cpp
float current = bikeBus.getBatteryCurrentA();
Serial.printf("Current: %.2fA\n", current);
```

##### `uint8_t getBatterySOC()`

Returns battery state of charge as a percentage.

**Returns:** SOC from 0 to 100%

**Example:**
```cpp
uint8_t soc = bikeBus.getBatterySOC();
Serial.printf("Battery: %d%%\n", soc);
```

##### `uint16_t getBatteryTemp()`

Returns battery temperature in 0.1 Kelvin units.

**Returns:** Temperature in 0.1K

**Example:**
```cpp
uint16_t temp = bikeBus.getBatteryTemp();
```

##### `float getBatteryTempC()`

Returns battery temperature in Celsius.

**Returns:** Temperature in degrees Celsius

**Example:**
```cpp
float tempC = bikeBus.getBatteryTempC();
Serial.printf("Battery temp: %.1f°C\n", tempC);
```

##### `uint16_t getBatteryStatus()`

Returns battery status word (SMBus battery status).

**Returns:** 16-bit status word

**Example:**
```cpp
uint16_t status = bikeBus.getBatteryStatus();
```

##### `uint16_t getBatteryTimeToEmpty()`

Returns estimated time until battery is empty.

**Returns:** Time in minutes

**Example:**
```cpp
uint16_t timeLeft = bikeBus.getBatteryTimeToEmpty();
Serial.printf("Time remaining: %d minutes\n", timeLeft);
```

#### Communication Status

##### `unsigned long getLastSuccessfulRxTime()`

Returns the timestamp (in milliseconds) of the last successful telegram reception.

**Returns:** Timestamp in milliseconds (from `millis()`)

**Example:**
```cpp
unsigned long lastRx = bikeBus.getLastSuccessfulRxTime();
Serial.printf("Last RX: %lu ms ago\n", millis() - lastRx);
```

##### `bool isBusActive(unsigned long timeoutMs = 5000)`

Checks if the bus has been active (received responses) within the specified timeout period.

**Parameters:**
- `timeoutMs`: Timeout in milliseconds (default: 5000ms / 5 seconds)

**Returns:** `true` if bus responded within timeout, `false` otherwise

**Example:**
```cpp
if (bikeBus.isBusActive(10000)) {
    Serial.println("Bus is active");
} else {
    Serial.println("Bus timeout - no response");
}
```

### Main Loop

#### `void update()`

Executes the BikeBus telegram sequence. **Must be called regularly in the main loop.**

This method:
- Manages the 24-step telegram sequence
- Sends motor control commands
- Queries battery and motor data
- Handles timeouts and retries
- Maintains 20ms telegram timing

**Example:**
```cpp
void loop() {
    bikeBus.update();  // Call every loop iteration
    // Your other code here
}
```

## Protocol Constants

### Device Addresses

```cpp
BIKEBUS_ADDR_MASTER         // 1   - Master controller
BIKEBUS_ADDR_DISPLAY_SLAVE  // 2   - Display slave
BIKEBUS_ADDR_MOTOR          // 16  - Motor controller
BIKEBUS_ADDR_BRAKE          // 24  - Brake system
BIKEBUS_ADDR_BATTERY1       // 32  - Primary battery
BIKEBUS_ADDR_BATTERY2       // 33  - Secondary battery
BIKEBUS_ADDR_LIGHT          // 48  - Light system
BIKEBUS_ADDR_SERVICE_TOOL   // 240 - Service/diagnostic tool
```

### Motor Tokens

```cpp
MOTOR_TOKEN_ERROR_BITS      // 2  - Error status bits
MOTOR_TOKEN_MAIN_CONTROL    // 3  - Main motor control (MMC)
MOTOR_TOKEN_REVOLUTIONS     // 4  - Total revolutions
MOTOR_TOKEN_SPEED           // 14 - Motor speed (RPM)
MOTOR_TOKEN_TEMPERATURE     // 16 - Motor temperature
MOTOR_TOKEN_CONFIG_WORD_R   // 32 - Read configuration
MOTOR_TOKEN_CONFIG_WORD_W   // 33 - Write configuration
```

### Battery Tokens (SMBus-based)

```cpp
BATTERY_TOKEN_TEMP          // 18 - Temperature
BATTERY_TOKEN_VOLTAGE       // 20 - Voltage
BATTERY_TOKEN_CURRENT       // 22 - Current
BATTERY_TOKEN_AVG_CURRENT   // 24 - Average current
BATTERY_TOKEN_SOC           // 28 - State of charge
BATTERY_TOKEN_STATUS        // 46 - Status word
```

### Timing Constants

```cpp
BIKEBUS_TX_INTERVAL         // 20ms - Telegram send interval
BIKEBUS_TX_RETRY_INTERVAL   // 30ms - Retry after timeout
BIKEBUS_RX_TIMEOUT          // 25ms - Response timeout
MOTOR_SAFETY_TIMEOUT        // 100ms - Motor safety cutoff
```

## Examples

### Example 1: Basic Speed Display

```cpp
#include <BikeBus.h>

BikeBus bikeBus(&Serial2, 2222);

void setup() {
    Serial.begin(115200);
    bikeBus.begin(9600);
    bikeBus.initializeMotor();
    bikeBus.setSupportLevel(3);
}

void loop() {
    bikeBus.update();
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        float speed = bikeBus.getSpeedKmh();
        Serial.printf("Speed: %.1f km/h\n", speed);
    }
}
```

### Example 2: Battery Monitor

```cpp
void loop() {
    bikeBus.update();
    
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 1000) {
        lastUpdate = millis();
        
        uint8_t soc = bikeBus.getBatterySOC();
        float voltage = bikeBus.getBatteryVoltageV();
        float current = bikeBus.getBatteryCurrentA();
        float temp = bikeBus.getBatteryTempC();
        
        Serial.printf("Battery: %d%% | %.1fV | %.2fA | %.1f°C\n", 
                      soc, voltage, current, temp);
    }
}
```

### Example 3: Support Level Control

```cpp
void loop() {
    bikeBus.update();
    
    // Read button inputs (example)
    if (buttonUpPressed()) {
        static uint8_t level = 3;
        if (level < 5) {
            level++;
            bikeBus.setSupportLevel(level);
            Serial.printf("Support level: %d\n", level);
        }
    }
    
    if (buttonDownPressed()) {
        static uint8_t level = 3;
        if (level > 1) {
            level--;
            bikeBus.setSupportLevel(level);
            Serial.printf("Support level: %d\n", level);
        }
    }
}
```

### Example 4: Push Assist Button

```cpp
void loop() {
    bikeBus.update();
    
    static bool pushAssistActive = false;
    bool buttonPressed = digitalRead(PUSH_BUTTON_PIN) == LOW;
    
    if (buttonPressed && !pushAssistActive) {
        pushAssistActive = true;
        bikeBus.setPushAssist(true);
        Serial.println("Push assist ON");
    } else if (!buttonPressed && pushAssistActive) {
        pushAssistActive = false;
        bikeBus.setPushAssist(false);
        Serial.println("Push assist OFF");
    }
}
```

## Protocol Details

### Telegram Structure

Each BikeBus telegram consists of 5 bytes:

```
Byte 0: Address (1-240)
Byte 1: Token (command/register)
Byte 2: Value Low Byte
Byte 3: Value High Byte
Byte 4: Checksum (sum of bytes 0-3)
```

### Communication Sequence

The library implements a 24-step telegram sequence:

1. **Step 0**: Query service tool
2. **Odd steps (1, 3, 5...)**: Motor control commands
3. **Even steps (2, 4, 6...)**: Data queries (battery voltage, current, SOC, motor speed, temperature, etc.)

### Half-Duplex Operation

The BikeBus uses a single wire for both TX and RX (half-duplex):

1. Master sends telegram
2. Small delay (2ms) for bus turnaround
3. Clear RX buffer to remove echo of transmitted bytes
4. Wait for slave response (25ms timeout)
5. Verify checksum
6. Process response data

### Error Handling

- **Checksum Validation**: Every received telegram is verified
- **Timeout Handling**: 25ms timeout with automatic retry after 30ms
- **Echo Cancellation**: Transmitted bytes are filtered from RX buffer
- **Error Logging**: All errors logged when debug mode is enabled

## Troubleshooting

### No Response from BikeBus

**Symptoms**: Timeouts, no data received

**Solutions**:
1. Check wiring (TX, RX, GND)
2. Verify baud rate (must be 9600)
3. Ensure proper voltage levels
4. Check for proper half-duplex connection
5. Enable debug output: `bikeBus.setDebug(true)`

### Checksum Errors

**Symptoms**: "Checksum error" in debug output

**Solutions**:
1. Check for electrical noise on the bus
2. Verify ground connection
3. Add ferrite beads to reduce EMI
4. Check cable quality and length

### Motor Not Responding

**Symptoms**: Motor doesn't start or respond to commands

**Solutions**:
1. Ensure `initializeMotor()` is called in `setup()`
2. Check motor address (default: 16)
3. Verify motor is powered and operational
4. Check for error codes: `getMotorErrorBits()`

### Incorrect Speed Reading

**Symptoms**: Speed doesn't match actual speed

**Solutions**:
1. Verify wheel circumference setting
2. Measure actual wheel circumference
3. Use: `bikeBus.setWheelCircumference(measured_mm)`

### Debug Output

Enable debug mode to see all communication:

```cpp
bikeBus.setDebug(true);
```

This will output:
```
TX: 10 03 00 0B 1E
RX: 01 03 00 00 04
TX: 20 14 00 00 34
RX: 01 14 2C 00 41
...
```

## License

This library is based on the BikeBus v1.8 specification.

## License

GNU General Public License v3.0

```
BikeBus Library - Arduino library for BikeBus v1.8 protocol communication
Copyright (C) 2026 Markus Stoeckli <support@stoeckli.net>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

---

**Author**: Markus Stoeckli <support@stoeckli.net>  
**Repository**: https://github.com/stoeckli/BikeBusDisplay  
**Version**: 1.0.0  
**Last Updated**: January 5, 2026

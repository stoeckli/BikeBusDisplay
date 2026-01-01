# BikeBus Protocol Library

This library implements the BikeBus protocol v1.8 for E-Bike communication systems.

## Overview

BikeBus is a communication protocol used for E-Bike systems, allowing communication between:
- Operator Panel (Display) - Master (Address 1)
- Operator Panel in Slave Mode (Address 2)
- Motor Controller (Address 16)
- Brake System (Address 24)
- Battery/Batteries (Address 32/33)
- Lights (Address 48)
- Service Tool (Address 240)

## Hardware Interface

The BikeBus protocol uses a single-wire serial bus with the following characteristics:
- **Baud Rate**: 9600 bps (standard UART)
- **Voltage Level**: Converted to 3.3V for microcontroller compatibility
- **Bus Topology**: Single master (Display), multiple slaves
- **Pull-up**: Master provides pull-up resistor, slaves only pull down

## Protocol Specifications

### Timing
- **Master Transmission**: Every 20ms when client responds
- **Retry Interval**: 30ms if no response received
- **Response Timeout**: Slaves respond within 10ms after master telegram
- **Motor Safety Timeout**: Motor enters safe state if no MainMotorControl telegram received for 100ms

### Telegram Structure
Each telegram consists of 5 bytes:

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
|--------|--------|---------|---------|----------|
| Slave Address | Token | Value (Low) | Value (High) | Checksum |

**Checksum**: Arithmetic sum of first 4 bytes

### Response Structure
Normal response:
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
|--------|--------|---------|---------|----------|
| 0x01 (Master) | Token | Value (Low) | Value (High) | Checksum |

Unknown token response:
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |
|--------|--------|---------|---------|----------|
| 0x01 (Master) | 0x00 | Slave Address | Request Token | Checksum |

## Usage Example

```cpp
#include <BikeBus.h>

// Create BikeBus instance with Serial2
BikeBus bikeBus(Serial2);

void setup() {
    // Initialize at 9600 baud
    bikeBus.begin(9600);
}

void loop() {
    uint16_t totalKm = 0;
    
    // Read total kilometers from display (slave mode)
    if (bikeBus.readValue(BIKEBUS_ADDR_OPERATOR_PANEL_SLAVE, 
                          TOKEN_TOTAL_KM, 
                          totalKm)) {
        Serial.printf("Total KM: %d\n", totalKm);
    }
    
    // Write wheel circumference (2200mm)
    bikeBus.writeValue(BIKEBUS_ADDR_OPERATOR_PANEL_SLAVE,
                       TOKEN_WHEEL_CIRCUMFERENCE_WRITE,
                       2200);
    
    delay(100);
}
```

## Available Tokens (GOD Display - Address 2)

### Read/Write Tokens
- `TOKEN_AUTO_OFF_TIME` (68/69) - Automatic power-off time
- `TOKEN_IMPERIAL` (70/71) - Metric/Imperial units
- `TOKEN_WHEEL_CIRCUMFERENCE` (72/73) - Wheel circumference in mm
- `TOKEN_EBIKE_DEMAND` (74/75) - E-bike demand settings
- `TOKEN_BRAKE_DEMAND` (76/77) - Brake demand settings
- `TOKEN_LED_BRIGHTNESS` (80/81) - LED brightness and LCD contrast

### Read-Only Tokens
- `TOKEN_TOTAL_MILES` (132) - Total miles (long)
- `TOKEN_COUNTER` (134) - Counter value
- `TOKEN_TOTAL_KM` (136) - Total kilometers (long)
- `TOKEN_TOTAL_TIME` (140) - Total time (long)
- `TOKEN_TOTAL_ENERGY` (144) - Total energy (long)
- `TOKEN_MOTOR_ERR_BITS` (196) - Motor error bits
- `TOKEN_MOTOR_ERR_CODE` (198) - Motor error code
- `TOKEN_BATTERY_ERRORS` (200) - Battery errors
- `TOKEN_BATTERY_SAFETY_STATUS` (202) - Battery safety status

## Error Codes

- `0x20` - TX_ERRORS: Transmission error
- `0x21` - TX_BUSY: Bus busy
- `0x22` - TX_BYTE_TIMEOUT: Transmission timeout
- `0x23` - WRONG_ADDR_ECHO: Wrong address response
- `0x24` - WRONG_TOKEN_ECHO: Wrong token response
- `0x25` - WRONG_VALLO_ECHO: Wrong low byte response
- `0x26` - WRONG_VALHI_ECHO: Wrong high byte response

## API Reference

### Constructor
```cpp
BikeBus(HardwareSerial &serial)
```

### Methods
- `void begin(uint32_t baudRate = 9600)` - Initialize communication
- `bool readValue(uint8_t slaveAddr, uint8_t token, uint16_t &value)` - Read value from slave
- `bool writeValue(uint8_t slaveAddr, uint8_t token, uint16_t value)` - Write value to slave
- `uint8_t getLastError()` - Get last error code
- `void clearError()` - Clear error state
- `bool isBusAvailable()` - Check if bus is available

## Hardware Connection (M5Stack Core2)

```
M5Stack Core2          BikeBus Interface
--------------         -----------------
GPIO 13 (RX2)  <--->   BikeBus Data
GPIO 14 (TX2)  <--->   BikeBus Data
GND            <--->   GND
```

Note: You'll need the BikeBus hardware interface circuit (see BikeBus-v1.8.pdf Figure 1) to convert between UART TX/RX and the single-wire BikeBus data line.

## License

This implementation is based on the BikeBus protocol specification v1.8.

## Reference

Based on: BikeBus-v1.8.docx - Project E-13-0090 (E-Bike)

#include "BikeBus.h"

// ============================================================================
// BikeBus Class Implementation
// ============================================================================

BikeBus::BikeBus(HardwareSerial* ser, uint16_t wheelCircumferenceMm) 
    : serial(ser), 
      sequenceIndex(0),
      motorControlValue(MMC_NEUTRAL),
      motorConfigured(false), 
      waitingForResponse(false),
      motorSpeed(0), 
      motorRevolutions(0), 
      motorTemperature(0),
      motorErrorBits(0), 
      motorErrorCode(0),
      batteryVoltage(0), 
      batteryCurrent(0), 
      batteryAvgCurrent(0),
      batterySOC(0), 
      batteryTemp(0), 
      batteryStatus(0),
      batteryTimeToEmpty(0), 
      lightStatus(0),
      debugEnabled(false),
      wheelCircumference(wheelCircumferenceMm) {
    lastTxTime = 0;
    lastMotorControlTime = 0;
}

void BikeBus::begin(unsigned long baud) {
    serial->begin(baud);
    sequenceIndex = 0;
}

void BikeBus::setDebug(bool enable) {
    debugEnabled = enable;
}

void BikeBus::setWheelCircumference(uint16_t circumferenceMm) {
    wheelCircumference = circumferenceMm;
}

// Calculate checksum
uint8_t BikeBus::calculateChecksum(uint8_t* data) {
    return data[0] + data[1] + data[2] + data[3];
}

// Send telegram
bool BikeBus::sendTelegram(uint8_t address, uint8_t token, uint16_t value) {
    lastTxTime = millis();
    txBuffer[0] = address;
    txBuffer[1] = token;
    txBuffer[2] = value & 0xFF;        // Low byte
    txBuffer[3] = (value >> 8) & 0xFF; // High byte
    txBuffer[4] = calculateChecksum(txBuffer);
    
    serial->write(txBuffer, 5);
    serial->flush(); // Wait for transmission to complete
    
    // Clear receive buffer to remove echo of transmitted bytes
    delay(2); // Small delay to ensure transmitted bytes appear in RX buffer
    while (serial->available()) {
        serial->read();
    }
    
    // Debug: Print sent message
    if (debugEnabled) {
        Serial.print("TX: ");
        for (int i = 0; i < 5; i++) {
            if (txBuffer[i] < 0x10) Serial.print("0");
            Serial.print(txBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    currentAddress = address;
    currentToken = token;
    waitingForResponse = true;

    return true;
}

// Receive telegram
bool BikeBus::receiveTelegram(unsigned long timeout) {
    unsigned long startTime = millis();
    int bytesRead = 0;
    
    while (millis() - startTime < timeout) {
        if (serial->available()) {
            rxBuffer[bytesRead] = serial->read();
            bytesRead++;
            
            if (bytesRead >= 5) {
                // Debug: Print received message
                if (debugEnabled) {
                    Serial.print("RX: ");
                    for (int i = 0; i < 5; i++) {
                        if (rxBuffer[i] < 0x10) Serial.print("0");
                        Serial.print(rxBuffer[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                }
                
                // Verify checksum
                uint8_t expectedChecksum = calculateChecksum(rxBuffer);
                if (rxBuffer[4] == expectedChecksum) {
                    waitingForResponse = false;
                    return true;
                }
                
                // Debug: Print checksum error
                if (debugEnabled) {
                    Serial.print("Checksum error! Expected: 0x");
                    if (expectedChecksum < 0x10) Serial.print("0");
                    Serial.print(expectedChecksum, HEX);
                    Serial.print(", Got: 0x");
                    if (rxBuffer[4] < 0x10) Serial.print("0");
                    Serial.println(rxBuffer[4], HEX);
                }
                return false;
            }
        }
    }
    
    // Debug: Print timeout
    if (debugEnabled) {
        Serial.print("RX timeout after ");
        Serial.print(bytesRead);
        Serial.println(" bytes");
    }
    return false;
}

// Process received response
void BikeBus::processResponse() {
    uint16_t value = rxBuffer[2] | (rxBuffer[3] << 8);
    
    // Check if this is an error response (Token 0x00)
    if (rxBuffer[1] == 0x00) {
        // Unknown token error
        return;
    }
    
    // Process based on address and token
    if (currentAddress == BIKEBUS_ADDR_MOTOR) {
        switch (currentToken) {
            case MOTOR_TOKEN_MAIN_CONTROL:
                motorErrorBits = value;
                break;
            case MOTOR_TOKEN_SPEED:
                motorSpeed = value;
                break;
            case MOTOR_TOKEN_REVOLUTIONS:
                motorRevolutions = value;
                break;
            case MOTOR_TOKEN_TEMPERATURE:
                motorTemperature = value;
                break;
            case MOTOR_TOKEN_ERROR_CODE:
                motorErrorCode = value & 0xFF;
                break;
        }
    } else if (currentAddress == BIKEBUS_ADDR_BATTERY1) {
        switch (currentToken) {
            case BATTERY_TOKEN_VOLTAGE:
                batteryVoltage = value;
                break;
            case BATTERY_TOKEN_CURRENT:
                batteryCurrent = (int16_t)value;
                break;
            case BATTERY_TOKEN_AVG_CURRENT:
                batteryAvgCurrent = (int16_t)value;
                break;
            case BATTERY_TOKEN_SOC:
                batterySOC = value & 0xFF;
                break;
            case BATTERY_TOKEN_TEMP:
                batteryTemp = value;
                break;
            case BATTERY_TOKEN_STATUS:
                batteryStatus = value;
                break;
            case BATTERY_TOKEN_AVG_TIME_EMPTY:
                batteryTimeToEmpty = value;
                break;
        }
    }
}

// Initialize motor (set to Pedelec mode)
void BikeBus::initializeMotor() {
    sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_CONFIG_WORD_W, CONFIG_PEDELEC_MODE);
    delay(20);
    if (receiveTelegram(BIKEBUS_RX_TIMEOUT)) {
        processResponse();
    }
    motorConfigured = true;
}

// Set motor control value (support level, regeneration, or push assist)
void BikeBus::setMotorControl(uint16_t value) {
    motorControlValue = value;
}

// Set support level (1-5)
void BikeBus::setSupportLevel(uint8_t level) {
    if (level >= 1 && level <= 5) {
        motorControlValue = 0x0800 + (level << 8);
    }
}

// Set regeneration level (1-3)
void BikeBus::setRegenerationLevel(uint8_t level) {
    if (level >= 1 && level <= 3) {
        motorControlValue = 0x0800 - ((4 - level) << 8);
    }
}

// Enable/disable push assist
void BikeBus::setPushAssist(bool enable) {
    if (enable) {
        // Switch to eBike mode first
        sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_CONFIG_WORD_W, CONFIG_EBIKE_MODE);
        delay(20);
        receiveTelegram(BIKEBUS_RX_TIMEOUT);
        
        motorControlValue = MMC_PUSH_ASSIST;
    } else {
        // Deactivate push assist sequence
        motorControlValue = MMC_NEUTRAL;
        
        // Send neutral 8 times
        for (int i = 0; i < 8; i++) {
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_MAIN_CONTROL, MMC_NEUTRAL);
            delay(20);
            receiveTelegram(BIKEBUS_RX_TIMEOUT);
            delay(20);
        }
        
        // Switch back to Pedelec mode
        sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_CONFIG_WORD_W, CONFIG_PEDELEC_MODE);
        delay(20);
        receiveTelegram(BIKEBUS_RX_TIMEOUT);
    }
}

// Set current limit (in Amperes)
void BikeBus::setCurrentLimit(uint16_t currentA) {
    sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_CURRENT_LIMIT_R, currentA);
    delay(20);
    receiveTelegram(BIKEBUS_RX_TIMEOUT);
}

// Control light
void BikeBus::setLight(uint8_t mode) {
    sendTelegram(BIKEBUS_ADDR_LIGHT, LIGHT_TOKEN_CONTROL_W, mode);
}

// Main update function - executes telegram sequence
void BikeBus::update() {
    unsigned long currentTime = millis();
    
    // Check if enough time has passed for next telegram
    bool shouldSend = false;
    if (waitingForResponse) {
        if (currentTime - lastTxTime >= BIKEBUS_TX_RETRY_INTERVAL) {
            shouldSend = true; // Retry after timeout
        }
    } else {
        if (currentTime - lastTxTime >= BIKEBUS_TX_INTERVAL) {
            shouldSend = true;
        }
    }
    
    if (!shouldSend) return;
    
    // Execute telegram sequence
    switch (sequenceIndex) {
        case 0: // Check for service tool
            sendTelegram(BIKEBUS_ADDR_SERVICE_TOOL, 0x02, 0x0000);
            break;
        case 1: // Motor control
        case 3:
        case 5:
        case 7:
        case 9:
        case 11:
        case 13:
        case 15:
        case 17:
        case 19:
        case 21:
        case 23:
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_MAIN_CONTROL, motorControlValue);
            lastMotorControlTime = currentTime;
            break;
        case 2: // Battery: Average time to empty
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_AVG_TIME_EMPTY, 0x0000);
            break;
        case 4: // Motor: Revolutions
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_REVOLUTIONS, 0x0000);
            break;
        case 6: // Motor: Speed
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_SPEED, 0x0000);
            break;
        case 8: // Battery: Flags
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_FLAGS_R, 0x0000);
            break;
        case 10: // Battery: Current
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_CURRENT, 0x0000);
            break;
        case 12: // Battery: Average current
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_AVG_CURRENT, 0x0000);
            break;
        case 14: // Battery: Voltage
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_VOLTAGE, 0x0000);
            break;
        case 16: // Battery: Status
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_STATUS, 0x0000);
            break;
        case 18: // Battery: State of charge
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_SOC, 0x0000);
            break;
        case 20: // Battery: Temperature
            sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_TEMP, 0x0000);
            break;
        case 22: // Motor: Temperature
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_TEMPERATURE, 0x0000);
            break;
    }
    
    // Wait for response
    if (receiveTelegram(BIKEBUS_RX_TIMEOUT)) {
        processResponse();
    }
    
    // Move to next sequence step
    sequenceIndex++;
    if (sequenceIndex >= 24) {
        sequenceIndex = 0;
    }
}

// Motor getters
int16_t BikeBus::getMotorSpeed() { 
    return motorSpeed; 
}

float BikeBus::getSpeedKmh() { 
    return (motorSpeed * wheelCircumference * 60.0) / 1000000.0; 
}

uint16_t BikeBus::getMotorRevolutions() { 
    return motorRevolutions; 
}

uint16_t BikeBus::getMotorTemperature() { 
    return motorTemperature; 
}

float BikeBus::getMotorTempC() { 
    return motorTemperature - 273.15; 
}

uint16_t BikeBus::getMotorErrorBits() { 
    return motorErrorBits; 
}

uint8_t BikeBus::getMotorErrorCode() { 
    return motorErrorCode; 
}

// Battery getters
uint16_t BikeBus::getBatteryVoltage() { 
    return batteryVoltage; 
}

float BikeBus::getBatteryVoltageV() { 
    return batteryVoltage / 1000.0; 
}

int16_t BikeBus::getBatteryCurrent() { 
    return batteryCurrent; 
}

float BikeBus::getBatteryCurrentA() { 
    return batteryCurrent / 1000.0; 
}

uint8_t BikeBus::getBatterySOC() { 
    return batterySOC; 
}

uint16_t BikeBus::getBatteryTemp() { 
    return batteryTemp; 
}

float BikeBus::getBatteryTempC() { 
    return (batteryTemp / 10.0) - 273.15; 
}

uint16_t BikeBus::getBatteryStatus() { 
    return batteryStatus; 
}

uint16_t BikeBus::getBatteryTimeToEmpty() { 
    return batteryTimeToEmpty; 
}

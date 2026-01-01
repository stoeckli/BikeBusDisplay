#include "BikeBus.h"

BikeBus::BikeBus(HardwareSerial &serial) 
    : _serial(serial), _lastError(0), _lastSendTime(0) {
}

void BikeBus::begin(uint32_t baudRate) {
    _serial.begin(baudRate);
    _lastError = 0;
    _lastSendTime = 0;
}

uint8_t BikeBus::calculateChecksum(const BikeBusTelegram &telegram) {
    // Checksum is the arithmetic sum of the first 4 bytes
    return telegram.address + telegram.token + telegram.valueLow + telegram.valueHigh;
}

bool BikeBus::validateTelegram(const BikeBusTelegram &telegram) {
    uint8_t expectedChecksum = calculateChecksum(telegram);
    return telegram.checksum == expectedChecksum;
}

void BikeBus::sendRawTelegram(const BikeBusTelegram &telegram) {
    _serial.write(telegram.address);
    _serial.write(telegram.token);
    _serial.write(telegram.valueLow);
    _serial.write(telegram.valueHigh);
    _serial.write(telegram.checksum);
    _serial.flush();
}

bool BikeBus::receiveRawTelegram(BikeBusTelegram &telegram, uint32_t timeout) {
    uint32_t startTime = millis();
    uint8_t bytesRead = 0;
    uint8_t buffer[5];
    
    while (bytesRead < 5 && (millis() - startTime) < timeout) {
        if (_serial.available()) {
            buffer[bytesRead++] = _serial.read();
        }
    }
    
    if (bytesRead < 5) {
        _lastError = ERROR_TXBYTETIMEOUT;
        return false;
    }
    
    telegram.address = buffer[0];
    telegram.token = buffer[1];
    telegram.valueLow = buffer[2];
    telegram.valueHigh = buffer[3];
    telegram.checksum = buffer[4];
    
    return true;
}

bool BikeBus::sendTelegram(uint8_t slaveAddr, uint8_t token, uint16_t value) {
    // Check if enough time has passed since last transmission
    uint32_t currentTime = millis();
    uint32_t timeSinceLastSend = currentTime - _lastSendTime;
    
    if (timeSinceLastSend < BIKEBUS_MASTER_INTERVAL) {
        _lastError = ERROR_TXBUSY;
        return false;
    }
    
    BikeBusTelegram telegram;
    telegram.address = slaveAddr;
    telegram.token = token;
    telegram.valueLow = value & 0xFF;
    telegram.valueHigh = (value >> 8) & 0xFF;
    telegram.checksum = calculateChecksum(telegram);
    
    // Clear receive buffer
    while (_serial.available()) {
        _serial.read();
    }
    
    sendRawTelegram(telegram);
    _lastSendTime = currentTime;
    
    return true;
}

bool BikeBus::receiveResponse(BikeBusTelegram &telegram, uint32_t timeout) {
    if (!receiveRawTelegram(telegram, timeout)) {
        return false;
    }
    
    // Validate checksum
    if (!validateTelegram(telegram)) {
        _lastError = ERROR_TXERRORS;
        return false;
    }
    
    // Check if it's an "unknown token" response
    if (telegram.token == 0x00) {
        _lastError = ERROR_WRONGTOKENECHO;
        return false;
    }
    
    return true;
}

bool BikeBus::readValue(uint8_t slaveAddr, uint8_t token, uint16_t &value) {
    if (!sendTelegram(slaveAddr, token, 0)) {
        return false;
    }
    
    BikeBusTelegram response;
    if (!receiveResponse(response, BIKEBUS_RESPONSE_TIMEOUT)) {
        return false;
    }
    
    // Verify response address (should be master address 0x01)
    if (response.address != BIKEBUS_ADDR_OPERATOR_PANEL) {
        _lastError = ERROR_WRONGADDRECHO;
        return false;
    }
    
    // Verify response token matches request
    if (response.token != token) {
        _lastError = ERROR_WRONGTOKENECHO;
        return false;
    }
    
    // Extract value
    value = response.valueLow | (response.valueHigh << 8);
    
    return true;
}

bool BikeBus::writeValue(uint8_t slaveAddr, uint8_t token, uint16_t value) {
    if (!sendTelegram(slaveAddr, token, value)) {
        return false;
    }
    
    BikeBusTelegram response;
    if (!receiveResponse(response, BIKEBUS_RESPONSE_TIMEOUT)) {
        return false;
    }
    
    // Verify response
    if (response.address != BIKEBUS_ADDR_OPERATOR_PANEL) {
        _lastError = ERROR_WRONGADDRECHO;
        return false;
    }
    
    if (response.token != token) {
        _lastError = ERROR_WRONGTOKENECHO;
        return false;
    }
    
    return true;
}

bool BikeBus::isBusAvailable() {
    return _lastError == 0;
}

#include <Arduino.h>
#include <M5Unified.h>

// ============================================================================
// BikeBus Protocol Implementation
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
#define BIKEBUS_RX_TIMEOUT          10
#define MOTOR_SAFETY_TIMEOUT        100

// Debug flag - set to true to print hex messages
#define BIKEBUS_DEBUG               true

// wheel circumference (in mm)
#define WHEEL_CIRCUMFERENCE          2222

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
    
    // Telegram sequence state
    int sequenceIndex;
    
    // Motor state
    uint16_t motorControlValue;
    int16_t motorSpeed;           // in RPM (can be negative for reverse)
    uint16_t motorRevolutions;
    uint16_t motorTemperature;    // in Kelvin
    uint16_t motorErrorBits;
    uint8_t motorErrorCode;
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
    uint8_t calculateChecksum(uint8_t* data) {
        return data[0] + data[1] + data[2] + data[3];
    }
    
    // Send telegram
    bool sendTelegram(uint8_t address, uint8_t token, uint16_t value) {
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
        if (BIKEBUS_DEBUG) {
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
        lastTxTime = millis();
        

        return true;
    }
    
    // Receive telegram
    bool receiveTelegram(unsigned long timeout) {
        unsigned long startTime = millis();
        int bytesRead = 0;
        
        while (millis() - startTime < timeout) {
            if (serial->available()) {
                rxBuffer[bytesRead] = serial->read();
                bytesRead++;
                
                if (bytesRead >= 5) {
                    // Debug: Print received message
                    if (BIKEBUS_DEBUG) {
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
                    if (BIKEBUS_DEBUG) {
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
        if (BIKEBUS_DEBUG) {
            Serial.print("RX timeout after ");
            Serial.print(bytesRead);
            Serial.println(" bytes");
        }
        return false;
    }
    
    // Process received response
    void processResponse() {
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
    
public:
    BikeBus(HardwareSerial* ser) : serial(ser), sequenceIndex(0), 
                                   motorControlValue(MMC_NEUTRAL),
                                   motorConfigured(false), waitingForResponse(false),
                                   motorSpeed(0), motorRevolutions(0), motorTemperature(0),
                                   motorErrorBits(0), motorErrorCode(0),
                                   batteryVoltage(0), batteryCurrent(0), batteryAvgCurrent(0),
                                   batterySOC(0), batteryTemp(0), batteryStatus(0),
                                   batteryTimeToEmpty(0), lightStatus(0) {
        lastTxTime = 0;
        lastMotorControlTime = 0;
    }
    
    void begin(unsigned long baud = 9600) {
        serial->begin(baud);
        sequenceIndex = 0;
    }
    
    // Initialize motor (set to Pedelec mode)
    void initializeMotor() {
        sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_CONFIG_WORD_W, CONFIG_PEDELEC_MODE);
        delay(20);
        if (receiveTelegram(BIKEBUS_RX_TIMEOUT)) {
            processResponse();
        }
        motorConfigured = true;
    }
    
    // Set motor control value (support level, regeneration, or push assist)
    void setMotorControl(uint16_t value) {
        motorControlValue = value;
    }
    
    // Set support level (1-5)
    void setSupportLevel(uint8_t level) {
        if (level >= 1 && level <= 5) {
            motorControlValue = 0x0800 + (level << 8);
        }
    }
    
    // Set regeneration level (1-3)
    void setRegenerationLevel(uint8_t level) {
        if (level >= 1 && level <= 3) {
            motorControlValue = 0x0800 - ((4 - level) << 8);
        }
    }
    
    // Enable/disable push assist
    void setPushAssist(bool enable) {
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
    void setCurrentLimit(uint16_t currentA) {
        sendTelegram(BIKEBUS_ADDR_BATTERY1, BATTERY_TOKEN_CURRENT_LIMIT_R, currentA);
        delay(20);
        receiveTelegram(BIKEBUS_RX_TIMEOUT);
    }
    
    // Control light
    void setLight(uint8_t mode) {
        sendTelegram(BIKEBUS_ADDR_LIGHT, LIGHT_TOKEN_CONTROL_W, mode);
    }
    
    // Main update function - executes telegram sequence
    void update() {
        unsigned long currentTime = millis();
        
        // Safety check: Motor must receive control telegram every 100ms
        /*if (motorConfigured && (currentTime - lastMotorControlTime >= MOTOR_SAFETY_TIMEOUT)) {
            sendTelegram(BIKEBUS_ADDR_MOTOR, MOTOR_TOKEN_MAIN_CONTROL, motorControlValue);
            lastMotorControlTime = currentTime;
            if (receiveTelegram(BIKEBUS_RX_TIMEOUT)) {
                processResponse();
            }
            return;
        }*/
        
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
    
    // Getters
    int16_t getMotorSpeed() { return motorSpeed; }  // RPM (can be negative)
    float getSpeedKmh() { return (motorSpeed * WHEEL_CIRCUMFERENCE * 60.0) / 1000000.0; }
    uint16_t getMotorRevolutions() { return motorRevolutions; }
    uint16_t getMotorTemperature() { return motorTemperature; }
    float getMotorTempC() { return motorTemperature - 273.15; }
    uint16_t getMotorErrorBits() { return motorErrorBits; }
    uint8_t getMotorErrorCode() { return motorErrorCode; }
    
    uint16_t getBatteryVoltage() { return batteryVoltage; }
    float getBatteryVoltageV() { return batteryVoltage / 1000.0; }
    int16_t getBatteryCurrent() { return batteryCurrent; }
    float getBatteryCurrentA() { return batteryCurrent / 1000.0; }
    uint8_t getBatterySOC() { return batterySOC; }
    uint16_t getBatteryTemp() { return batteryTemp; }
    float getBatteryTempC() { return (batteryTemp / 10.0) - 273.15; }
    uint16_t getBatteryStatus() { return batteryStatus; }
    uint16_t getBatteryTimeToEmpty() { return batteryTimeToEmpty; }
};

// ============================================================================
// Global Variables
// ============================================================================

BikeBus bikeBus(&Serial2);  // Use Serial2 for BikeBus communication
unsigned long lastDisplayUpdate = 0;
uint8_t currentSupportLevel = 5;  // Default to level 5

// ============================================================================
// Setup
// ============================================================================

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    
    // Initialize USB Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait up to 3 seconds for Serial
    Serial.println("\n\n=== BikeBus Display Starting ===");
    
    // Initialize GPIO 26 for device power/enable
    pinMode(26, OUTPUT);
    digitalWrite(26, HIGH);
    
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    
    // Initialize BikeBus on Serial2
    // Pin configuration for M5Stack Core2: RX=13, TX=14
    bikeBus.begin(9600);
    
    // Small delay for bus stabilization
    delay(10);
    
    // Initialize motor
    bikeBus.initializeMotor();
    bikeBus.setSupportLevel(currentSupportLevel);
    
    M5.Display.clear();
    M5.Display.setCursor(10, 10);
    M5.Display.println("BikeBus Display");
    M5.Display.println("Initialized");
    
    delay(10);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    M5.update();
    
    // Update BikeBus communication
    bikeBus.update();
    
    // Handle button inputs
    if (M5.BtnA.wasPressed()) {
        // Decrease support level
        if (currentSupportLevel > 1) {
            currentSupportLevel--;
            bikeBus.setSupportLevel(currentSupportLevel);
        }
    }
    
    if (M5.BtnB.wasPressed()) {
        // Toggle light
        static bool lightOn = false;
        lightOn = !lightOn;
        bikeBus.setLight(lightOn ? LIGHT_DRIVE : LIGHT_OFF);
    }
    
    if (M5.BtnC.wasPressed()) {
        // Increase support level
        if (currentSupportLevel < 5) {
            currentSupportLevel++;
            bikeBus.setSupportLevel(currentSupportLevel);
        }
    }
    
    // Update display every 200ms
    if (millis() - lastDisplayUpdate >= 200) {
        lastDisplayUpdate = millis();
        
        M5.Display.clear();
        M5.Display.setCursor(10, 10);
        
        // Display header
        M5.Display.setTextColor(YELLOW);
        M5.Display.println("=== BikeBus Display ===");
        M5.Display.println();
        
        // Display motor info
        M5.Display.setTextColor(GREEN);
        M5.Display.println("MOTOR:");
        M5.Display.setTextColor(WHITE);
        M5.Display.printf("  Level: %d\n", currentSupportLevel);
        M5.Display.printf("  Speed: %.1f km/h\n", bikeBus.getSpeedKmh());
        M5.Display.printf("  Temp:  %.1f C\n", bikeBus.getMotorTempC());
        
        // Display battery info
        M5.Display.setTextColor(GREEN);
        M5.Display.println("BATTERY:");
        M5.Display.setTextColor(WHITE);
        M5.Display.printf("  SOC:   %d%%\n", bikeBus.getBatterySOC());
        M5.Display.printf("  Volt:  %.1f V\n", bikeBus.getBatteryVoltageV());
        M5.Display.printf("  Curr:  %.1f A\n", bikeBus.getBatteryCurrentA());
        M5.Display.printf("  Temp:  %.1f C\n", bikeBus.getBatteryTempC());
        
        // Display errors if any
        if (bikeBus.getMotorErrorBits() != 0) {
            M5.Display.setTextColor(RED);
            M5.Display.printf("\nERROR: 0x%04X\n", bikeBus.getMotorErrorBits());
        }
        
        // Button legend
        M5.Display.setCursor(10, 220);
        M5.Display.setTextColor(CYAN);
        M5.Display.println("A:-  B:Light  C:+");
    }
    
    delay(10);
}
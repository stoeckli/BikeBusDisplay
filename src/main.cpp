/**
 * @file main.cpp
 * @brief Main application for BikeBus Display system
 * 
 * This file implements the main application logic for the BikeBus Display, a comprehensive
 * e-bike display and control system for M5Stack Core2. It provides real-time monitoring
 * of speed, battery status, motor parameters, and control functions including support
 * levels, push assist, and lighting control.
 * 
 * Features:
 * - Real-time speed, battery SOC, and motor parameter display
 * - 5 support levels with visual feedback
 * - Touch-activated push assist mode
 * - Bus monitoring mode for protocol debugging
 * - GPIO-controlled bus voltage for proper power management
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

#include <Arduino.h>
#include <M5Unified.h>
#include <BikeBus.h>

// ============================================================================
// Pin Definitions
// ============================================================================
#define BUS_VOLTAGE_ENABLE_PIN 25    // GPIO for bus voltage control
#define BUS_ENABLE_PIN  26  // GPIO for bus control

// ============================================================================
// Constants
// ============================================================================
#define AUTO_SHUTDOWN_TIMEOUT_MS  (10 * 60 * 1000)  // 10 minutes in milliseconds

// ============================================================================
// Global Variables
// ============================================================================

BikeBus bikeBus(&Serial2, 2222);  // Use Serial2 for BikeBus communication, wheel circumference 2222mm
unsigned long lastDisplayUpdate = 0;
unsigned long lastActivityTime = 0;  // Track last activity for auto-shutdown
uint8_t currentSupportLevel = 0;  // Default to level 0
bool pushAssistActive = false;    // Track push assist state
bool busMonitorMode = false;      // Track if bus monitor mode is active

// Previous values for display update detection
float prevSpeed = -1;
uint8_t prevSOC = 255;
uint8_t prevLevel = 0;
uint16_t prevErrorBits = 0;
float prevVoltage = -1;
float prevCurrent = 999;
float prevTemp = -999;
uint16_t prevMotorCurrentLimit = 0;
bool prevPushAssist = false;
bool displayInitialized = false;

// Common gauge parameters (constants)
const int gaugeX = 20;
const int gaugeWidth = 280;
const int gaugeHeight = 40;
const int labelWidth = 80;
const int barX = gaugeX + labelWidth;
const int barWidth = gaugeWidth - labelWidth;
const int speedY = 20;
const int socY = 90;
const int levelY = 160;

// Function to draw static display elements (labels and borders)
void drawStaticElements() {
    // SPEED label and border
    M5.Display.setTextColor(YELLOW);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(gaugeX, speedY + 12);
    M5.Display.print("SPEED");
    M5.Display.drawRect(barX, speedY, barWidth, gaugeHeight, WHITE);
    
    // BATTERY label and border
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(gaugeX, socY + 12);
    M5.Display.print("CHARGE");
    M5.Display.drawRect(barX, socY, barWidth, gaugeHeight, WHITE);
    
    // LEVEL label and border
    M5.Display.setTextColor(CYAN);
    M5.Display.setCursor(gaugeX, levelY + 12);
    M5.Display.print("LEVEL");
    M5.Display.drawRect(barX, levelY, barWidth, gaugeHeight, WHITE);
}

// Function to update speed gauge
void updateSpeedGauge(float speed) {
    // Clear the gauge area (inside the border)
    M5.Display.fillRect(barX + 2, speedY + 2, barWidth - 4, gaugeHeight - 4, BLACK);
    
    // Calculate and draw fill
    int speedFillWidth = (barWidth - 4) * speed / 50.0;
    if (speedFillWidth > 0) {
        M5.Display.fillRect(barX + 2, speedY + 2, speedFillWidth, gaugeHeight - 4, CYAN);
    }
    
    // Draw speed value
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(barX + barWidth/2 - 35, speedY + 12);
    M5.Display.printf("%.1f km/h", speed);
}

// Function to update SOC gauge
void updateSOCGauge(uint8_t soc) {
    // Clear the gauge area (inside the border)
    M5.Display.fillRect(barX + 2, socY + 2, barWidth - 4, gaugeHeight - 4, BLACK);
    
    // Calculate and draw fill
    int socFillWidth = (barWidth - 4) * soc / 100;
    
    // Color based on SOC level
    uint16_t socColor;
    if (soc > 50) {
        socColor = GREEN;
    } else if (soc > 20) {
        socColor = ORANGE;
    } else {
        socColor = RED;
    }
    
    if (socFillWidth > 0) {
        M5.Display.fillRect(barX + 2, socY + 2, socFillWidth, gaugeHeight - 4, socColor);
    }
    
    // Draw SOC percentage
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(barX + barWidth/2 - 20, socY + 12);
    M5.Display.printf("%d%%", soc);
}

// Function to update level gauge
void updateLevelGauge(uint8_t level, bool pushActive = false) {
    // Clear the gauge area (inside the border)
    M5.Display.fillRect(barX + 2, levelY + 2, barWidth - 4, gaugeHeight - 4, BLACK);
    
    if (pushActive) {
        // Fill entire bar in orange/yellow for push assist
        M5.Display.fillRect(barX + 2, levelY + 2, barWidth - 4, gaugeHeight - 4, ORANGE);
        
        // Draw "PUSH" text
        M5.Display.setTextColor(BLACK);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(barX + barWidth/2 - 30, levelY + 12);
        M5.Display.print("PUSH");
    } else {
        // Calculate and draw fill
        int levelFillWidth = (barWidth - 4) * level / 5;
        if (levelFillWidth > 0) {
            M5.Display.fillRect(barX + 2, levelY + 2, levelFillWidth, gaugeHeight - 4, MAGENTA);
        }
        
        // Draw level value
        M5.Display.setTextColor(WHITE);
        M5.Display.setTextSize(3);
        M5.Display.setCursor(barX + barWidth/2 - 10, levelY + 8);
        M5.Display.printf("%d", level);
    }
}

// Function to update bottom info line
void updateBottomInfo(float voltage, float current, float temp, uint16_t currentLimit) {
    // Clear the bottom info area
    M5.Display.fillRect(gaugeX, 220, 280, 20, BLACK);
    
    // Draw info - first line
    M5.Display.setTextColor(DARKGREY);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(gaugeX, 220);
    M5.Display.printf("V:%.1fV I:%.1fA T:%.0fC", voltage, current, temp);
    
    // Draw info - second line with motor current limit
    M5.Display.setCursor(gaugeX, 228);
    M5.Display.printf("Motor Limit: %dA", currentLimit);
}

// Function to update error display
void updateErrorDisplay(uint16_t errorBits) {
    // Clear error area
    M5.Display.fillRect(10, 5, 150, 10, BLACK);
    
    // Draw error if present
    if (errorBits != 0) {
        M5.Display.setTextColor(RED);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(10, 5);
        M5.Display.printf("ERR:0x%04X", errorBits);
    }
}

// Function to perform shutdown sequence
void performShutdown(const char* reason) {
    Serial.printf("\n*** AUTO SHUTDOWN: %s ***\n", reason);
    
    // Show shutdown message
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(ORANGE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(30, 80);
    M5.Display.println("AUTO SHUTDOWN");
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(30, 110);
    M5.Display.println(reason);
    delay(2000);
    
    // Disable bus control
    digitalWrite(BUS_ENABLE_PIN, LOW);
    
    // Disable bus voltage before shutdown
    digitalWrite(BUS_VOLTAGE_ENABLE_PIN, LOW);
    delay(100);  // Allow bus to settle
    
    // Power off the device
    M5.Power.powerOff();
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    
    // Check if Button A is pressed during boot to enter bus monitor mode
    M5.update();
    if (M5.BtnA.isPressed()) {
        busMonitorMode = true;
    }
    
    // Initialize USB Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait up to 3 seconds for Serial
    Serial.println("\n\n=== BikeBus Display Starting ===");
    
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    
    // Initialize BikeBus on Serial2
    // Pin configuration for M5Stack Core2: RX=13, TX=14
    bikeBus.begin(9600);
    
    // Initialize GPIO for bus voltage control
    // HIGH = Enable bus voltage for communication
    // LOW = Disable bus voltage for shutdown
    pinMode(BUS_VOLTAGE_ENABLE_PIN, OUTPUT);
    digitalWrite(BUS_VOLTAGE_ENABLE_PIN, HIGH);

    // Initialize GPIO for Bus Control
    pinMode(BUS_ENABLE_PIN, OUTPUT);
    digitalWrite(BUS_ENABLE_PIN, HIGH);

    if (busMonitorMode) {
        // Bus Monitor Mode
        Serial.println("\n*** BUS MONITOR MODE ACTIVATED ***");
        Serial.println("Listening to all BikeBus traffic...\n");
        Serial.println("Format: [Timestamp] ADDR:xxx (Device) TOKEN:xxx VALUE:0xXXXX (decimal) RAW:[hex bytes] CHECKSUM:status\n");
        
        M5.Display.clear();
        M5.Display.setCursor(10, 10);
        M5.Display.setTextColor(YELLOW);
        M5.Display.println("BUS MONITOR");
        M5.Display.println("MODE");
        M5.Display.setTextColor(WHITE);
        M5.Display.setTextSize(1);
        M5.Display.println();
        M5.Display.println("Listening to");
        M5.Display.println("all bus traffic");
        M5.Display.println();
        M5.Display.println("See Serial output");
        M5.Display.println("for details");
        
        return; // Skip normal initialization
    }
    
    // Normal mode continues here
    bikeBus.setDebug(true);  // Enable debug output
    
    // Small delay for bus stabilization
    delay(10);
    
    // Query motor current limit
    bikeBus.queryMotorCurrentLimit();
    delay(10);

    // Initialize motor
    bikeBus.initializeMotor();
    delay(10);


    // Display current limit
    uint16_t currentLimit = bikeBus.getMotorCurrentLimit();
    Serial.printf("Motor current limit: %dA\n", currentLimit);

    // Set motor current limit
    bikeBus.setMotorCurrentLimit(currentLimit);
    delay(10);

    // query actual motor speed
    bikeBus.queryMotorSpeed();
    delay(10);

    // Set support level
    bikeBus.setSupportLevel(currentSupportLevel);

    // Send motor control value to activate the motor
    bikeBus.sendMotorControl();

    // Initialize activity timer
    lastActivityTime = millis();
    
    M5.Display.clear();
    M5.Display.setCursor(10, 10);
    M5.Display.println("BikeBus Display");
    M5.Display.println("Initialized");
    M5.Display.printf("Max Current: %dA", currentLimit);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    M5.update();
    
    // Check if in bus monitor mode
    if (busMonitorMode) {
        // Bus monitor mode - just listen and report
        bikeBus.busMonitor();
        
        // Check if user wants to exit (hold button A for 3 seconds)
        if (M5.BtnA.pressedFor(3000)) {
            Serial.println("\n*** Exiting Bus Monitor Mode - Rebooting ***\n");
            M5.Display.clear();
            M5.Display.setCursor(10, 100);
            M5.Display.println("Rebooting...");
            delay(1000);
            ESP.restart();
        }
        return; // Don't execute normal loop code
    }
    
    // Normal mode continues here
    // Update BikeBus communication
    bikeBus.update();
    
    // Check for activity (bus communication or movement)
    float currentSpeed = bikeBus.getSpeedKmh();
    bool busActive = bikeBus.isBusActive(5000);  // Check if bus responded in last 5 seconds
    bool hasMovement = (currentSpeed > 0.5);  // Speed threshold to detect movement
    
    if (busActive || hasMovement) {
        // Update activity timestamp if there's bus communication or movement
        lastActivityTime = millis();
    }
    
    // Check for auto-shutdown timeout (10 minutes of inactivity)
    if (millis() - lastActivityTime > AUTO_SHUTDOWN_TIMEOUT_MS) {
        if (!busActive && !hasMovement) {
            performShutdown("Inactivity timeout");
        }
    }
    
    // Handle touch input for push assist
    // Only activate when touching the upper half of the display (y < 120)
    auto touch = M5.Touch.getDetail();
    bool screenTouched = (touch.state != m5::touch_state_t::none);
    bool touchInUpperHalf = screenTouched && (touch.y < 120);
    
    if (touchInUpperHalf && !pushAssistActive) {
        // Touch detected in upper half - enable push assist
        pushAssistActive = true;
        bikeBus.setPushAssist(true);
        Serial.printf("Push assist ON (touch at x=%d, y=%d)\n", touch.x, touch.y);
    } else if (!touchInUpperHalf && pushAssistActive) {
        // Touch released or moved to lower half - disable push assist
        pushAssistActive = false;
        bikeBus.setPushAssist(false);
        Serial.println("Push assist OFF");
    }
    
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
    
    // Check for long press on Button B (hold for 3 seconds to power off)
    if (M5.BtnB.pressedFor(3000)) {
        // Show power off message
        M5.Display.fillScreen(BLACK);
        M5.Display.setTextColor(RED);
        M5.Display.setTextSize(3);
        M5.Display.setCursor(60, 100);
        M5.Display.println("POWERING OFF...");
        delay(1000);
        
        // Disable bus control
        digitalWrite(BUS_ENABLE_PIN, LOW);

        // Disable bus voltage before shutdown
        digitalWrite(BUS_VOLTAGE_ENABLE_PIN, LOW);
        delay(100);  // Allow bus to settle
        
        // Power off the device
        M5.Power.powerOff();
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
        
        // Get current values
        float speed = bikeBus.getSpeedKmh();
        if (speed < 0) speed = 0;
        if (speed > 50) speed = 50;
        
        uint8_t soc = bikeBus.getBatterySOC();
        if (soc > 100) soc = 100;
        
        uint16_t errorBits = bikeBus.getMotorErrorBits();
        float voltage = bikeBus.getBatteryVoltageV();
        float current = bikeBus.getBatteryCurrentA();
        float temp = bikeBus.getMotorTempC();
        uint16_t motorCurrentLimit = bikeBus.getMotorCurrentLimit();
        
        // Initialize display on first run
        if (!displayInitialized) {
            M5.Display.clear();
            drawStaticElements();
            displayInitialized = true;
            
            // Force initial draw of all values
            prevSpeed = -999;
            prevSOC = 255;
            prevLevel = 0;
            prevErrorBits = 0xFFFF;
            prevVoltage = -999;
            prevCurrent = 999;
            prevTemp = -999;
        }
        
        // Update only changed elements
        if (abs(speed - prevSpeed) > 0.5) {
            updateSpeedGauge(speed);
            prevSpeed = speed;
        }
        
        if (soc != prevSOC) {
            updateSOCGauge(soc);
            prevSOC = soc;
        }
        
        if (currentSupportLevel != prevLevel || pushAssistActive != prevPushAssist) {
            updateLevelGauge(currentSupportLevel, pushAssistActive);
            prevLevel = currentSupportLevel;
            prevPushAssist = pushAssistActive;
        }
        
        if (errorBits != prevErrorBits) {
            updateErrorDisplay(errorBits);
            prevErrorBits = errorBits;
        }
        
        if (abs(voltage - prevVoltage) > 0.1 || 
            abs(current - prevCurrent) > 0.1 || 
            abs(temp - prevTemp) > 1.0 ||
            motorCurrentLimit != prevMotorCurrentLimit) {
            updateBottomInfo(voltage, current, temp, motorCurrentLimit);
            prevVoltage = voltage;
            prevCurrent = current;
            prevTemp = temp;
            prevMotorCurrentLimit = motorCurrentLimit;
        }
    }
    
}
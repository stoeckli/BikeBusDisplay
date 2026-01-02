#include <Arduino.h>
#include <M5Unified.h>
#include <BikeBus.h>

// ============================================================================
// Global Variables
// ============================================================================

BikeBus bikeBus(&Serial2, 2222);  // Use Serial2 for BikeBus communication, wheel circumference 2222mm
unsigned long lastDisplayUpdate = 0;
uint8_t currentSupportLevel = 5;  // Default to level 5

// Previous values for display update detection
float prevSpeed = -1;
uint8_t prevSOC = 255;
uint8_t prevLevel = 0;
uint16_t prevErrorBits = 0;
float prevVoltage = -1;
float prevCurrent = 999;
float prevTemp = -999;
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
void updateLevelGauge(uint8_t level) {
    // Clear the gauge area (inside the border)
    M5.Display.fillRect(barX + 2, levelY + 2, barWidth - 4, gaugeHeight - 4, BLACK);
    
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

// Function to update bottom info line
void updateBottomInfo(float voltage, float current, float temp) {
    // Clear the bottom info area
    M5.Display.fillRect(gaugeX, 220, 280, 10, BLACK);
    
    // Draw info
    M5.Display.setTextColor(DARKGREY);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(gaugeX, 220);
    M5.Display.printf("V:%.1fV I:%.1fA T:%.0fC", voltage, current, temp);
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
    bikeBus.setDebug(true);  // Enable debug output
    
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
    
    // Check for long press on Button B (hold for 3 seconds to power off)
    if (M5.BtnB.pressedFor(3000)) {
        // Show power off message
        M5.Display.fillScreen(BLACK);
        M5.Display.setTextColor(RED);
        M5.Display.setTextSize(3);
        M5.Display.setCursor(60, 100);
        M5.Display.println("POWERING OFF...");
        delay(1000);
        
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
        
        if (currentSupportLevel != prevLevel) {
            updateLevelGauge(currentSupportLevel);
            prevLevel = currentSupportLevel;
        }
        
        if (errorBits != prevErrorBits) {
            updateErrorDisplay(errorBits);
            prevErrorBits = errorBits;
        }
        
        if (abs(voltage - prevVoltage) > 0.1 || 
            abs(current - prevCurrent) > 0.1 || 
            abs(temp - prevTemp) > 1.0) {
            updateBottomInfo(voltage, current, temp);
            prevVoltage = voltage;
            prevCurrent = current;
            prevTemp = temp;
        }
    }
    
}
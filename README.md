# BikeBus Display

A comprehensive e-bike display system for BikeBus v1.8 compatible motors and batteries using the M5Stack Core2.

![License](https://img.shields.io/badge/license-GPL--3.0-blue.svg)
![PlatformIO](https://img.shields.io/badge/PlatformIO-compatible-orange.svg)

## Overview

BikeBus Display is a feature-rich display and control interface for e-bikes using the BikeBus v1.8 protocol. It provides real-time monitoring of speed, battery status, motor parameters, and allows control of support levels, push assist, and lighting.

## Features

### Display & Monitoring
- **Real-time speed display** with configurable wheel circumference
- **Battery state of charge (SOC)** with color-coded gauge
- **Support level indication** (1-5) with visual feedback
- **Push assist mode** with touch control
- **Motor temperature monitoring**
- **Battery voltage and current monitoring**
- **Motor current limit display**
- **Error code display** for diagnostics

### Control Functions
- **5 support levels** for pedal assistance
- **Push assist mode** (touch-activated)
- **Light control** (on/off via button)
- **Auto-shutdown** after 10 minutes of inactivity (no bus response or zero speed)
- **Bus monitoring mode** for protocol debugging
- **GPIO-controlled bus voltage** for proper power management

### Hardware Support
- M5Stack Core2 (ESP32-based)
- BikeBus v1.8 compatible motor controllers
- BikeBus v1.8 compatible battery management systems
- BikeBus v1.8 compatible lighting systems

## Hardware Setup

### Required Hardware
- M5Stack Core2
- M5Stack Proto Module
- LIN interface (e.g. TLIN2029A)
- Step-down voltage regulator which can hanlde 50 V (e.g. Pololu D36V6F5)
- cirquit to enable bus power (e.g. NPN and P-FET)

### Pin Configuration
- **Serial2 RX**: GPIO 13
- **Serial2 TX**: GPIO 14
- **Bus Voltage Enable**: GPIO 25
- **Bus Enable**: GPIO 26

### Wiring
Connect the M5Stack Core2 to the Proto Module
Implement the volage regulator, level coverter and LIN interface on the Proto Module

## Software Setup

### Requirements
- [PlatformIO IDE](https://platformio.org/) (VS Code extension recommended)
- M5Stack Core2 board support
- M5Unified library (automatically installed)

### Installation

1. Clone or download this repository:
```bash
git clone https://github.com/stoeckli/BikeBusDisplay.git
cd BikeBusDisplay
```

2. Open the project in PlatformIO:
```bash
code .  # or open in VS Code with PlatformIO extension
```

3. Build and upload:
```bash
pio run --target upload
```

## Usage

### Normal Operation Mode

1. **Power On**: The display will initialize and show the BikeBus Display splash screen
2. **Speed Display**: Shows current speed in km/h (0-50 km/h range)
3. **Battery Display**: Shows state of charge percentage with color coding:
   - Green: > 50%
   - Orange: 20-50%
   - Red: < 20%
4. **Support Level**: Shows current assistance level (1-5)

### Button Controls

- **Button A (Left)**: Decrease support level
- **Button B (Middle)**: Toggle lights (single press) / Power off (hold 3 seconds)
- **Button C (Right)**: Increase support level

### Touch Controls

- **Touch upper half of screen**: Activate push assist mode
- **Release touch**: Deactivate push assist mode

### Bus Monitor Mode

Hold Button A during boot to enter bus monitoring mode. This mode displays all BikeBus traffic on the serial console for debugging purposes. Hold Button A for 3 seconds to exit and reboot. You might have to manually interrupt the BUS pull-up when another controller is being monitored.

### Auto-Shutdown Feature

The display will automatically power off after **10 minutes of inactivity** to conserve battery power. Inactivity is defined as:
- No response from the bus (motor/battery not communicating)
- Zero speed (bike not moving)

Activity is detected when:
- Bus successfully responds to queries (motor/battery communication active)
- Speed is above 0.5 km/h (bike is moving)

The shutdown countdown resets whenever activity is detected. When auto-shutdown occurs, the display shows a brief message before powering off gracefully.

## Configuration

### Auto-Shutdown Timeout

To change the auto-shutdown timeout, modify the constant in `src/main.cpp`:

```cpp
#define AUTO_SHUTDOWN_TIMEOUT_MS  (10 * 60 * 1000)  // Change 10 to desired minutes
```

### Wheel Circumference

The default wheel circumference is set to 2222mm. To change this, modify the value in `src/main.cpp`:

```cpp
BikeBus bikeBus(&Serial2, 2222);  // Change 2222 to your wheel circumference in mm
```

### GPIO Pins

To change the bus voltage control pins, modify the defines in `src/main.cpp`:

```cpp
#define BUS_VOLTAGE_ENABLE_PIN 25    // GPIO for bus voltage control
#define BUS_ENABLE_PIN  26           // GPIO for bus control
```

## Project Structure

```
BikeBusDisplay/
├── src/
│   └── main.cpp              # Main application code
├── lib/
│   └── BikeBus/              # BikeBus protocol library
│       ├── BikeBus.h         # Library header
│       ├── BikeBus.cpp       # Library implementation
│       └── README.md         # Library documentation
├── include/                  # Project headers (if any)
├── test/                     # Unit tests (if any)
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## Development

### Building

```bash
pio run
```

### Uploading

```bash
pio run --target upload
```

### Serial Monitor

```bash
pio device monitor
```

## Troubleshooting

### No Communication with Motor/Battery

1. Check wiring connections (RX/TX/GND)
2. Verify bus voltage is enabled (GPIO pins)
3. Check baud rate (should be 9600)
4. Use bus monitor mode to see raw traffic
5. Check for error codes on display

### Display Not Updating

1. Verify motor is responding to queries
2. Check serial debug output
3. Ensure BikeBus is properly initialized
4. Check for error bits in motor response

### Push Assist Not Working

1. Ensure touch is in upper half of screen
2. Check motor supports push assist mode
3. Verify motor control telegrams are being sent

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the GNU General Public License v3.0 - see below for details.

```
BikeBus Display - E-bike display system for BikeBus v1.8 protocol
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

## References

- [BikeBus Protocol v1.8 Specification](https://github.com/stoeckli/Bikebus_Battery/blob/main/BikeBus-v1.8.pdf)
- [M5Stack Core2 Documentation](https://docs.m5stack.com/en/core/core2)
- [PlatformIO Documentation](https://docs.platformio.org/)

## Author

**Markus Stoeckli**  
Email: support@stoeckli.net  
GitHub: [@stoeckli](https://github.com/stoeckli)

## Acknowledgments

- BikeBus protocol specification by BikeBus consortium
- M5Stack for the excellent Core2 hardware platform
- PlatformIO for the development environment

---

**Version**: 1.0.0  
**Last Updated**: January 5, 2026

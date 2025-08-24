# Developer Guide - ESP8266 Plant Watering System

## Overview

This guide provides technical details for developers working on the ESP8266-based automatic plant watering system. The system uses deep sleep for power efficiency and capacitive soil moisture sensing for reliable plant care.

## System Architecture

### Hardware Components
- **ESP8266 Development Board** (NodeMCU/Wemos D1 Mini)
- **Capacitive Soil Moisture Sensor** (connected to A0)
- **Water Pump** (controlled via relay on D6)
- **Power Supply** (3.3V/5V depending on components)

### Pin Configuration
```cpp
#define relayPin D5          // Legacy definition (not used)
const int SENSOR_POWER = D1; // Powers the moisture sensor
const int PUMP_PIN = D6;     // Controls water pump relay
// A0 - Analog input for moisture sensor
```

### Power Management
- **Deep Sleep Mode**: 8-hour cycles (28,800 seconds)
- **Wake-up Trigger**: Internal timer
- **Power Consumption**: ~20µA in deep sleep, ~80mA when active

## Code Structure

### Main Flow
1. **Boot/Wake-up** → Check reset reason
2. **Sensor Reading** → Power sensor, read moisture, power down
3. **Decision Logic** → Compare moisture level, trigger pump if needed
4. **Deep Sleep** → Enter 8-hour sleep cycle

### Key Functions

#### `setup()`
- Initializes WiFi off mode for power saving
- Checks reset reason (cold boot vs deep sleep wake)
- Executes watering logic only on deep sleep wake
- Enters deep sleep mode

#### `readMoisture()`
```cpp
int readMoisture()
```
- **Purpose**: Read soil moisture with noise filtering
- **Algorithm**: Exponential Moving Average (EMA) filter
- **Samples**: 15 readings with 10ms intervals
- **Power Management**: Sensor powered only during reading
- **Returns**: Filtered analog value (0-1024)

#### `triggerRelay()`
```cpp
void triggerRelay()
```
- **Purpose**: Activate water pump for irrigation
- **Duration**: 6 seconds watering cycle
- **Safety**: Pin configured as output before use
- **Power**: Pump powered via relay (HIGH = ON)

## Configuration Parameters

### Moisture Calibration
```cpp
const int AirValue = 600;    // Sensor reading in dry air
const int WaterValue = 260;  // Sensor reading in water
```
**Calibration Process:**
1. Place sensor in dry air → record `AirValue`
2. Place sensor in water → record `WaterValue`
3. Update constants in code

### Timing Configuration
```cpp
const uint32_t maxDeepSleepTime = 8000000; // 8 hours in microseconds
```
**Sleep Duration Calculation:**
- Value in microseconds (µs)
- 8,000,000 µs = 8 seconds (for testing)
- 28,800,000,000 µs = 8 hours (production)

### Watering Thresholds
```cpp
if (soilmoisturepercent < 10) // Trigger watering below 10%
```

## Development Setup

### Prerequisites
- Arduino IDE 1.8.x or newer
- ESP8266 Board Package
- USB cable for programming

### Board Configuration
```
Board: NodeMCU 1.0 (ESP-12E Module)
CPU Frequency: 80 MHz
Flash Size: 4MB (FS:2MB OTA:~1019KB)
Upload Speed: 115200
```

### Required Libraries
```cpp
#include <ESP8266WiFi.h>  // Built-in ESP8266 library
```

## Testing & Debugging

### Serial Monitor Setup
- **Baud Rate**: 115200
- **Line Ending**: Both NL & CR
- **Monitor Output**: Reset reason, moisture readings, watering actions

### Debug Output Example
```
Reset reason: 5
Woke from deep sleep
342
23%
wet
Entering deep sleep for 8 hours...
```

### Test Procedures

#### 1. Moisture Sensor Test
```cpp
// Add to setup() for testing
Serial.println("Testing moisture sensor...");
for(int i = 0; i < 10; i++) {
  int reading = readMoisture();
  Serial.println(reading);
  delay(1000);
}
```

#### 2. Pump Test
```cpp
// Test pump operation
Serial.println("Testing pump...");
triggerRelay();
```

## Troubleshooting

### Common Issues

#### 1. Relay Triggers on Boot
**Problem**: D5 pin goes HIGH during ESP8266 boot
**Solution**: Use D6 for pump control (implemented in code)

#### 2. Inconsistent Moisture Readings
**Problem**: Sensor noise or power issues
**Solutions**:
- Check sensor power connections
- Verify `AirValue` and `WaterValue` calibration
- Increase `SETTLE_DELAY` if needed

#### 3. Deep Sleep Not Working
**Problem**: ESP8266 not entering sleep mode
**Check**: D0 connected to RST pin for wake-up capability

### Reset Reason Codes
```cpp
REASON_DEFAULT_RST = 0      // Power on reset
REASON_WDT_RST = 1         // Hardware watchdog reset
REASON_EXCEPTION_RST = 2    // Exception reset
REASON_SOFT_WDT_RST = 3    // Software watchdog reset
REASON_SOFT_RESTART = 4     // Software restart
REASON_DEEP_SLEEP_AWAKE = 5 // Deep sleep wake up
REASON_EXT_SYS_RST = 6     // External system reset
```

## Code Modifications

### Adding WiFi Connectivity
```cpp
// Add after sensor reading
WiFi.mode(WIFI_STA);
WiFi.begin("SSID", "PASSWORD");
// Send data to server
WiFi.mode(WIFI_OFF);
```

### Multiple Sensors Support
```cpp
const int SENSOR2_POWER = D2;
int readMoisture2() {
  // Similar to readMoisture() but for second sensor
}
```

## Hardware Considerations

### PCB Design
- Add pull-down resistors on control pins
- Include test points for debugging
- Consider reverse polarity protection
- Add LED indicators for status

### Power Optimization
- **Deep Sleep**: ~20µA current draw
- **Active Time**: <10 seconds per cycle
- **Battery Life**: ~6 months on 18650 Li-ion

---

**Last Updated**: December 2024  
**Hardware Version**: v1.0  
**Firmware Version**: v1.0
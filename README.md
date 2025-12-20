🚗 Smart Parking System with AI Car Detection

An intelligent parking management system using ESP32-CAM for AI-based car detection, ultrasonic distance sensing, and automated gate control with Arduino MKR WiFi 1010.

🎯 Overview

This smart parking system combines computer vision, ultrasonic sensing, and automated gate control to manage parking lot capacity efficiently. The system uses Edge Impulse AI model for real-time car detection and communicates via serial protocol for gate automation.

### Key Capabilities
- Real-time car detection using Edge Impulse AI
- Ultrasonic distance measurement for object detection
- Automated parking gate control
- Capacity management (max 5 vehicles)
- Low-power mode for energy efficiency
- Real-time system health monitoring
- Web-based visualization dashboard

## ✨ Features

### ESP32-CAM Features
- ✅ AI-powered car detection using Edge Impulse
- ✅ HC-SR04 ultrasonic distance sensor integration
- ✅ Low-power mode (activates when no objects within 30cm)
- ✅ JSON-based data transmission
- ✅ System health monitoring (memory, camera, sensors)
- ✅ Real-time timestamp tracking

### Arduino MKR WiFi 1010 Features
- ✅ Servo motor gate control
- ✅ Vehicle counter (0-5 capacity)
- ✅ Automatic gate operation
- ✅ Parking full detection
- ✅ Visual capacity display in Serial Monitor

### Web Dashboard Features
- ✅ Real-time parking capacity visualization
- ✅ Live sensor data display
- ✅ System health monitoring
- ✅ Gate status indicator
- ✅ Responsive design

## 🛠 Hardware Requirements

### ESP32-CAM Module
- **Board:** ESP32-CAM
- **RAM:** 4MB
- **Power:** 5V via USB or external supply

### Arduino MKR WiFi 1010
- **Microcontroller:** SAMD21 Cortex-M0+ 32bit
- **Connectivity:** WiFi
- **Power:** 5V via USB or VIN

### Additional Components
| Component                        | Quantity | Purpose              |
|----------------------------------|----------|----------------------|
| HC-SR04 Ultrasonic Sensor        |     1    | Distance measurement |
| Servo Motor (SG90 or similar)    |     1    | Gate control         |
| Jumper Wires                     |    10+   | Connections          |
| Breadboard                       |     1    | Prototyping          |
| 5V Power Supply                  |    1-2   | Power for modules    |

## 💻 Software Requirements

### Arduino IDE
- **Download:** https://www.arduino.cc/en/software

### Required Libraries

#### For ESP32-CAM
```
- ESP32 Board Support (by Espressif Systems)
- Edge Impulse Arduino Library
- ESP32 Camera Library (included with ESP32 board support)
```

#### For Arduino MKR WiFi 1010
```
- ArduinoJson (by Benoit Blanchon) v6.x
- Servo (included with Arduino IDE)
- SAMD Board Support (by Arduino)
```

### Edge Impulse Account
- Sign up at: [edgeimpulse.com](https://edgeimpulse.com)
- Train your car detection model
- Export as Arduino library

### Installation Steps for Libraries

1. **Install Board Support:**
   - Open Arduino IDE
   - Go to `Tools` → `Board` → `Boards Manager`
   - Search "ESP32" and install "esp32 by Espressif Systems"
   - Search "Arduino SAMD" and install "Arduino SAMD Boards"

2. **Install ArduinoJson:**
   - Go to `Sketch` → `Include Library` → `Manage Libraries`
   - Search "ArduinoJson"
   - Install version 6.x or higher

3. **Install Edge Impulse Library:**
   - Download your trained model from Edge Impulse Studio
   - Go to `Sketch` → `Include Library` → `Add .ZIP Library`
   - Select your downloaded Edge Impulse library

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     SMART PARKING SYSTEM                     │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐         Serial (TX/RX)        ┌──────────────────┐
│   ESP32-CAM      │ ──────────────────────────────▶│  Arduino MKR     │
│                  │         JSON Data              │  WiFi 1010       │
│  - Camera (AI)   │                                │                  │
│  - HC-SR04       │                                │  - Servo Motor   │
│  - Edge Impulse  │                                │  - Counter Logic │
└──────────────────┘                                └──────────────────┘
        │                                                     │
        │                                                     │
        ▼                                                     ▼
┌──────────────────┐                                ┌──────────────────┐
│  Car Detection   │                                │  Gate Control    │
│  Distance Sense  │                                │  Capacity Mgmt   │
└──────────────────┘                                └──────────────────┘
```

### Data Flow
1. ESP32-CAM captures image and measures distance
2. AI model processes image for car detection
3. System enters low-power mode if no objects nearby (>30cm)
4. JSON data sent via TX to Arduino
5. Arduino receives data and updates counter
6. If car detected AND counter < 5: Open gate
7. If counter = 5: Gate remains closed (parking full)
8. Gate auto-closes after 5 seconds

## 🔧 Installation

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/smart-parking-system.git
cd smart-parking-system
```

### Step 2: Install Arduino IDE and Libraries
Follow the [Software Requirements](#software-requirements) section above.

### Step 3: Prepare Your Edge Impulse Model
1. Go to [Edge Impulse Studio](https://studio.edgeimpulse.com)
2. Create a new project
3. Collect and label car images
4. Train your model
5. Download as Arduino library
6. Install in Arduino IDE

### Step 4: Upload Code to ESP32-CAM
1. Open `ESP32_CAM_Code/ESP32_CAM_Code.ino`
2. Select board: `Tools` → `Board` → `ESP32 Arduino` → `AI Thinker ESP32-CAM`
3. Connect ESP32-CAM via USB-TTL programmer
4. Hold BOOT button while uploading
5. Click Upload
6. Open Serial Monitor (115200 baud) to verify

### Step 5: Upload Code to Arduino MKR WiFi 1010
1. Open `Arduino_MKR_Code/Arduino_MKR_Code.ino`
2. Select board: `Tools` → `Board` → `Arduino SAMD Boards` → `Arduino MKR WiFi 1010`
3. Select correct port
4. Click Upload
5. Open Serial Monitor (115200 baud)

### Step 6: Open Web Dashboard
1. Open `web_dashboard/index.html` in a web browser
2. The dashboard will simulate data for testing
3. For live data, implement WebSocket or Serial-to-Web bridge

## 🔌 Wiring Diagram

### ESP32-CAM Connections

| ESP32-CAM Pin | Component | Pin/Wire |
|---------------|-----------|----------|
| GPIO 13 | HC-SR04 TRIG | TRIG Pin |
| GPIO 15 | HC-SR04 ECHO | ECHO Pin |
| GPIO 1 (TX) | Arduino MKR RX | Pin 13 |
| GPIO 3 (RX) | Arduino MKR TX | Pin 14 |
| GND | Common Ground | GND |
| 5V | Power Supply | 5V |

### Arduino MKR WiFi 1010 Connections

| Arduino Pin | Component | Notes |
|-------------|-----------|-------|
| Pin 9 | Servo Signal | Orange/Yellow wire |
| Pin 13 (RX) | ESP32-CAM TX | Serial communication |
| Pin 14 (TX) | ESP32-CAM RX | Serial communication |
| 5V | Servo Power | Red wire |
| GND | Servo & ESP32 GND | Brown/Black wire |

### HC-SR04 Ultrasonic Sensor

| HC-SR04 Pin | ESP32-CAM Pin |
|-------------|---------------|
| VCC | 5V |
| TRIG | GPIO 13 |
| ECHO | GPIO 15 |
| GND | GND |

### Power Supply Notes
- Use common ground for all components

## ⚙️ Configuration

### ESP32-CAM Settings (in code)

```cpp
// Node identification
#define NODE_ID "ESP32_CAM_01"  // Change this for multiple nodes

// Power management
#define LOW_POWER_THRESHOLD 30  // Distance threshold in cm
#define INFERENCE_INTERVAL_MS 5 // Normal mode delay
#define LOW_POWER_INTERVAL_MS 1000 // Low power mode delay

// Pins
#define CAR_SIGNAL_PIN 12
#define TRIG_PIN 13
#define ECHO_PIN 15
```

### Arduino MKR WiFi 1010 Settings

```cpp
// Servo configuration
#define SERVO_PIN 9
#define GATE_OPEN_ANGLE 90    // Adjust based on your servo
#define GATE_CLOSED_ANGLE 0   // Adjust based on your servo

// Parking capacity
#define MAX_CAPACITY 5  // Change to your parking lot size
```

## 🚀 Usage

### Starting the System

1. **Power on ESP32-CAM:**
   - Connect to power supply
   - Wait for "Camera initialized" message
   - System will start inference after 2 seconds

2. **Power on Arduino MKR WiFi 1010:**
   - System will wait for incoming JSON data
   - Serial Monitor will show "Waiting for data..."

3. **Monitor Operation:**
   - ESP32-CAM will detect cars and send JSON
   - Arduino will control gate based on detection
   - Counter increments with each car entry
   - Gate locks when capacity reaches 5

### Serial Monitor Output

**ESP32-CAM Output:**
```json
{
  "node_id": "ESP32_CAM_01",
  "timestamp": 123456,
  "sensors": {
    "distance_cm": 25.40,
    "car_detected": true,
    "signal_pin": "HIGH"
  },
  "health": {
    "low_power_mode": false,
    "camera_initialized": true,
    "memory_allocated": true,
    "ultrasonic_ok": true,
    "free_heap_bytes": 45000,
    "uptime_ms": 123456
  }
}
```

**Arduino Output:**
```
--- Received Data ---
Node: ESP32_CAM_01
Timestamp: 123456 ms
Distance: 25.4 cm
Car Detected: YES
Low Power Mode: INACTIVE
>>> GATE OPENED <<<
Car #1 entered

========== PARKING STATUS ==========
Cars in parking: 1 / 5
Available spaces: 4
Gate status: OPEN
Capacity: 20%
Capacity Bar: [█····]
====================================
```

## 📊 JSON Data Format

### Complete JSON Structure
```json
{
  "node_id": "string",           // Unique node identifier
  "timestamp": 0,                // Milliseconds since boot
  "sensors": {
    "distance_cm": 0.0,          // Distance in centimeters
    "car_detected": false,       // Boolean: car detection status
    "signal_pin": "LOW"          // Signal pin state: HIGH/LOW
  },
  "health": {
    "low_power_mode": false,     // Power mode status
    "camera_initialized": false, // Camera status
    "memory_allocated": false,   // Memory status
    "ultrasonic_ok": false,      // Ultrasonic sensor status
    "free

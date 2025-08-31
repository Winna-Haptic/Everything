# ⚡ Haptic Glove Wiring Guide

**Complete wiring instructions for the 8-sensor, 8-motor location-specific haptic feedback glove**

## 🎯 System Overview

### **Pin Assignment Strategy**
```cpp
// ESP32 Pin Mapping for Location-Specific Haptic Glove
// Organized by function for easy debugging

// I2C Buses (3 buses for 8 sensors)
#define I2C0_SDA 21    // Primary bus: Thumb, Index, Middle
#define I2C0_SCL 22
#define I2C1_SDA 16    // Secondary bus: Ring, Pinky, Back-of-hand  
#define I2C1_SCL 17
#define I2C2_SDA 32    // Tertiary bus: Wrist, Elbow
#define I2C2_SCL 33

// Haptic Motor PWM Outputs (8 motors)
#define THUMB_MOTOR 25      // Coin vibrator, 200Hz
#define INDEX_MOTOR 26      // Coin vibrator, 235Hz
#define MIDDLE_MOTOR 27     // Coin vibrator, 235Hz
#define RING_MOTOR 14       // Coin vibrator, 200Hz
#define PINKY_MOTOR 12      // Coin vibrator, 180Hz
#define WRIST_FLEX_MOTOR 13 // LRA motor, 175Hz
#define WRIST_ROT_MOTOR 15  // LRA motor, 175Hz
#define ELBOW_MOTOR 2       // Large LRA, 150Hz

// User Interface
#define MODE_BUTTON 0       // Built-in boot button
#define CALIB_BUTTON 4      // External button with pullup
#define STATUS_LED 5        // System status indicator
#define BATTERY_LED 18      // Battery level indicator

// Power Management
#define BATTERY_MONITOR 34  // ADC input for battery voltage
#define POWER_ENABLE 19     // Enable/disable haptic system
```

## 🔌 Detailed Wiring Instructions

### **Power Distribution (Critical First Step)**

```
3.7V Li-Po Battery Connections:
├── Battery+ → TP4056 IN+
├── Battery- → TP4056 IN-
├── TP4056 OUT+ → 3.3V Regulator VIN
├── TP4056 OUT- → Common Ground
├── Regulator VOUT → ESP32 3.3V
└── Common Ground → ESP32 GND

Power Rail Distribution:
├── 3.3V Rail → All sensor VCC pins
├── 3.3V Rail → All motor driver VCC pins  
├── GND Rail → All sensor GND pins
├── GND Rail → All motor driver GND pins
└── GND Rail → All pull-up resistor grounds
```

**⚠️ Critical Power Notes:**
- **Never connect 5V to sensors** - they are 3.3V only!
- **Use thick wires** for power distribution (22 AWG minimum)
- **Add decoupling capacitors** (100nF ceramic + 10μF electrolytic) near each sensor
- **Measure power rail voltage** before connecting sensors

### **I2C Bus 0: Primary Sensors (Thumb, Index, Middle)**

```
ESP32 Connections:
├── GPIO 21 (SDA) → Common SDA line
├── GPIO 22 (SCL) → Common SCL line
├── 3.3V → Pull-up resistors (4.7kΩ each to SDA and SCL)

THUMB_TIP Sensor (Address 0x68):
├── VCC → 3.3V rail
├── GND → GND rail  
├── SDA → GPIO 21 (with 4.7kΩ pullup to 3.3V)
├── SCL → GPIO 22 (with 4.7kΩ pullup to 3.3V)
└── AD0 → GND (sets address to 0x68)

INDEX_TIP Sensor (Address 0x69):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 21 (shared bus)
├── SCL → GPIO 22 (shared bus)
└── AD0 → 3.3V (sets address to 0x69)

MIDDLE_TIP Sensor (Address 0x6A):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 21 (shared bus)
├── SCL → GPIO 22 (shared bus)
└── AD0 → GPIO 23 (sets address to 0x6A via pull-up)
```

### **I2C Bus 1: Secondary Sensors (Ring, Pinky, Back-of-hand)**

```
ESP32 Connections:
├── GPIO 16 (SDA) → Common SDA line
├── GPIO 17 (SCL) → Common SCL line
├── 3.3V → Pull-up resistors (4.7kΩ each to SDA and SCL)

RING_TIP Sensor (Address 0x6B):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 16 (with 4.7kΩ pullup to 3.3V)
├── SCL → GPIO 17 (with 4.7kΩ pullup to 3.3V)
└── AD0 → GPIO 35 (sets custom address)

PINKY_TIP Sensor (Address 0x6C):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 16 (shared bus)
├── SCL → GPIO 17 (shared bus)
└── AD0 → GPIO 36 (sets custom address)

BACK_OF_HAND Sensor (Address 0x6D):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 16 (shared bus)
├── SCL → GPIO 17 (shared bus)
└── AD0 → GPIO 39 (sets custom address)
```

### **I2C Bus 2: Arm Sensors (Wrist, Elbow)**

```
ESP32 Connections:
├── GPIO 32 (SDA) → Common SDA line
├── GPIO 33 (SCL) → Common SCL line
├── 3.3V → Pull-up resistors (4.7kΩ each to SDA and SCL)

WRIST Sensor (Address 0x6E):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 32 (with 4.7kΩ pullup to 3.3V)
├── SCL → GPIO 33 (with 4.7kΩ pullup to 3.3V)
└── AD0 → Custom address configuration

ELBOW Sensor (Address 0x6F):
├── VCC → 3.3V rail
├── GND → GND rail
├── SDA → GPIO 32 (shared bus)
├── SCL → GPIO 33 (shared bus)
└── AD0 → Custom address configuration
```

### **Haptic Motor Array (8 Motors)**

```
Fingertip Coin Motors (5x):
THUMB_MOTOR (GPIO 25):
├── GPIO 25 → Motor Driver IN (or MOSFET gate)
├── 3.3V → Motor Driver VCC
├── GND → Motor Driver GND
├── Motor+ → Driver OUT+
├── Motor- → Driver OUT-
└── Flyback diode across motor terminals

INDEX_MOTOR (GPIO 26):
├── GPIO 26 → Motor Driver IN
├── Same power and protection as above

MIDDLE_MOTOR (GPIO 27):
├── GPIO 27 → Motor Driver IN
├── Same power and protection as above

RING_MOTOR (GPIO 14):
├── GPIO 14 → Motor Driver IN
├── Same power and protection as above

PINKY_MOTOR (GPIO 12):
├── GPIO 12 → Motor Driver IN
├── Same power and protection as above

Wrist LRA Motors (2x):
WRIST_FLEX_MOTOR (GPIO 13):
├── GPIO 13 → DRV2605L or similar haptic driver
├── Driver configured for 175Hz LRA operation
├── Higher current capability than coin motors

WRIST_ROT_MOTOR (GPIO 15):
├── GPIO 15 → DRV2605L or similar haptic driver
├── Driver configured for 175Hz LRA operation

Elbow Large LRA Motor (1x):
ELBOW_MOTOR (GPIO 2):
├── GPIO 2 → High-current motor driver
├── Driver configured for 150Hz LRA operation
├── Maximum current capability for largest motor
```

### **User Interface Connections**

```
Button Inputs (with debouncing):
MODE_BUTTON (GPIO 0):
├── One terminal → GPIO 0
├── Other terminal → GND
├── Built-in pull-up resistor (no external needed)
└── Note: This is the ESP32 boot button

CALIB_BUTTON (GPIO 4):
├── One terminal → GPIO 4
├── Other terminal → GND
├── 10kΩ pull-up resistor → 3.3V
└── 100nF debounce capacitor across button

LED Indicators:
STATUS_LED (GPIO 5):
├── GPIO 5 → 220Ω resistor → LED anode
├── LED cathode → GND
└── LED indicates system state (off/calibration/training)

BATTERY_LED (GPIO 18):
├── GPIO 18 → 220Ω resistor → LED anode
├── LED cathode → GND
└── LED indicates battery level (off/low/good)

Battery Monitoring:
BATTERY_MONITOR (GPIO 34):
├── Battery+ → 10kΩ resistor → GPIO 34
├── GPIO 34 → 10kΩ resistor → GND
├── Creates voltage divider (measures half of battery voltage)
└── ESP32 ADC reads 0-3.3V representing 0-6.6V battery range
```

## 🔧 Motor Driver Circuits

### **Coin Motor Driver (Simple MOSFET)**
```
For each fingertip coin motor:
├── ESP32 GPIO → 1kΩ resistor → MOSFET gate (2N7002 or similar)
├── MOSFET source → GND
├── MOSFET drain → Motor negative terminal
├── Motor positive terminal → 3.3V
├── Flyback diode (1N4148) across motor terminals
└── Gate pull-down resistor (10kΩ) for safety
```

### **LRA Motor Driver (DRV2605L Recommended)**
```
For wrist and elbow LRA motors:
├── ESP32 GPIO → DRV2605L PWM input
├── DRV2605L VDD → 3.3V
├── DRV2605L GND → GND
├── DRV2605L OUT+ → LRA motor positive
├── DRV2605L OUT- → LRA motor negative
├── Configure for appropriate resonant frequency
└── I2C control optional for advanced waveforms
```

## 📏 Cable Management

### **Flexible Cable Routing**
```
Finger Sensor Cables:
├── Use 30 AWG flexible wire (4 conductors per sensor)
├── Route along finger sides to avoid interference
├── Strain relief at knuckle joints with heat shrink
└── Secure to glove with conductive thread

Hand/Arm Sensor Cables:
├── Use 28 AWG wire (slightly thicker for longer runs)
├── Route along natural hand/arm contours
├── Avoid areas of high flexion
└── Use spiral cable wrap for organization

Motor Cables:
├── Use 28 AWG wire (2 conductors per motor)
├── Route opposite side of sensors when possible
├── Keep motor cables away from sensor cables
└── Use shielded cable for longer runs
```

### **Connector Strategy**
```
Recommended Connectors:
├── JST PH series (2mm pitch) for power connections
├── JST SH series (1mm pitch) for I2C connections
├── Micro JST (0.5mm pitch) for fingertip connections
└── Magnetic connectors for easy glove removal
```

## 🧪 Testing & Validation

### **Step-by-Step Testing Protocol**

**1. Power System Test:**
```bash
# Before connecting any components
multimeter_reading = measure_voltage(3.3V_rail)
assert multimeter_reading == 3.3V ± 0.1V
assert measure_voltage(GND_rail) == 0V
```

**2. I2C Bus Test:**
```cpp
// Upload I2C scanner sketch to ESP32
// Should detect all 8 sensors at correct addresses
Expected_addresses = [0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F]
```

**3. Individual Sensor Test:**
```cpp
// Test each sensor individually
for (sensor in sensor_array) {
    assert sensor.read_accel() != [0, 0, 0];
    assert sensor.read_gyro() != [0, 0, 0];
    assert sensor.sample_rate >= 100; // Hz
}
```

**4. Haptic Motor Test:**
```cpp
// Test each motor at different intensities
for (motor in motor_array) {
    motor.set_intensity(0.5);
    assert user_can_feel_vibration();
    motor.set_frequency(motor.resonant_freq);
    assert vibration_is_strongest();
}
```

**5. Location-Specific Mapping Test:**
```cpp
// Verify sensor-to-motor mapping
thumb_sensor.trigger_golden_zone();
assert only_thumb_motor_activates();
wrist_sensor.trigger_golden_zone();
assert only_wrist_motor_activates();
```

## ⚠️ Common Wiring Issues & Solutions

### **I2C Communication Problems**
```
Issue: Sensors not detected
Solutions:
├── Check pull-up resistors (4.7kΩ to 3.3V on SDA and SCL)
├── Verify power supply voltage (exactly 3.3V)
├── Check for short circuits with multimeter
├── Use I2C scanner to identify address conflicts
└── Reduce I2C clock speed if cables are long
```

### **Motor Not Vibrating**
```
Issue: Haptic feedback not working
Solutions:
├── Check motor polarity (some motors are polarity sensitive)
├── Verify PWM signal with oscilloscope
├── Check power supply current capability
├── Test motor with direct 3.3V connection
└── Verify flyback diode orientation
```

### **Power Supply Issues**
```
Issue: System resets or sensors malfunction
Solutions:
├── Add more decoupling capacitors near sensors
├── Use thicker power wires (lower resistance)
├── Check battery voltage under load
├── Verify regulator current capability
└── Add bulk capacitance at power input
```

## 🎯 Production Wiring Optimization

### **PCB Design Considerations**
```
When moving from breadboard to PCB:
├── Separate analog and digital ground planes
├── Use proper I2C trace impedance (90Ω differential)
├── Add test points for all critical signals
├── Include EMI filtering on motor driver outputs
├── Use flex-rigid PCB for finger sensors
└── Design for automated assembly and testing
```

**This wiring guide ensures reliable, location-specific haptic feedback with minimal interference and maximum performance!** ⚡

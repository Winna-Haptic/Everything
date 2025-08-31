# 🔧 Haptic Glove Hardware Components

**Complete component list for building the location-specific haptic feedback glove**

## 🎯 Core System Requirements

### **Processing Unit**
- **ESP32 Development Board** (ESP32-WROOM-32 or similar)
  - Dual-core 240MHz processor
  - Built-in WiFi and Bluetooth
  - 30+ GPIO pins
  - Multiple I2C buses for sensor arrays

### **8x IMU Sensor Array (100Hz minimum)**
```
Recommended: MPU6050 or BNO055 breakout boards
├── Fingertip Sensors (5x): Small form factor IMUs
│   ├── THUMB_TIP → I2C Address 0x68
│   ├── INDEX_TIP → I2C Address 0x69  
│   ├── MIDDLE_TIP → I2C Address 0x6A
│   ├── RING_TIP → I2C Address 0x6B
│   └── PINKY_TIP → I2C Address 0x6C
├── Hand/Arm Sensors (3x): Standard IMU breakouts
│   ├── BACK_OF_HAND → I2C Address 0x6D
│   ├── WRIST → I2C Address 0x6E
│   └── ELBOW → I2C Address 0x6F
```

**Specifications:**
- Accelerometer: ±16g range, 16-bit resolution
- Gyroscope: ±2000°/s range, 16-bit resolution
- Sample Rate: 100Hz minimum (200Hz preferred)
- Communication: I2C (400kHz fast mode)
- Power: 3.3V operation

### **8x Haptic Motor Array**
```
Location-Specific Motor Mapping:
├── Fingertip Motors (5x): Coin vibrators (10mm diameter)
│   ├── THUMB_ZONE → Pin 25, 200Hz resonant frequency
│   ├── INDEX_ZONE → Pin 26, 235Hz resonant frequency
│   ├── MIDDLE_ZONE → Pin 27, 235Hz resonant frequency
│   ├── RING_ZONE → Pin 14, 200Hz resonant frequency
│   └── PINKY_ZONE → Pin 12, 180Hz resonant frequency
├── Wrist Motors (2x): LRA actuators (12mm diameter)
│   ├── WRIST_FLEXION_ZONE → Pin 13, 175Hz resonant frequency
│   └── WRIST_ROTATION_ZONE → Pin 15, 175Hz resonant frequency
└── Elbow Motor (1x): Large LRA actuator (16mm diameter)
    └── ELBOW_ZONE → Pin 2, 150Hz resonant frequency
```

**Motor Specifications:**
- **Coin Motors (fingertips)**: 3.3V, 50mA max, 10mm x 3mm
- **LRA Motors (wrist)**: 3.3V, 80mA max, 12mm diameter
- **Large LRA (elbow)**: 3.3V, 100mA max, 16mm diameter
- **Control**: PWM (0-255 duty cycle)
- **Frequency**: Resonant frequency optimization per motor

### **Power Management**
```
Power System:
├── 3.7V Li-Po Battery (2000mAh recommended)
├── TP4056 USB-C Charging Module
├── 3.3V LDO Regulator (AMS1117-3.3)
└── Power Distribution:
    ├── ESP32: ~200mA typical, 400mA peak
    ├── 8x IMU Sensors: ~40mA total (5mA each)
    ├── 8x Haptic Motors: ~400mA peak (when all active)
    └── Total System: ~640mA peak, ~240mA typical
```

**Battery Life Estimation:**
- **Active Training**: 3-4 hours continuous use
- **Standby**: 24+ hours with sleep mode
- **Charging**: 2-3 hours via USB-C

### **Communication & Interface**
```
User Interface:
├── Mode Button → GPIO 0 (with pull-up resistor)
├── Calibration Button → GPIO 4 (with pull-up resistor)
├── Status LED → GPIO 5 (with 220Ω resistor)
└── Battery Level LED → GPIO 18 (with 220Ω resistor)

Wireless Communication:
├── Bluetooth Low Energy (BLE) → Built-in ESP32
└── WiFi (optional) → Built-in ESP32 for data upload
```

## 🔌 I2C Bus Organization

### **Multiple I2C Buses for Sensor Array**
```cpp
// ESP32 I2C Bus Configuration
I2C Bus 0 (Primary): GPIO 21 (SDA), GPIO 22 (SCL)
├── THUMB_TIP (0x68)
├── INDEX_TIP (0x69)
└── MIDDLE_TIP (0x6A)

I2C Bus 1 (Secondary): GPIO 16 (SDA), GPIO 17 (SCL)  
├── RING_TIP (0x6B)
├── PINKY_TIP (0x6C)
└── BACK_OF_HAND (0x6D)

I2C Bus 2 (Tertiary): GPIO 32 (SDA), GPIO 33 (SCL)
├── WRIST (0x6E)
└── ELBOW (0x6F)
```

**Why Multiple I2C Buses:**
- **Prevents address conflicts** with 8 sensors
- **Reduces bus loading** for 100Hz operation
- **Improves reliability** and timing
- **Easier debugging** and troubleshooting

## 🧤 Glove Integration

### **Flexible Glove Base**
```
Recommended Materials:
├── Neoprene athletic glove (breathable, stretchy)
├── Conductive thread for sensor connections
├── Flexible ribbon cables (0.5mm pitch)
└── 3D printed sensor mounts (TPU flexible filament)
```

### **Sensor Placement Strategy**
```
Fingertip Sensors:
├── Mount on fingertip pads (not nails)
├── Secure with flexible TPU mounts
├── Route cables along finger sides
└── Use strain relief at knuckle joints

Hand/Arm Sensors:
├── BACK_OF_HAND: Center of hand back
├── WRIST: Just above wrist bone
└── ELBOW: Lateral epicondyle area
```

### **Haptic Motor Placement**
```
Motor Positioning (1:1 with sensors):
├── Fingertip motors: Opposite side of sensors
├── Wrist motors: Dorsal and ventral positions
└── Elbow motor: Over muscle belly for maximum sensation
```

## 📋 Bill of Materials (BOM)

### **Electronic Components**
| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|-------|
| ESP32 Dev Board | 1 | $12 | $12 |
| MPU6050 IMU | 8 | $3 | $24 |
| Coin Vibrator (10mm) | 5 | $2 | $10 |
| LRA Motor (12mm) | 2 | $8 | $16 |
| LRA Motor (16mm) | 1 | $12 | $12 |
| Li-Po Battery (2000mAh) | 1 | $15 | $15 |
| TP4056 Charger | 1 | $3 | $3 |
| 3.3V Regulator | 1 | $2 | $2 |
| Resistors/Capacitors | Set | $5 | $5 |
| Flexible Cables | Set | $10 | $10 |
| **Electronics Total** | | | **$109** |

### **Mechanical Components**
| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|-------|
| Neoprene Glove | 1 | $15 | $15 |
| 3D Printed Mounts | Set | $8 | $8 |
| Conductive Thread | 10m | $5 | $5 |
| Velcro Straps | Set | $3 | $3 |
| Enclosure Material | Set | $7 | $7 |
| **Mechanical Total** | | | **$38** |

### **Total System Cost: ~$147**

## 🔧 Assembly Complexity

### **Skill Level Required**
- **Electronics**: Intermediate (soldering, I2C debugging)
- **Mechanical**: Beginner (3D printing, fabric work)
- **Programming**: Advanced (C++ firmware development)
- **Time Investment**: 15-20 hours total assembly

### **Tools Required**
```
Electronics Tools:
├── Soldering iron and solder
├── Multimeter for testing
├── Oscilloscope (helpful for I2C debugging)
└── Heat shrink tubing and wire strippers

Mechanical Tools:
├── 3D printer (for sensor mounts)
├── Fabric scissors
├── Sewing machine (optional)
└── Hot glue gun for strain relief
```

## 🎯 Hardware Validation Plan

### **Component Testing Sequence**
1. **ESP32 Basic Function** → LED blink test
2. **Single IMU Communication** → I2C scanner, basic readings
3. **Multiple IMU Array** → Address conflicts, timing validation
4. **Single Haptic Motor** → PWM control, frequency response
5. **Full Haptic Array** → Simultaneous operation, power consumption
6. **Integrated System** → Sensor-to-motor mapping validation
7. **Glove Integration** → Mechanical fit, cable management
8. **Full System Test** → Complete location-specific feedback

### **Performance Metrics**
- **Sensor Update Rate**: 100Hz minimum, 200Hz target
- **Haptic Latency**: <10ms from sensor reading to motor activation
- **Battery Life**: 4+ hours continuous training
- **Mechanical Durability**: 1000+ throw cycles
- **Wireless Range**: 10+ meters BLE connection

## 🚀 Hardware Development Roadmap

### **Phase 1: Breadboard Prototype**
- ✅ **Algorithm validation** (current simulation)
- 🔄 **Single sensor + motor pair** testing
- ⏳ **I2C bus configuration** validation
- ⏳ **Power consumption** analysis

### **Phase 2: PCB Development**
- ⏳ **Custom PCB design** for compact integration
- ⏳ **Flexible PCB** for finger sensors
- ⏳ **Power management optimization**
- ⏳ **EMI/EMC testing**

### **Phase 3: Glove Integration**
- ⏳ **Mechanical design** and 3D printing
- ⏳ **Cable management** and strain relief
- ⏳ **User comfort** testing and optimization
- ⏳ **Durability** validation

**This hardware specification supports our location-specific haptic feedback architecture perfectly!** 🎯

# Autonomous Fire-Fighting Robot

> An Arduino-based robot that autonomously scans its environment for fire, identifies the direction of the flame, aims a water pump at it, and extinguishes it all without human intervention.

---

## Project Overview

This project was developed during my **SIWES (Students' Industrial Work Experience Scheme - Nigeria's national internship program)** internship at **Hub360 Circuits Ltd, Abuja, Nigeria**. It demonstrates the integration of embedded systems, servo motor control, sensor interfacing, and basic autonomous decision-making on a microcontroller platform.

The robot continuously scans its surroundings using a flame sensor mounted on a rotating servo. Upon detecting fire, it locks onto the direction, aims a second servo-mounted water pump nozzle at the source, and activates a relay-controlled pump to dispense water. After extinguishing the fire, it resumes scanning automatically.

---

## Objectives

- Design a low-cost, autonomous fire detection and suppression system
- Implement servo-based directional scanning for 360° situational awareness (within a configurable arc)
- Develop state-machine logic to transition between **Scan Mode** and **Extinguish Mode**
- Control a water pump via relay based on sensor feedback
- Validate the system's ability to detect and extinguish a small open flame

---

## Components & Hardware

| Component | Quantity | Purpose |
|---|---|---|
| Arduino Uno | 1 | Main microcontroller |
| IR Flame Sensor | 1 | Detect presence of fire (mounted on scan servo) |
| SG90 Servo Motor (Scan) | 1 | Sweeps flame sensor across the environment |
| SG90 Servo Motor (Pump) | 1 | Aims the water pump nozzle toward detected fire |
| 5V Relay Module | 1 | Switches the water pump on/off |
| Mini Water Pump (DC) | 1 | Dispenses water to extinguish the flame |
| Water Reservoir (container) | 1 | Stores water for the pump |
| Silicone tubing | - | Channels water from pump to nozzle |
| Jumper wires, breadboard | - | Wiring and prototyping |
| 9V / USB Power Supply | 1 | Powers the Arduino and peripherals |
Note: All components share a common ground (GND) to ensure stable signal communication between the sensors, servos, and the Arduino.
---

## System Architecture

The system operates as a **finite state machine (FSM)** with two primary states:

```
┌─────────────────────────────────────────────────────────────────┐
│                         SYSTEM START                            │
│               Servos initialized, pump OFF                      │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                      STATE 1: SCAN MODE                         │
│  • Scan servo sweeps from 10° → 170° and back in 5° steps       │
│  • Flame sensor polled at each position                         │
│  • Sensor reads LOW = fire detected                             │
└────────────────────────────┬────────────────────────────────────┘
                             │
                     Fire Detected?
                      (LOW signal)
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                   STATE 2: EXTINGUISH MODE                      │
│  • Record angle of detection                                    │
│  • Aim pump servo to same angle                                 │
│  • Activate relay → pump ON for 3 seconds                       │
│  • Re-check sensor after pump stops                             │
│  • If fire gone → return to SCAN MODE                           │
│  • If fire remains → continue pumping                           │
└─────────────────────────────────────────────────────────────────┘
```

### Pin Assignments

| Arduino Pin | Connected To | Mode |
|---|---|---|
| D2 | IR Flame Sensor (OUT) | INPUT |
| D8 | Relay Module (IN) | OUTPUT |
| D9 | Scan Servo (Signal) | PWM OUTPUT |
| D10 | Pump Servo (Signal) | PWM OUTPUT |

---

## ⚙️ How It Works — Step by Step

1. **Initialization:** On power-up, the scan servo moves to 10° (start position), the pump servo centers at 90°, and the relay is set LOW (pump off).

2. **Scanning:** The `scanForFire()` function moves the scan servo in 5° increments from 10° to 170° and back, pausing 100ms at each position to poll the flame sensor.

3. **Fire Detection:** The IR flame sensor outputs `LOW` when it detects infrared radiation characteristic of a flame. Upon detection, the current servo angle is stored as `fireDetectedAngle`.

4. **Aiming:** The pump servo is immediately commanded to rotate to `fireDetectedAngle`, aligning the nozzle with the detected fire source.

5. **Suppression:** The relay activates (pump ON). The pump runs for `PUMP_DURATION` (3 seconds by default, configurable).

6. **Verification:** After the pump cycle, the sensor is re-read. If the flame is gone, the system resets to scan mode. If the fire persists, the pump reactivates and continues suppressing.

---

## Software & Code

**Language:** C++ (Arduino)  
**IDE:** Arduino IDE 2.x  
**Libraries Used:**
- `Servo.h` — Standard Arduino servo control library (built-in)

### Key Configurable Parameters

```cpp
const int SCAN_MIN_ANGLE = 10;    // Leftmost scan position (degrees)
const int SCAN_MAX_ANGLE = 170;   // Rightmost scan position (degrees)
const int SCAN_STEP = 5;          // Angular resolution of scan
const int SCAN_DELAY = 100;       // Pause per step (ms) — allows sensor to stabilize
const int PUMP_DURATION = 3000;   // Pump run time per cycle (ms)
```

The full source code is available in [`fire_fighting_robot.ino`](./fire_fighting_robot.ino).

---

## Results & Observations

| Test Condition | Outcome |
|---|---|
| Candle flame at ~30 cm distance | Detected and extinguished within ~2 sec |
| Flame at extreme angles (near 10° / 170°) | Successfully detected; servo reached target position |
| Fire already extinguished after first pump cycle | System correctly resumed scan mode |
| Fire persisting after first pump cycle | System correctly ran a second suppression cycle |

**Observations:**
- The 100ms scan delay was critical — reducing it caused false negatives due to sensor settling time
- Servo alignment between the scan angle and pump aim angle was accurate to ±5°
- The system was sensitive to ambient bright light (fluorescent/sunlight), which could cause false positives — a shielded sensor housing would improve robustness

---

## Setup & Replication

1. Wire all components according to the pin assignments table above
2. Clone this repository:
   ```bash
   git clone https://github.com/YOUR-USERNAME/fire-fighting-robot.git
   ```
3. Open `fire_fighting_robot.ino` in Arduino IDE
4. Select **Board:** Arduino Uno, **Port:** your COM port
5. Click **Upload**
6. Open **Serial Monitor** at 9600 baud to observe system status messages

---

## 🚀 Potential Improvements

- [ ] Add a chassis and wheels for a mobile fire-fighting robot
- [ ] Replace IR flame sensor with UV/IR dual-spectrum sensor for outdoor use
- [ ] Add a water level sensor to prevent dry pump operation
- [ ] Implement wireless alerting (ESP8266/GSM) to notify a remote operator
- [ ] Use a PID controller for smoother servo tracking of a moving flame
- [ ] Multi-sensor array for faster and more reliable flame triangulation

---

## Project Context

| Detail | Info |
|---|---|
| Programme | SIWES (Industrial Training) |
| Organization | Hub360 Circuits Ltd, Abuja, Nigeria |
| Year | 2024/2025 |
| Developer | Raphael Ebubechi Efita |
| Institution | Federal University of Technology, Minna |
| Department | Mechatronics Engineering |

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](./LICENSE) file for details.  
Feel free to use, modify, and build upon this work with attribution.

---

## Connect

**Raphael Ebubechi Efita**  
Mechatronics Engineering | Embedded Systems | IoT  
Federal University of Technology, Minna, Nigeria


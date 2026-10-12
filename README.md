# Autonomous Obstacle Avoiding Robot with PWM Motor Control 

An efficient and responsive embedded robotics project featuring dual-speed PWM motor control, ultrasonic distance sensing with dynamic pan scanning, and intelligent obstacle avoidance decision logic.

Built for Arduino-compatible microcontrollers with the **L298N Dual H-Bridge Motor Driver**, an **HC-SR04 Ultrasonic Sensor**, and an **SG90 Pan Servo**.

---

##  Key Features

- **Dual-Speed PWM Motor Control**:
  - Smooth cruising at low speed (`NORMAL_SPEED = 130`) to prevent skidding and preserve battery.
  - High torque and speed for maneuvering (`TURN_SPEED = 230`, `UTURN_SPEED = 230`) to execute crisp 90° and 180° turns.
- **Dynamic Ultrasonic Pan Scanning**:
  - Front path continuously scanned with the **HC-SR04** ultrasonic module.
  - When an obstacle is detected within $\le 30\text{ cm}$, the robot stops and pans the sensor with an **SG90 Micro Servo** to $150^\circ$ (Left) and $30^\circ$ (Right) to evaluate alternate paths.
- **Smart 3-Way Avoidance Logic**:
  - **Left Turn**: Steers left when the left clearance is greater than the right clearance.
  - **Right Turn**: Steers right when the right clearance is greater.
  - **180° Turnaround**: Performs an in-place $180^\circ$ turnaround pivot when both directions are equally obstructed or equally clear ($|Left - Right| \le 5\text{ cm}$).
- **Fail-Safe Sensor Protection**:
  - Validates ultrasonic range ($2.0\text{ cm}$ to $200.0\text{ cm}$).
  - Rejects echo timeouts and invalid readings ($< 0$), safely stopping the chassis rather than executing erratic turns.
- **Dual Format Codebase**:
  - Arduino IDE sketch (`obstacle_avoiding_robot.ino`) and standalone clean C++ source (`obstacle.cpp`) with explicit function prototypes.

---

## 📂 Directory Structure

```yaml
├── README.md                    # Project documentation & wiring guide
├── obstacle.cpp                 # Clean C++ implementation source with declarations
└── obstacle_avoiding_robot.ino  # Main Arduino sketch file
```

---

## 🛠️ Hardware Requirements & Components

| Component | Quantity | Description / Model |
| :--- | :---: | :--- |
| **Microcontroller** | 1 | Arduino Uno / Nano / Mega or compatible AVR board |
| **Motor Driver** | 1 | L298N Dual H-Bridge Motor Driver Module |
| **DC Motors** | 2 | 3V–6V TT DC Gear Motors with Rubber Wheels |
| **Ultrasonic Sensor** | 1 | HC-SR04 Ultrasonic Distance Sensor |
| **Servo Motor** | 1 | SG90 9g Micro Servo |
| **Chassis Kit** | 1 | 2WD Robot Chassis with Front Caster Wheel |
| **Power Supply** | 1 | 7.4V (2x 18650 Li-ion) or 9V–12V Battery Pack |
| **Jumper Wires** | — | Male-to-Male, Male-to-Female jumper cables |

---

## 🔌 Hardware Wiring & Pin Mapping

### 1. Arduino Pin Connections Diagram

```text
                     +----------------------------+
                     |    Arduino Microcontroller |
                     +----------------------------+
                     | Pin 3  (PWM) ---> ENA      | (Left Motor Speed)
                     | Pin 7        ---> IN1      | (Left Motor Dir 1)
                     | Pin 6        ---> IN2      | (Left Motor Dir 2)
                     | Pin 5        ---> IN3      | (Right Motor Dir 1)
                     | Pin 4        ---> IN4      | (Right Motor Dir 2)
                     | Pin 11 (PWM) ---> ENB      | (Right Motor Speed)
                     |                            |
                     | Pin 9        ---> TRIG     | (HC-SR04 Trigger)
                     | Pin 8        ---> ECHO     | (HC-SR04 Echo)
                     |                            |
                     | Pin 10 (PWM) ---> SERVO    | (SG90 Servo Signal)
                     +----------------------------+
```

### 2. Complete Wiring Table

| Module | Pin | Arduino Pin | Power / Connection Details |
| :--- | :--- | :--- | :--- |
| **L298N Driver** | `ENA` | Pin 3 | Left Motor PWM Speed Control (Remove jumper) |
| | `IN1` | Pin 7 | Left Motor Direction 1 |
| | `IN2` | Pin 6 | Left Motor Direction 2 |
| | `IN3` | Pin 5 | Right Motor Direction 1 |
| | `IN4` | Pin 4 | Right Motor Direction 2 |
| | `ENB` | Pin 11 | Right Motor PWM Speed Control (Remove jumper) |
| | `12V` / `VCC` | External Battery (+) | Connected to Battery positive (+7.4V to +12V) |
| | `GND` | Common GND | **Must share common ground with Arduino GND** |
| | `5V Out` | Arduino `5V` or `VIN` | Power for Arduino (if using onboard 5V regulator) |
| **HC-SR04** | `VCC` | 5V | 5V Power Supply |
| | `Trig` | Pin 9 | Ultrasonic Trigger Pulse Output |
| | `Echo` | Pin 8 | Ultrasonic Echo Return Input |
| | `GND` | GND | Common System Ground |
| **SG90 Servo** | `Signal` (Orange/Yellow) | Pin 10 | PWM Servo Control Signal |
| | `VCC` (Red) | 5V | 5V Power Supply |
| | `GND` (Brown/Black) | GND | Common System Ground |

> [!IMPORTANT]
> - Ensure you **remove the onboard jumpers** on the L298N `ENA` and `ENB` pins so the Arduino can apply variable PWM speed control.
> - Always connect all **GND** lines together (Battery (-), L298N GND, and Arduino GND). Without a common ground reference, motor signals and sensor readings will become unstable.

---

## 🧠 System Architecture & Control Logic

### Distance Calculation
The distance from the HC-SR04 is computed by measuring the echo pulse return time:
$$\text{Distance (cm)} = \frac{\text{Duration } (\mu\text{s}) \times 0.0343}{2}$$

A timeout threshold of $12000\ \mu\text{s}$ restricts measuring range to valid obstacles up to $\approx 200\text{ cm}$, preventing blocking stalls. Readings below $2.0\text{ cm}$ or above $200.0\text{ cm}$ are marked as `INVALID` (`-1`).

### State Machine Flowchart

```mermaid
flowchart TD
    Start([Power On / Reset]) --> Init[Initialize Pins, Attach Servo & Stop Motors]
    Init --> CenterServo[Center Servo at 90°]
    CenterServo --> ReadFront[Measure Front Distance]
    
    ReadFront --> CheckValid{Reading Valid?\ncm >= 0}
    CheckValid -- NO --> StopInvalid[Stop Motors & Wait 50ms]
    StopInvalid --> CenterServo

    CheckValid -- YES --> CheckObstacle{Front Distance\n<= 30.0 cm?}
    
    CheckObstacle -- NO --> DriveForward[Drive Forward\nNormal Speed: 130 PWM]
    DriveForward --> LoopWait[Delay 20ms]
    LoopWait --> CenterServo

    CheckObstacle -- YES --> StopScan[Stop Car & Wait 150ms]
    StopScan --> ScanSides[Pan Servo:\nLeft 150° -> Measure\nRight 30° -> Measure\nCenter 90°]
    ScanSides --> CheckScanValid{Both Left & Right\nReadings Valid?}
    
    CheckScanValid -- NO --> StopRetry[Stop Motors & Wait 100ms]
    StopRetry --> CenterServo

    CheckScanValid -- YES --> CheckEqual{abs Left - Right\n<= 5.0 cm?}
    
    CheckEqual -- YES --> TurnAround[Turn Around 180°\nSpeed: 230 PWM | Time: 800ms]
    CheckEqual -- NO --> CheckLeftBetter{Left Distance >\nRight Distance?}
    
    CheckLeftBetter -- YES --> TurnLeft[Turn Left 90°\nSpeed: 230 PWM | Time: 400ms]
    CheckLeftBetter -- NO --> TurnRight[Turn Right 90°\nSpeed: 230 PWM | Time: 400ms]
    
    TurnAround --> Settle[Stop Car, Center Servo & Settle 150ms]
    TurnLeft --> Settle
    TurnRight --> Settle
    Settle --> CenterServo
```

---

## ⚙️ Configuration & Tunable Parameters

All behavioral constants are defined at the top of [obstacle_avoiding_robot.ino](file:///c:/Users/ALIF/OneDrive/Desktop/code/esp32-pathguard/obstacle_avoiding_robot.ino) and [obstacle.cpp](file:///c:/Users/ALIF/OneDrive/Desktop/code/esp32-pathguard/obstacle.cpp):

| Parameter | Default Value | Description |
| :--- | :---: | :--- |
| `NORMAL_SPEED` | `130` | Cruising PWM duty cycle ($0 - 255$) when moving forward. |
| `TURN_SPEED` | `230` | Turning PWM duty cycle ($0 - 255$) during 90° turns. |
| `UTURN_SPEED` | `230` | Turning PWM duty cycle ($0 - 255$) during 180° turnaround. |
| `OBSTACLE_LIMIT` | `30.0` cm | Distance threshold to trigger obstacle avoidance sequence. |
| `EQUAL_LIMIT` | `5.0` cm | Difference between left & right distances to trigger a 180° turnaround. |
| `SERVO_CENTER` | `90`° | Straight-ahead scanning angle for SG90 servo. |
| `SERVO_LEFT` | `150`° | Left pan scanning angle. |
| `SERVO_RIGHT` | `30`° | Right pan scanning angle. |
| `TURN_90_TIME` | `400` ms | Motor burst duration to achieve a 90° pivot turn. |
| `TURN_180_TIME`| `800` ms | Motor burst duration to achieve a 180° turnaround pivot. |

> [!TIP]
> Depending on surface friction (tile vs. carpet) and battery charge level, calibrate `TURN_90_TIME` and `TURN_180_TIME` to achieve exact $90^\circ$ and $180^\circ$ rotations.

---

## 🚀 Installation & Flashing Instructions

### Using Arduino IDE
1. Open **Arduino IDE** (v1.8.x or v2.x).
2. Connect your Arduino board via USB.
3. In Arduino IDE, open [obstacle_avoiding_robot.ino](file:///c:/Users/ALIF/OneDrive/Desktop/code/esp32-pathguard/obstacle_avoiding_robot.ino).
4. Select your board under **Tools $\rightarrow$ Board** (e.g. `Arduino Uno` or `Arduino Nano`).
5. Select your serial port under **Tools $\rightarrow$ Port**.
6. Ensure the `<Servo.h>` library is installed (included by default in Arduino IDE).
7. Click the **Upload** button (`Ctrl + U`).

### Using PlatformIO / C++
If using PlatformIO or native AVR-GCC toolchains, compile with [obstacle.cpp](file:///c:/Users/ALIF/OneDrive/Desktop/code/esp32-pathguard/obstacle.cpp) which includes `<Arduino.h>`, `<Servo.h>`, and all function declarations.

---

## 🔍 Serial Monitor Telemetry

Open the Serial Monitor at **9600 baud** to view live telemetry from the car:

```text
PWM Obstacle Avoiding Car Ready
Front: 84.32 cm
Front: 51.10 cm
Front: 24.65 cm
Left: 48.20 cm | Right: 12.40 cm
Front: 62.15 cm
Front: 18.30 cm
Left: 11.20 cm | Right: 12.00 cm
```

- When front distance is below $30\text{ cm}$, the scan output will display left and right distances.
- If $|Left - Right| \le 5\text{ cm}$, the robot recognizes a tight dead-end or obstacle-blocked corridor and triggers an in-place turnaround.

---

## 🛠️ Troubleshooting & Tips

| Symptom | Probable Cause | Recommended Solution |
| :--- | :--- | :--- |
| **Car turns in reverse or spins when driving forward** | Motor polarity reversed on L298N terminal blocks | Swap the positive and negative wires of that motor on the L298N terminal block, or swap `IN1`/`IN2` (or `IN3`/`IN4`) in the pin definitions. |
| **Arduino restarts or freezes when servo moves** | Current spike on the 5V rail | Servos draw significant current while panning. Power the SG90 from a dedicated 5V buck converter or external regulated supply rather than the Arduino's 5V pin. |
| **Front distance always reads `INVALID`** | Disconnected Echo/Trig pins or bad ground | Check wiring on Pins 8 and 9. Ensure the ultrasonic sensor is supplied with a steady 5V. |
| **Car under-turns or over-turns** | Wheel traction or battery voltage difference | Increase or decrease `TURN_90_TIME` (default: 400ms) and `TURN_180_TIME` (default: 800ms) until turns match 90° and 180°. |
| **Motors buzz but car doesn't move** | `NORMAL_SPEED` PWM too low for motor stall torque | Increase `NORMAL_SPEED` from `130` to `150`–`170` to overcome gear/floor friction. |

---

## 📝 License

This project is licensed under the **MIT License**. Free for educational, hobbyist, and robotics development use!

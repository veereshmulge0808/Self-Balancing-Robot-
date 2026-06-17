🤖 VoxBot: RTOS-Driven Self-Balancing Robot

VoxBot is a high-performance, dynamically stable, two-wheeled inverted pendulum robot. It is designed to solve the "Cognitive Lag" problem in modern edge robotics by completely decoupling high-speed physical stabilization from asynchronous Artificial Intelligence tasks using a strict Dual-Core FreeRTOS architecture.

📖 Table of Contents

The Engineering Problem

Hardware Architecture

Software Architecture (FreeRTOS)

Control Theory & Physics

Wiring & Pin Map

Installation & Setup

Usage (NLP Integration)

Future Roadmap

🛑 The Engineering Problem

Integrating Artificial Intelligence (like Voice Control/NLP) onto dynamically unstable robots usually results in catastrophic failure. Standard single-core microcontrollers pause their balancing control loops to parse incoming wireless commands, causing the robot to fall over.

The VoxBot Solution: Off-board the heavy AI processing to an external application and utilize the ESP32's dual Tensilica Xtensa cores to strictly isolate the robot's physical "reflexes" (PID balancing) from its "consciousness" (Bluetooth Low Energy communication).

🛠️ Hardware Architecture

VoxBot utilizes a 3-Tier Vertical Chassis designed specifically to manipulate the moment of inertia for high-speed (600 RPM) actuators.

Bill of Materials (BOM)

Brain: ESP32 Development Board

IMU: MPU6050 (GY-521) 6-axis Accelerometer/Gyroscope

Actuators: 2x N20 Micro Metal Gearmotors (600 RPM "Sprinter" profile) with Encoders

Motor Driver: DRV8833 Dual H-Bridge

Power: 7.4V Li-ion Battery Pack (2x 18650 cells)

Regulation: LM2596 DC-DC Buck Converter (Stepped down to isolated 5.0V)

Wheels: 43mm D-shaft rubber wheels (to maximize torque leverage)

Center of Gravity (CoG) Optimization

Because 600 RPM motors have exceptionally low torque, the robot's "fall rate" must be slowed down. The heaviest component (the 7.4V battery pack) is mounted on the Middle Tier, raising the CoG and giving the high-speed motors the crucial milliseconds needed to recover balance without stalling.

🧠 Software Architecture (FreeRTOS)

The system utilizes FreeRTOS to prevent thread blocking and ensure microsecond-precision stability.

Core 1 ("The Reflexes"): Pinned at Priority 2 (Highest). Runs a strict 200Hz (5ms) cascaded PID balancing loop. Reads the IMU, calculates trigonometric pitch, and drives the hardware PWM.

Core 0 ("The Consciousness"): Pinned at Priority 1. Runs an asynchronous BLE Server. Listens for intent strings (e.g., DRIVE_FORWARD) from the external NLP application.

Inter-Process Communication (IPC): Commands are passed from Core 0 to Core 1 using thread-safe FreeRTOS Queues. Mutexes are strictly forbidden to prevent memory locks from crashing the 200Hz balancing loop.

🧮 Control Theory & Physics

Sensor Fusion

Pitch is calculated purely via gravity vectors on the accelerometer, smoothed by the MPU6050's internal 21Hz Digital Low-Pass Filter (DLPF) to reject motor vibration.

Pitch = atan2(Ax, sqrt(Ay^2 + Az^2)) * (180 / PI)


600 RPM Custom PID Tuning

Standard PID values will destabilize 600 RPM motors. VoxBot uses a custom algorithm:

Dampened $K_p$: Kept low to prevent violent over-correction and high-frequency shaking.

Elevated $K_d$: Acts as an electronic brake to smoothly decelerate the fast gears as the robot approaches $0.0^\circ$.

Deadband Compensation: Hardcoded minimum PWM jumps (e.g., 15/255) to instantly overcome N20 gearbox static friction.

🔌 Wiring & Pin Map

⚠️ CRITICAL: The ESP32 and MPU6050 must be powered by the 5.0V output of the LM2596. The motors are powered directly by the 7.4V battery. All components must share a common ground.

Component

Pin / Pad

ESP32 Connection

MPU6050

SDA

GPIO 21



SCL

GPIO 22

DRV8833

IN1 (Left Motor Fwd)

GPIO 32 (via ledc PWM)



IN2 (Left Motor Rev)

GPIO 33 (via ledc PWM)



IN3 (Right Motor Fwd)

GPIO 25 (via ledc PWM)



IN4 (Right Motor Rev)

GPIO 26 (via ledc PWM)

🚀 Installation & Setup

Assemble the chassis ensuring extreme rigidity to prevent the "bobblehead" effect.

Open the .ino file in the Arduino IDE.

Install the required libraries via the Library Manager:

Adafruit MPU6050

Adafruit Unified Sensor

Set the Board to DOIT ESP32 DEVKIT V1 and upload the firmware.

Manually hold the robot vertical upon power-up to allow the IMU to calibrate.

🗣️ Usage (NLP Integration)

VoxBot acts as a BLE Peripheral. It expects string commands sent to its custom characteristic.

Connect to the device named VoxBot_AI via Bluetooth.

Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b

Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8

Write string payloads to control the robot:

"DRIVE_FORWARD" (Applies a velocity offset, forcing the robot to lean and drive forward)

"STOP" (Returns the velocity offset to 0.0)

🔮 Future Roadmap

[ ] Implement Reinforcement Learning (Proximal Policy Optimization) for auto-tuning PID gains via PyBullet digital twin.

[ ] Upgrade the off-board application to include YOLO object-tracking for visual follow-me capabilities over BLE.

<img width="720" height="1280" alt="image" src="https://github.com/user-attachments/assets/b38b83b2-243f-4d93-9174-af16b3975209" />

<img width="720" height="1280" alt="image" src="https://github.com/user-attachments/assets/c05cee62-a946-41bd-bda2-7757c984e8ea" />

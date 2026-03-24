🛠 Self-Balancing Scooter Control System

An STM32-based real-time self-balancing scooter implementing an inverted pendulum control system with cascaded PID controllers, sensor fusion, and wireless control.

🚀 Overview

This project implements a two-wheel self-balancing scooter using an STM32 microcontroller.
The system maintains upright balance by continuously estimating the tilt angle and applying motor corrections in real time.

Key features include:

Real-time control loop (5 ms)
Cascaded PID control (attitude, velocity, steering)
IMU + encoder sensor fusion
Wireless control via BLE
Safety mechanisms (emergency stop, saturation protection)

🧠 Control Strategy

The system uses a cascaded control structure:

Inner loop (PD) → stabilizes tilt angle (fast response)
Outer loop (PI) → regulates velocity
Steering loop (PI) → controls turning behavior

The control loop runs at 5 ms intervals, triggered by the MPU6050 interrupt for deterministic timing.

📡 Sensor Fusion
IMU (MPU6050) provides tilt angle and angular velocity
Encoder provides wheel speed feedback
A complementary filter is used to combine accelerometer and gyroscope data for stable angle estimation
🔧 Hardware
STM32F103 (Blue Pill)
MPU6050 IMU
Motor driver (e.g., L298N / TB6612)
DC motors with encoders
JDY-31 BLE module (UART)
Battery power system
📲 Features
Wireless control via BLE (forward / backward / turn)
Emergency stop for safety
Motor saturation limits to prevent instability
Real-time debugging via UART
🧪 Performance
Stable balancing under small disturbances
Control loop latency: ~5 ms
Responsive to user input via BLE

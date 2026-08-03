# AI-Based Autonomous Parking System for Mobile Robot

An autonomous mobile robot designed to navigate indoor environments, detect designated parking bays using computer vision, evaluate parking constraints in real time, and execute precise self-parking routines without human intervention. 

---

## 📌 Project Overview

This project presents a dual-tier distributed computing architecture for an autonomous parking robot. High-level computer vision and state management are processed on a **Raspberry Pi 5** via ROS 2, while real-time low-level motor actuation and encoder tracking are offloaded to a **Raspberry Pi Pico 2**. Sensor fusion between an IMU gyroscope and optical wheel encoders eliminates rotational drift to achieve precise alignment and parking.

---

## 🛠️ System Architecture

### 🛠️ Hardware Components

| Subsystem | Component | Description / Role |
| :--- | :--- | :--- |
| **High-Level Processing** | Raspberry Pi 5 | Runs ROS 2 (Jazzy/Humble), computer vision pipelines, and FSM logic |
| **Microcontroller** | Raspberry Pi Pico 2 | Low-level real-time motor control and optical encoder tracking |
| **Sensors** | 10 DOF Waveshare IMU (MPU6050) | Gyroscope turning angle measurement to eliminate rotational drift |
| | Onboard USB Camera | Visual input for ArUco tag detection and HSV line color masking |
| | Optical Wheel Encoders | Quadrature tick integration for linear distance measurement |
| **Actuation** | 4× DC Motors | Primary drive system for omnidirectional/differential movement |
| | 4× IBT-2 Motor Drivers | H-Bridge drivers for high-current PWM motor control |
| **Power System** | 16.8V Li-Ion Battery Pack | 4S Sony 18650 cell configuration managed by an onboard BMS |
| | 5V / 5A DC-DC Buck Converters | Voltage regulation for single-board computers and sensor logic |

### Software & Networking
* **OS / Framework**: Linux (Ubuntu), ROS 2 (Jazzy / Humble), OpenCV[cite: 1]
* **Networking**: Headless operation via peer-to-peer Tailscale VPN using Cyclone DDS middleware over SSH[cite: 1]

---

## ⚙️ Methodology & Algorithmic Pipeline

### 1. Sensor Fusion Odometry
To counteract mechanical wheel slip during turns, the navigation stack decouples linear and angular motion tracking[cite: 1]:
* **Linear Distance**: Calculated via wheel encoder tick integration[cite: 1].
* **Angular Orientation**: Tracked using the integrated MPU6050 IMU gyroscope[cite: 1].

### 2. Finite State Machine (FSM) Logic
The autonomous decision pipeline executes across four main states[cite: 1]:
1. **ArUco Tag Hunting**: Rotates $360^\circ$ to locate target ArUco markers[cite: 1].
2. **Alignment & Approach**: Aligns the vehicle heading with the target bay[cite: 1].
3. **Boundary Measurement**: Uses HSV color masking to detect red line parking constraints[cite: 1].
4. **Autonomous Parking / Rejection**: Evaluates space clearance to either perform the parking maneuver or reject the spot[cite: 1].

---

## 📂 Repository Structure

```text
├── config/              # ROS 2 parameter files & HSV color thresholds
├── launch/              # ROS 2 launch files
├── nodes/               # ROS 2 Python / C++ nodes
│   ├── vision_node.py   # OpenCV ArUco & HSV masking pipeline
│   ├── fsm_node.py      # Main Finite State Machine controller
│   └── imu_fusion.py    # Sensor fusion odometry node
├── pico_firmware/       # Raspberry Pi Pico 2 C++/MicroPython motor controller
├── docs/                # Project documentation & report
└── README.md

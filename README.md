# Sensor-Based Automated Rain Protection System for Clothesline  
### Dual-Microcontroller IoT-Enabled Smart Automation

This repository contains the complete implementation of a **sensor-based automated rain protection system for clotheslines** using a **dual-microcontroller architecture**.  
The system enables **automatic rain-triggered protection** as well as **manual remote control** through a mobile application.

The design is **universally adaptable** and can be installed on existing clothesline setups without structural modification.

---

## Project Overview

### Key Features
- Automatic rain detection and cloth protection
- Mobile-based control using **Blynk IoT**
- Dual operation modes  
  - **Autonomous mode** (rain-based)
  - **Manual mode** (user-controlled)
- Hall-effect sensors for precise position detection
- UART-based communication between controllers
- Smart home–ready IoT architecture
- Low-cost, modular, and scalable design

---

## System Architecture

### Dual-Microcontroller Design
| Controller | Function |
|-----------|----------|
| **STM32F401CCU6** | Real-time control, sensor processing, motor control |
| **ESP32** | Wi-Fi connectivity, Blynk interface, remote commands |

### Sensors & Actuators
- Rain sensor (ADC-based)
- Hall-effect sensors (home & end positions)
- DC motors with L298N motor driver
- Rope-based longitudinal and lateral guiding mechanism

---

## Hardware Components

| Component | Quantity |
|---------|----------|
| STM32F401CCU6 Development Board | 1 |
| ESP32 Development Board | 1 |
| Rain Sensor Module | 1 |
| Hall-Effect Sensors (A3144) | 2 |
| DC Motors (300 RPM) | 2 |
| L298N Motor Driver Module | 1 |
| 12V Power Supply | 1 |
| Buck Converter | 1 |



## Software Stack

### STM32 Firmware
- Language: **Embedded C**
- Programming: **Register-level**
- Peripherals used:
  - ADC (Rain sensor input)
  - UART (ESP32 communication)
  - GPIO (Motor driver & Hall sensors)
  - Timers (Periodic sampling)
- Architecture: **Interrupt-driven, non-blocking**

### ESP32 Firmware
- Framework: **Arduino**
- IoT Platform: **Blynk**
- Communication: UART with STM32
- Responsibilities:
  - Mode selection
  - Motor control commands
  - Status updates

---

## Mobile Application (Blynk)

- Auto / Manual mode toggle
- Forward / Reverse / Stop controls
- Real-time system feedback




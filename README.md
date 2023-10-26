# Autonomous Hovercraft Maze Solver

This repository contains the code for an autonomous hovercraft built as a school project for a competition. The objective of the project was to design a hovercraft capable of navigating through a predetermined maze using autonomous algorithms. The challenge was to achieve this without relying on predetermined actions and ensuring the hovercraft could successfully complete the maze from various initial positions.

## Components Used

- 2 Fans
- Servo Motor
- MPU Sensor (for yaw detection)
- Ultrasonic (US) Sensor (for wall detection)
- Arduino Nano
- ATmega382p Board

## System Overview

The hovercraft's autonomous navigation is achieved through a combination of hardware components and custom-written C++ code. Here's a brief overview of the system:

- **Fans**: Provide thrust for propulsion.
- **Servo Motor**: Controls the direction of thrust. It is also used to position the Ultrasonic (US) Sensor to detect walls when the hovercraft comes to a stop.
- **MPU Sensor**: Measures yaw, enabling precise 90-degree turns.
- **US Sensor**: Detects obstacles/walls when the hovercraft comes to a stop.
- **Arduino Nano**: Acts as the microcontroller for the system.
- **ATmega382p Board**: Provides additional processing power and capabilities.

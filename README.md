<div align='center'><img src='https://github.com/AbdoullahBougataya/penduleum-system/blob/main/img/system.jpeg' alt="System" width="350" height="350" style="display: block; margin: 0 auto"/></div>

# Bi-actuated self-balancing aero-penduleum system

> [!TIP]
> This codebase is compatible with the Arduino UNO board

## Overview

This repository is the software part of a Bi-actuated self-balancing aero-penduleum system. A bi-actuated pendulum is a system that uses two BLDC to control the position of a pendulum. The system is designed to balance the pendulum in an flat 0° position. The system uses a combination of sensors (Gyroscope + Accelerometer) and actuators to achieve this goal.

## Project structure
``` sh
/penduleum-system
├── /include
│     ├── BMI160.h
│     └── RCFilter.h
├── /src
│     ├── BMI160.cpp
│     └── RCFilter.cpp
├── penduleum-system.ino
├── LICENSE
├── README.md
└── .gitignore
```

## Features

- Real-time balance stabilization using **PIV control**.
- Supports the **BMI160 IMU** (**I**nertial **M**easurement **U**nit) sensor.
- Simple sensor communications **using I²C**.
- Minimum modifications required for cross-platform compatibility with **Arduino language** compatible boards (UNO, MEGA, ESP32, Teensy...).

## Getting Started

Follow these steps to set up and run the penduleum software.

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software)
- Required hardware:
  - Microcontroller (e.g., [Arduino UNO](https://store.arduino.cc/products/arduino-uno-rev3)...)
  - [BMI160](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) IMU.
  - Two BLDC motors with ESCs (Electronic Speed Controllers).
  - Power supply (e.g., LiPo battery).
  - Jumper wires for connections.
  - Mechanical structure for the pendulum.

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/AbdoullahBougataya/penduleum-system.git
   ```
2. Open the project in your preferred IDE.
3. Compile and upload the code to the controller.

## Usage
1. Connect the **BMI160** IMU to the microcontroller using I²C (SDA, SCL).
   - Ensure the IMU is powered correctly.
   - Connect the two ESCs to the appropriate pins on the controller.
2. Power on the controller and **wait 10 seconds** for the sensors and the ESCs to calibrate.

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork** the repository.
2. Create a new branch for your *feature* or *bugfix*.
3. **Submit a pull request** with a detailed description of your changes.

## License

This project is licensed under the **GPL v2.0 License**. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Carbon aeronautics** series on making a Quadcopter using Teensy (Arduino compatible).
- **Phil's Lab** series on DSP using STM32.


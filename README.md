Pi Interface (RP2350-Based Mechatronics Controller)
<img src="hardware/Pi_Interface_V1.jpg" width="300"/>
Overview

The Pi Interface is an RP2350-based mechatronics controller designed for robotics, CNC systems, and industrial automation. The board integrates an MKS MINI12864 V3.0 graphical LCD, motor drive interfaces, and an industrial-grade power architecture, while supporting MicroPython and CircuitPython for rapid development and research-oriented prototyping.

By leveraging Python’s extensive ecosystem, the controller enables seamless integration of motion control, sensor acquisition, human–machine interfaces (HMI), and IoT connectivity, effectively bridging academic research and industrial applications.

Key Features

Microcontroller: RP2350

Programming: MicroPython / CircuitPython

Display Interface:

MKS MINI12864 V3.0 (ST7567-based, SPI)

Real-time system monitoring and user interaction

Communication Interfaces:

SPI

I2C

UART

PWM

ADC

Motion Control:

Stepper motors (e.g., A4988, TMC2209)

DC motors

Servo motors

Sensor Support:

Industrial analog sensors

Digital sensors via I2C/SPI/UART

Application Domains:

Robotics

CNC & motion platforms

Smart manufacturing

Research and education

IoT-enabled automation systems

Pinout Diagram

Recommended Libraries

To run the provided examples and fully utilize the controller, install the following libraries:

Core Dependencies

ST7567 Display Driver
https://github.com/ChangboBro/ST7567-micropython-framebuff.git

Used for SPI communication with the MKS MINI12864 V3.0 LCD.

encoderLib
https://github.com/BramRausch/encoderLib.git

Provides rotary encoder support for motion feedback and user input.

picozero
https://github.com/RaspberryPiFoundation/picozero.git

Simplifies GPIO, PWM, motor, servo, button, and buzzer control.

Software Architecture & Peripheral Support
1. Display & User Interface

LCD Display

ST7567-based SPI display

Real-time graphical feedback (status, menus, diagnostics)

Buttons

Managed via picozero for debounced GPIO input

Buzzer

Controlled using PWM via picozero

Suitable for alerts and system notifications

2. Motor Control

Stepper Motors

Compatible with external stepper drivers (A4988, TMC2209, etc.)

Step/Dir control via GPIO

DC Motors

Speed and direction control using PWM and GPIO

Implemented via picozero

Servo Motors

Standard RC servo control using PWM

Managed via picozero

Dedicated PWM Outputs

GP16, GP17, GP18, GP19

3. Sensor Inputs & Analog Channels

ADC Inputs

GP26 → ADC0

GP27 → ADC1

GP28 → ADC2

Industrial Sensors

Voltage, pressure, force, and other analog sensors

Read using the machine module for precise acquisition

4. Communication Interfaces

SPI

High-speed peripheral communication (LCD, sensors)

I2C

Sensor arrays and expansion modules

UART

Serial communication with external controllers and devices

All interfaces are configured using the standard machine module in MicroPython/CircuitPython.

Installation & Setup

Follow the official picozero installation guide to get started:

👉 https://picozero.readthedocs.io/en/latest/gettingstarted.html

After installation:

Flash MicroPython or CircuitPython to the RP2350

Install required libraries

Upload example scripts

Power the board and verify LCD, motors, and sensors

Use Cases

Research prototypes for robotics and automation

CNC motion control and machine experimentation

Smart factory and Industry 4.0/5.0 testbeds

Educational platforms for embedded systems and mechatronics

Rapid development of Python-based industrial controllers****

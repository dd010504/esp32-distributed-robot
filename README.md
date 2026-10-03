# ESP32 Distributed Robot

A distributed robotics control platform built using three ESP32
microcontrollers.

## Architecture

### ESP32 #1 — Master Node

Responsible for:

- OLED user interface
- joystick input
- operating modes
- ESP-NOW coordination
- Wi-Fi dashboard
- system telemetry

### ESP32 #2 — Sensor Node

Responsible for:

- ultrasonic distance sensing
- temperature/humidity
- motion detection
- light sensing
- IMU data
- environmental telemetry

### ESP32 #3 — Actuator Node

Responsible for:

- servo control
- stepper motors
- LEDs
- buzzer
- relays
- physical robot actions

## Communication

ESP32 nodes communicate using ESP-NOW.

Future support will include:

- Wi-Fi dashboard
- telemetry logging
- Mega 2560 I/O controller
- autonomous operating modes
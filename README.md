# ARM Robotic Arm

## Overview
ARM is a Raspberry Pi + ESP32 robotic arm project. The Raspberry Pi handles high-level control, controller input, GUI, and future chess logic. The ESP32 handles low-level servo control, timing, limits, and safety behavior.

The current focus is V1: Controller Mode.

## Current Scope

### V1: Controller Mode
V1 is a manually controlled robotic arm. The arm is controlled from a game controller through the Raspberry Pi, while the ESP32 handles servo movement, limits, ramping, and basic safety behavior.

### V2: Chess Mode
V2 will add chess-playing functionality on top of the completed controller-mode platform.

## Current Features
- Raspberry Pi controller input
- UART communication from Raspberry Pi to ESP32
- ESP32 servo control
- Servo limits
- Servo ramping / controlled movement
- GUI/home screen in progress
- Future current sensing and fault reporting

## Project Structure

```text
trunk/
  pi/
    system/     Raspberry Pi control scripts
    GUI/        GUI prototype

  esp/
    servo_controller/   ESP32 firmware

docs/
  project_status.md
  versions_scope.md
  design_notes.md
# Design Notes

## System Overview
The Raspberry Pi handles high-level control, controller input, GUI, and command generation. The ESP32 handles time-sensitive servo control, movement limits, and low-level hardware behavior.

## System Design

```text
Xbox Controller
      ↓
Raspberry Pi Python App
      ↓
Regular UART Communication Packets
      ↓
ESP32 Firmware
      ↓
Servo Control / Current Monitoring
      ↓
Physical Arm Servos
```

## Current Hardware
- Raspberry Pi 4:
- ESP32-WROOM-32:
- Servos:
- Power supply:
- Buck converter:
- Camera:
- ToF sensor:
- Controller:
- Current sensing hardware:

## Current Software

### Raspberry Pi
- Language:
- Main scripts:
- Controller input:
- UART communication:
- GUI status:

### ESP32
- Framework:
- PWM/servo driver:
- UART receive:
- Control loop:
- Current sensing:
- Fault handling:

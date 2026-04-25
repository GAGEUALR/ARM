# Project Status

## Project Goal
Build a Raspberry Pi + ESP32 controlled robotic arm platform for manual controller-based operation, with later expansion toward chess/autonomous manipulation.

## What Currently Works
- Pi boots/runs main control program
- ESP32 builds/flashes
- Pi connects to ESP32 over UART
- Controller is detected
- Controller commands move servos
- Arm can move safely under manual control
- Servo limits are enforced
- Generic GUI displays non-real status

## Known Software Problems
- No home/startup position
- Servo speed/ramp behavior needs tuning
- No reliable return packet from esp32
- Manual control is difficult because movement is too fast
- UART packet loss/recovery behavior is not fully validated
- Control-loop timing may not be stable under full load

## Hardware/Mechanical ToDo List:
- Need secure wiring path for servos (they're left dangling as of now)
- No casing for the pi, esp, power components. Right now it's breadboarded

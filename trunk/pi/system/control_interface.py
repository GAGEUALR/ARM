import time
import serial
from evdev import InputDevice, ecodes

EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"

RT_CODE = ecodes.ABS_RZ
LT_CODE = ecodes.ABS_Z
RSX_CODE = ecodes.ABS_RX

ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200

SEND_HZ = 50
SEND_DT = 1.0 / SEND_HZ

PI_TRIGGER_DEADBAND_0_1023 = 60
PI_STICK_DEADBAND_SIGNED = 200

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def normalize_to_0_1023(value, vmin, vmax):
    value = clamp(value, vmin, vmax)
    if vmax == vmin:
        return 0
    scaled = int((value - vmin) * 1023 / (vmax - vmin))
    return clamp(scaled, 0, 1023)

def normalize_stick_to_signed_1023(value, vmin, vmax):
    value = clamp(value, vmin, vmax)
    if vmax == vmin:
        return 0

    center = (vmin + vmax) / 2.0
    span = (vmax - vmin) / 2.0
    if span <= 0:
        return 0

    scaled = int(((value - center) * 1023.0) / span)
    return clamp(scaled, -1023, 1023)

def apply_signed_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0
    return value

def main():
    dev = InputDevice(EVENT_PATH)

    if dev.name != REQUIRED_NAME:
        raise RuntimeError(
            f"{EVENT_PATH} name mismatch. Expected '{REQUIRED_NAME}', got '{dev.name}'"
        )

    rt_info = dev.absinfo(RT_CODE)
    lt_info = dev.absinfo(LT_CODE)
    rsx_info = dev.absinfo(RSX_CODE)

    rt_min, rt_max = rt_info.min, rt_info.max
    lt_min, lt_max = lt_info.min, lt_info.max
    rsx_min, rsx_max = rsx_info.min, rsx_info.max

    print(f"Using controller: {EVENT_PATH} name='{dev.name}'")
    print(f"RT  ABS_RZ range=({rt_min},{rt_max})")
    print(f"LT  ABS_Z  range=({lt_min},{lt_max})")
    print(f"RSX ABS_RX range=({rsx_min},{rsx_max})")

    ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)
    print(f"Connected to ESP32 on {ESP_PORT} @ {ESP_BAUD}")

    rt_0_1023 = 0
    lt_0_1023 = 0
    rsx_signed = 0

    last_send = 0.0
    last_rt_sent = None
    last_lt_sent = None
    last_rsx_sent = None

    dev.grab()

    try:
        for event in dev.read_loop():
            if event.type != ecodes.EV_ABS:
                continue

            updated = False

            if event.code == RT_CODE:
                rt_0_1023 = normalize_to_0_1023(event.value, rt_min, rt_max)
                if rt_0_1023 < PI_TRIGGER_DEADBAND_0_1023:
                    rt_0_1023 = 0
                updated = True

            elif event.code == LT_CODE:
                lt_0_1023 = normalize_to_0_1023(event.value, lt_min, lt_max)
                if lt_0_1023 < PI_TRIGGER_DEADBAND_0_1023:
                    lt_0_1023 = 0
                updated = True

            elif event.code == RSX_CODE:
                rsx_signed = normalize_stick_to_signed_1023(event.value, rsx_min, rsx_max)
                rsx_signed = apply_signed_deadband(rsx_signed, PI_STICK_DEADBAND_SIGNED)
                updated = True

            now = time.time()

            if updated and (now - last_send >= SEND_DT):
                if rt_0_1023 != last_rt_sent:
                    ser.write(f"RT={rt_0_1023}\n".encode())
                    last_rt_sent = rt_0_1023

                if lt_0_1023 != last_lt_sent:
                    ser.write(f"LT={lt_0_1023}\n".encode())
                    last_lt_sent = lt_0_1023

                if rsx_signed != last_rsx_sent:
                    ser.write(f"RSX={rsx_signed}\n".encode())
                    last_rsx_sent = rsx_signed

                last_send = now

    finally:
        try:
            dev.ungrab()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
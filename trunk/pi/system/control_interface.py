import time
import serial

from evdev import InputDevice, ecodes

EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"

# Analog inputs
LT_CODE = ecodes.ABS_Z
RT_CODE = ecodes.ABS_RZ
LSX_CODE = ecodes.ABS_X
RSX_CODE = ecodes.ABS_RX

# D-pad left/right is usually ABS_HAT0X on Linux
DPAD_X_CODE = ecodes.ABS_HAT0X

# Bumpers
LB_CODE = ecodes.BTN_TL
RB_CODE = ecodes.BTN_TR

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


def build_active_commands(lt_0_1023, rt_0_1023, lsx_signed, rsx_signed, dpx_signed, lb_pressed, rb_pressed):
    active = {}

    # Base from LSX
    if lsx_signed < 0:
        active["B"] = 0
    elif lsx_signed > 0:
        active["B"] = 1

    # Shoulder from RSX
    if rsx_signed < 0:
        active["S"] = 0
    elif rsx_signed > 0:
        active["S"] = 1

    # Forearm from bumpers
    if lb_pressed and not rb_pressed:
        active["F"] = 0
    elif rb_pressed and not lb_pressed:
        active["F"] = 1

    # Wrist from d-pad
    if dpx_signed < 0:
        active["W"] = 0
    elif dpx_signed > 0:
        active["W"] = 1

    # Gripper from triggers
    if lt_0_1023 > 0 and rt_0_1023 == 0:
        active["G"] = 0
    elif rt_0_1023 > 0 and lt_0_1023 == 0:
        active["G"] = 1

    return active


def update_press_order(active_now, press_order):
    keys_to_remove = [key for key in press_order if key not in active_now]
    for key in keys_to_remove:
        press_order.remove(key)

    for key in active_now:
        if key not in press_order:
            press_order.append(key)


def build_command_line(active_now, press_order):
    chosen = []

    for key in press_order:
        if key in active_now:
            chosen.append((key, active_now[key]))
        if len(chosen) == 2:
            break

    if len(chosen) == 0:
        return None

    if len(chosen) == 1:
        key1, val1 = chosen[0]
        key2, val2 = chosen[0]
        return f"{key1}={val1},{key2}={val2}\n"

    key1, val1 = chosen[0]
    key2, val2 = chosen[1]
    return f"{key1}={val1},{key2}={val2}\n"


def main():
    dev = InputDevice(EVENT_PATH)

    if dev.name != REQUIRED_NAME:
        raise RuntimeError(
            f"{EVENT_PATH} name mismatch. Expected '{REQUIRED_NAME}', got '{dev.name}'"
        )

    lt_info = dev.absinfo(LT_CODE)
    rt_info = dev.absinfo(RT_CODE)
    lsx_info = dev.absinfo(LSX_CODE)
    rsx_info = dev.absinfo(RSX_CODE)

    lt_min, lt_max = lt_info.min, lt_info.max
    rt_min, rt_max = rt_info.min, rt_info.max
    lsx_min, lsx_max = lsx_info.min, lsx_info.max
    rsx_min, rsx_max = rsx_info.min, rsx_info.max

    ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)

    print(f"Using controller: {EVENT_PATH} name='{dev.name}'")
    print(f"LT   ABS_Z     range=({lt_min},{lt_max})")
    print(f"RT   ABS_RZ    range=({rt_min},{rt_max})")
    print(f"LSX  ABS_X     range=({lsx_min},{lsx_max})")
    print(f"RSX  ABS_RX    range=({rsx_min},{rsx_max})")
    print(f"DPX  ABS_HAT0X")
    print(f"LB   BTN_TL")
    print(f"RB   BTN_TR")
    print(f"Connected to ESP32 on {ESP_PORT} @ {ESP_BAUD}")
    print("Sending command format like: B=0,S=1")

    lt_0_1023 = 0
    rt_0_1023 = 0
    lsx_signed = 0
    rsx_signed = 0
    dpx_signed = 0
    lb_pressed = 0
    rb_pressed = 0

    press_order = []
    last_send = 0.0
    last_line_sent = None

    dev.grab()

    try:
        for event in dev.read_loop():
            updated = False

            if event.type == ecodes.EV_ABS:
                if event.code == LT_CODE:
                    lt_0_1023 = normalize_to_0_1023(event.value, lt_min, lt_max)
                    if lt_0_1023 < PI_TRIGGER_DEADBAND_0_1023:
                        lt_0_1023 = 0
                    updated = True

                elif event.code == RT_CODE:
                    rt_0_1023 = normalize_to_0_1023(event.value, rt_min, rt_max)
                    if rt_0_1023 < PI_TRIGGER_DEADBAND_0_1023:
                        rt_0_1023 = 0
                    updated = True

                elif event.code == LSX_CODE:
                    lsx_signed = normalize_stick_to_signed_1023(event.value, lsx_min, lsx_max)
                    lsx_signed = apply_signed_deadband(lsx_signed, PI_STICK_DEADBAND_SIGNED)
                    updated = True

                elif event.code == RSX_CODE:
                    rsx_signed = normalize_stick_to_signed_1023(event.value, rsx_min, rsx_max)
                    rsx_signed = apply_signed_deadband(rsx_signed, PI_STICK_DEADBAND_SIGNED)
                    updated = True

                elif event.code == DPAD_X_CODE:
                    dpx_signed = clamp(int(event.value), -1, 1)
                    updated = True

            elif event.type == ecodes.EV_KEY:
                if event.code == LB_CODE:
                    lb_pressed = 1 if event.value else 0
                    updated = True

                elif event.code == RB_CODE:
                    rb_pressed = 1 if event.value else 0
                    updated = True

            now = time.time()

            if updated and (now - last_send >= SEND_DT):
                active_now = build_active_commands(
                    lt_0_1023,
                    rt_0_1023,
                    lsx_signed,
                    rsx_signed,
                    dpx_signed,
                    lb_pressed,
                    rb_pressed
                )

                update_press_order(active_now, press_order)
                line = build_command_line(active_now, press_order)

                if line is not None and line != last_line_sent:
                    ser.write(line.encode())
                    ser.flush()
                    last_line_sent = line

                if line is None:
                    last_line_sent = None

                last_send = now

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Sending SHUTDOWN...")
        ser.write(b"SHUTDOWN\n")
        ser.flush()
        time.sleep(0.25)

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
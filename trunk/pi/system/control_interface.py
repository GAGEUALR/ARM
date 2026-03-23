import time
import select
import serial

from evdev import InputDevice, ecodes

EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"

LT_CODE = ecodes.ABS_Z
RT_CODE = ecodes.ABS_RZ
LSX_CODE = ecodes.ABS_X
RSX_CODE = ecodes.ABS_RX
DPAD_X_CODE = ecodes.ABS_HAT0X
LB_CODE = ecodes.BTN_TL
RB_CODE = ecodes.BTN_TR

ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200

SEND_HZ = 100
SEND_DT = 1.0 / SEND_HZ

START_BYTE = 0xAA
SERVO_ORDER = ("B", "S", "F", "W", "G")
NEUTRAL = None

TRIGGER_THRESHOLD = 80
STICK_DEADBAND = 12000


def clamp(v, lo, hi):
    if v < lo:
        return lo

    if v > hi:
        return hi

    return v


def stick_direction(raw_value, center=0, deadband=STICK_DEADBAND):
    distance_from_center = raw_value - center

    if distance_from_center < -deadband:
        return 0

    if distance_from_center > deadband:
        return 1

    return NEUTRAL


def dpad_direction(raw_value):
    limited_value = clamp(int(raw_value), -1, 1)

    if limited_value < 0:
        return 0

    if limited_value > 0:
        return 1

    return NEUTRAL


def trigger_direction(lt_value, rt_value, threshold=TRIGGER_THRESHOLD):
    left_trigger_active = lt_value > threshold
    right_trigger_active = rt_value > threshold

    if left_trigger_active and not right_trigger_active:
        return 0

    if right_trigger_active and not left_trigger_active:
        return 1

    return NEUTRAL


def button_pair_direction(negative_pressed, positive_pressed):
    if negative_pressed and not positive_pressed:
        return 0

    if positive_pressed and not negative_pressed:
        return 1

    return NEUTRAL


def build_commands(lt_value, rt_value, lsx_value, rsx_value, dpx_value, lb_pressed, rb_pressed):
    raw_commands = {
        "B": stick_direction(lsx_value),
        "S": stick_direction(rsx_value),
        "F": button_pair_direction(lb_pressed, rb_pressed),
        "W": dpad_direction(dpx_value),
        "G": trigger_direction(lt_value, rt_value)
    }

    return raw_commands


def update_press_order(raw_commands, press_order):
    inactive_servos = [servo_name for servo_name in press_order if raw_commands.get(servo_name) is None]

    for servo_name in inactive_servos:
        press_order.remove(servo_name)

    for servo_name in SERVO_ORDER:
        if raw_commands[servo_name] is not None and servo_name not in press_order:
            press_order.append(servo_name)


def choose_two_oldest(raw_commands, press_order):
    chosen_commands = {}

    for servo_name in press_order:
        if raw_commands.get(servo_name) is not None:
            chosen_commands[servo_name] = raw_commands[servo_name]

            if len(chosen_commands) == 2:
                break

    return chosen_commands


def build_full_state(chosen_commands):
    full_state = {
        "B": NEUTRAL,
        "S": NEUTRAL,
        "F": NEUTRAL,
        "W": NEUTRAL,
        "G": NEUTRAL
    }

    for servo_name, servo_value in chosen_commands.items():
        full_state[servo_name] = servo_value

    return full_state


def build_packet(full_state):
    packet_bytes = bytearray()
    packet_bytes.append(START_BYTE)

    for servo_name in SERVO_ORDER:
        packet_bytes.append(ord(servo_name))

        servo_value = full_state[servo_name]
        servo_flags = 0

        if servo_value in (0, 1):
            servo_flags |= 0x01

            if servo_value == 1:
                servo_flags |= 0x02

        packet_bytes.append(servo_flags)

    checksum_value = 0

    for packet_byte in packet_bytes:
        checksum_value ^= packet_byte

    packet_bytes.append(checksum_value)

    return bytes(packet_bytes)

def read_available_lines(serial_port):
    while serial_port.in_waiting > 0:
        response = serial_port.readline().decode(errors="ignore").strip()
        if response:
            print(response)


def main():
    controller = InputDevice(EVENT_PATH)

    if controller.name != REQUIRED_NAME:
        raise RuntimeError(
            f"{EVENT_PATH} name mismatch. Expected '{REQUIRED_NAME}', got '{controller.name}'"
        )

    serial_port = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.02)

    print(f"Using controller: {EVENT_PATH} name='{controller.name}'")
    print(f"Connected to ESP32 on {ESP_PORT} @baud {ESP_BAUD}")

    lt_value = 0
    rt_value = 0
    lsx_value = 0
    rsx_value = 0
    dpx_value = 0
    lb_pressed = 0
    rb_pressed = 0

    press_order = []
    next_send_time = time.monotonic()

    controller.grab()

    try:
        while True:
            current_time = time.monotonic()
            wait_timeout = next_send_time - current_time

            if wait_timeout < 0:
                wait_timeout = 0

            ready_to_read = select.select(
                [controller.fd],
                [],
                [],
                wait_timeout
            )[0]

            if ready_to_read:
                for event in controller.read():
                    if event.type == ecodes.EV_ABS:
                        if event.code == LT_CODE:
                            lt_value = event.value

                        elif event.code == RT_CODE:
                            rt_value = event.value

                        elif event.code == LSX_CODE:
                            lsx_value = event.value

                        elif event.code == RSX_CODE:
                            rsx_value = event.value

                        elif event.code == DPAD_X_CODE:
                            dpx_value = event.value

                    elif event.type == ecodes.EV_KEY:
                        if event.code == LB_CODE:
                            lb_pressed = 1 if event.value else 0

                        elif event.code == RB_CODE:
                            rb_pressed = 1 if event.value else 0

            current_time = time.monotonic()

            if current_time >= next_send_time:
                raw_commands = build_commands(
                    lt_value,
                    rt_value,
                    lsx_value,
                    rsx_value,
                    dpx_value,
                    lb_pressed,
                    rb_pressed
                )

                update_press_order(raw_commands, press_order)
                chosen_commands = choose_two_oldest(raw_commands, press_order)
                full_state = build_full_state(chosen_commands)
                packet = build_packet(full_state)

                serial_port.write(packet)
                serial_port.flush()
                read_available_lines(serial_port)

                while next_send_time <= current_time:
                    next_send_time += SEND_DT

    except KeyboardInterrupt:
        neutral_state = {
            "B": NEUTRAL,
            "S": NEUTRAL,
            "F": NEUTRAL,
            "W": NEUTRAL,
            "G": NEUTRAL
        }

        neutral_packet = build_packet(neutral_state)

        for _ in range(3):
            serial_port.write(neutral_packet)
            serial_port.flush()
            read_available_lines(serial_port)
            time.sleep(0.02)

    finally:
        try:
            controller.ungrab()
        except Exception:
            pass

        try:
            serial_port.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
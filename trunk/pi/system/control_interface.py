"""

This is the control interface for the Raspberry Pi 4 --> ESP32. 

Right now it handles everything from connecting controller to sending bytes over UART.

It should probably be split up into seperate modules that the system can take over

Preferably into:

Controller connection

Controller feed

UART Communication

Intestines to GUI



"""



import time
import serial

from evdev import InputDevice, ecodes


#these are hard-coded and need to change
EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"


#these are fine, maybe need to go in a class to support other
#controller types
LT_CODE = ecodes.ABS_Z
RT_CODE = ecodes.ABS_RZ
LSX_CODE = ecodes.ABS_X
RSX_CODE = ecodes.ABS_RX
DPAD_X_CODE = ecodes.ABS_HAT0X
LB_CODE = ecodes.BTN_TL
RB_CODE = ecodes.BTN_TR

#also needs dynamic configuration, maybe in the GUI?
ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200


#these are pretty solid, we want to update the commands every 
#10ms. 
SEND_HZ = 100
SEND_DT = 1.0 / SEND_HZ

#UART start byte and servo order need to be the same, every time.
#the esp should send alert if byte is corrupted
START_BYTE = 0xAA
PACKET_SIZE = 15
CRC8_POLYNOMIAL = 0x07

REQUEST_VERSION = 1
REQUEST_TYPE_SERVO = 1

SERVO_COMMAND_MOVE = 0x01

SERVO_ORDER = ("B", "S", "F", "W", "G")
NEUTRAL = None

#these are calibrated, maybe we make these configurable?
TRIGGER_THRESHOLD = 80
STICK_DEADBAND = 12000

#these should match the safe PWM limits in the ESP32 firmware
SERVO_US_MIN_SAFE = 500
SERVO_US_MAX_SAFE = 2500
SERVO_US_CENTER = 1500

STICK_RAW_MIN = -32768
STICK_RAW_MAX = 32767

#Xbox controller triggers are often 0-1023 on Linux evdev.
#if your evtest output shows 0-255 instead, change this to 255.
TRIGGER_RAW_MIN = 0
TRIGGER_RAW_MAX = 1023


def clamp(v, lo, hi):
    #some controller buttons are absolute, and require 0/1 control.
    if v < lo:
        return lo

    if v > hi:
        return hi

    return v


def map_range(value, input_min, input_max, output_min, output_max):
    value = clamp(value, input_min, input_max)

    input_span = input_max - input_min
    output_span = output_max - output_min

    scaled_value = (value - input_min) / input_span

    return int(output_min + (scaled_value * output_span))


def map_signed_value_to_pwm(value):
    value = clamp(value, -1.0, 1.0)

    if value < 0:
        return int(SERVO_US_CENTER + (value * (SERVO_US_CENTER - SERVO_US_MIN_SAFE)))

    return int(SERVO_US_CENTER + (value * (SERVO_US_MAX_SAFE - SERVO_US_CENTER)))


def stick_to_pwm(raw_value, center=0, deadband=STICK_DEADBAND):
    #As of now, the sticks control the Wrist and Base Servos.
    #These probably shouldn't be 0/1 control
    distance_from_center = raw_value - center

    if abs(distance_from_center) < deadband:
        return NEUTRAL

    return map_range(
        raw_value,
        STICK_RAW_MIN,
        STICK_RAW_MAX,
        SERVO_US_MIN_SAFE,
        SERVO_US_MAX_SAFE
    )


def dpad_to_pwm(raw_value):
    limited_value = clamp(int(raw_value), -1, 1)

    if limited_value < 0:
        return SERVO_US_MIN_SAFE

    if limited_value > 0:
        return SERVO_US_MAX_SAFE

    return NEUTRAL


def trigger_pair_to_pwm(lt_value, rt_value, threshold=TRIGGER_THRESHOLD):
    #triggers probably shouldn't be 0/1 either
    left_trigger_active = lt_value > threshold
    right_trigger_active = rt_value > threshold

    if not left_trigger_active and not right_trigger_active:
        return NEUTRAL

    left_value = clamp(lt_value, TRIGGER_RAW_MIN, TRIGGER_RAW_MAX)
    right_value = clamp(rt_value, TRIGGER_RAW_MIN, TRIGGER_RAW_MAX)

    left_normalized = left_value / TRIGGER_RAW_MAX
    right_normalized = right_value / TRIGGER_RAW_MAX

    combined_value = right_normalized - left_normalized

    return map_signed_value_to_pwm(combined_value)


def button_pair_to_pwm(negative_pressed, positive_pressed):
    if negative_pressed and not positive_pressed:
        return SERVO_US_MIN_SAFE

    if positive_pressed and not negative_pressed:
        return SERVO_US_MAX_SAFE

    return NEUTRAL


def calculate_crc8(data):
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


def build_commands(lt_value, rt_value, lsx_value, rsx_value, dpx_value, lb_pressed, rb_pressed):
    raw_commands = {
        "B": stick_to_pwm(lsx_value),
        "S": stick_to_pwm(rsx_value),
        "F": button_pair_to_pwm(lb_pressed, rb_pressed),
        "W": dpad_to_pwm(dpx_value),
        "G": trigger_pair_to_pwm(lt_value, rt_value)
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


def update_requested_pwm_values(full_state, requested_pwm_values):
    for servo_name in SERVO_ORDER:
        requested_pulse = full_state[servo_name]

        if requested_pulse is NEUTRAL:
            continue

        requested_pwm_values[servo_name] = clamp(
            requested_pulse,
            SERVO_US_MIN_SAFE,
            SERVO_US_MAX_SAFE
        )


def build_packet(requested_pwm_values, message_id):
    packet_bytes = bytearray()
    packet_bytes.append(START_BYTE)
    packet_bytes.append(REQUEST_VERSION)
    packet_bytes.append(message_id & 0xFF)
    packet_bytes.append(REQUEST_TYPE_SERVO)

    for servo_name in SERVO_ORDER:
        command_type = SERVO_COMMAND_MOVE

        requested_pulse = clamp(
            requested_pwm_values[servo_name],
            SERVO_US_MIN_SAFE,
            SERVO_US_MAX_SAFE
        )

        packed_high = (
            ((command_type & 0x0F) << 4) |
            ((requested_pulse >> 8) & 0x0F)
        )

        packed_low = requested_pulse & 0xFF

        packet_bytes.append(packed_high)
        packet_bytes.append(packed_low)

    crc = calculate_crc8(packet_bytes)
    packet_bytes.append(crc)

    if len(packet_bytes) != PACKET_SIZE:
        raise RuntimeError(f"Invalid packet size: {len(packet_bytes)}")

    return bytes(packet_bytes)


def read_available_lines(serial_port):
    while serial_port.in_waiting > 0:
        response = serial_port.readline().decode(errors="ignore").strip()
        if response:
            print(response)


def send_hold_packets(serial_port, requested_pwm_values, message_id):
    for _ in range(3):
        hold_packet = build_packet(requested_pwm_values, message_id)

        serial_port.write(hold_packet)
        serial_port.flush()
        read_available_lines(serial_port)

        message_id = (message_id + 1) & 0xFF
        time.sleep(0.02)

    return message_id


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
    message_id = 0

    requested_pwm_values = {
        "B": SERVO_US_CENTER,
        "S": SERVO_US_CENTER,
        "F": SERVO_US_CENTER,
        "W": SERVO_US_CENTER,
        "G": SERVO_US_CENTER
    }

    #add guards around this to prevent crashing if no controller
    controller.grab()
    try:
        while True:
            current_time = time.monotonic()
            wait_timeout = next_send_time - current_time

            if wait_timeout < 0:
                wait_timeout = 0


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

                update_requested_pwm_values(full_state, requested_pwm_values)

                packet = build_packet(requested_pwm_values, message_id)
                message_id = (message_id + 1) & 0xFF

                print(  "\nlt_value: ", lt_value,
                        "\nrt_value: ", rt_value,
                        "\nlsx_value: ", lsx_value,
                        "\nrsx_value: ", rsx_value,
                        "\ndpx_value: ", dpx_value,
                        "\nlb_pressed: ", lb_pressed,
                        "\nrb_pressed: ", rb_pressed,
                        "\n"
                        )
                

                serial_port.write(packet)
                serial_port.flush()
                read_available_lines(serial_port)

                while next_send_time <= current_time:
                    next_send_time += SEND_DT

    except KeyboardInterrupt:   #this doesn't work on shutdown...
        message_id = send_hold_packets(
            serial_port,
            requested_pwm_values,
            message_id
        )

        try:
            serial_port.close()
        except Exception as e:
            print("\n\nError: ", e,"While closing serial port on shutdown!\n\n")

    finally:
        try:
            controller.ungrab() 
        except Exception as e:
            print("Error ungrabbing controller on shutdown! \n", e)


if __name__ == "__main__":
    main()
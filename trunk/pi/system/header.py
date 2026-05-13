import os

#uart related constants

ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200

START_BYTE = 0xAA
CRC8_POLYNOMIAL = 0x07

UPDATE_STATUS_PACKET_TYPE = "S"

FEEDBACK_PACKET_TYPE = 'F'
FEEDBACK_PACKET_SIZE = 15
FEEDBACK_CRC_INDEX = 14

FEEDBACK_SERVO_STATE_START_INDEX = 3
FEEDBACK_ADC_VALID_FLAGS_INDEX = 8
FEEDBACK_ADC_LEVEL_START_INDEX = 9

#controller related constants
TRIGGER_DEADBAND = 80
STICK_DEADBAND = 12000
EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"

#path constants
TRUNK_PATH = os.path.expanduser("~/Projects/ARM/trunk")
FLASH_SCRIPT_PATH = os.path.join(TRUNK_PATH, "pi", "system", "esp_flash.py")

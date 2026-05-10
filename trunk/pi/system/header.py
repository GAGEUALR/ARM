import os

#uart related constants

ESP_PORT = "/dev/ttyUSB0"
ESP_BAUD = 115200

START_BYTE = 0xAA
CRC8_POLYNOMIAL = 0x07

UPDATE_STATUS_PACKET_TYPE = "S"

#controller related constants
TRIGGER_DEADBAND = 80
STICK_DEADBAND = 12000
EVENT_PATH = "/dev/input/event11"
REQUIRED_NAME = "Xbox Wireless Controller"

#path constants
TRUNK_PATH = os.path.expanduser("~/Projects/ARM/trunk")
FLASH_SCRIPT_PATH = os.path.join(TRUNK_PATH, "pi", "system", "esp_flash.py")

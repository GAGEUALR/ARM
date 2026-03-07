import os
import subprocess

TRUNK_PATH = os.path.expanduser("~/Projects/ARM/trunk")
FLASH_SCRIPT = os.path.join(TRUNK_PATH, "pi/system/esp_flash.py")
CONTROL_SCRIPT = os.path.join(TRUNK_PATH, "pi/system/control_interface.py")


def startup():

    # check + flash firmware if needed
    subprocess.run(
        ["python3", FLASH_SCRIPT, "--flash"],
        check=True
    )


if __name__ == "__main__":

    startup()

    # start control system
    os.execvp("python3", ["python3", CONTROL_SCRIPT])
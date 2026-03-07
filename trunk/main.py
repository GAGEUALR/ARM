import os
import subprocess
import sys

TRUNK_PATH = os.path.expanduser("~/Projects/ARM/trunk")
FLASH_SCRIPT = os.path.join(TRUNK_PATH, "pi", "system", "esp_flash.py")
CONTROL_SCRIPT = os.path.join(TRUNK_PATH, "pi", "system", "control_interface.py")


def startup():
    result = subprocess.run(
        ["python3", FLASH_SCRIPT, "--check"]
    )

    if result.returncode != 0:
        print(f"esp_flash.py failed with exit code {result.returncode}")
        sys.exit(result.returncode)


if __name__ == "__main__":
    startup()
    os.execvp("python3", ["python3", CONTROL_SCRIPT])
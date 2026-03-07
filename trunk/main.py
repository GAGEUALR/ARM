import os
import subprocess
import sys

TRUNK_DIR = os.path.expanduser("~/Projects/ARM/trunk")
ESP_PROJECT_DIR = os.path.join(TRUNK_DIR, "esp", "servo_controller")
PI_SCRIPT = os.path.join(TRUNK_DIR, "pi", "system", "control_interface.py")
PORT = "/dev/ttyUSB0"

EXPORT_SCRIPT = os.path.expanduser("~/esp/esp-idf/export.sh")

def run(cmd, cwd):
    process = subprocess.Popen(
        cmd,
        cwd=cwd,
        shell=True,
        executable="/bin/bash"
    )
    process.wait()

    if process.returncode != 0:
        sys.exit(process.returncode)

def main():

    if not os.path.exists(EXPORT_SCRIPT):
        print("ESP-IDF export script not found:", EXPORT_SCRIPT)
        sys.exit(1)

    build_cmd = f". {EXPORT_SCRIPT} && idf.py -p {PORT} build flash"
    run(build_cmd, ESP_PROJECT_DIR)

    os.execvp("python3", ["python3", PI_SCRIPT])

if __name__ == "__main__":
    main()
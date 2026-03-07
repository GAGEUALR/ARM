import os
import subprocess
import sys

TRUNK_DIR = os.path.expanduser("~/Projects/ARM/trunk")
ESP_PROJECT_DIR = os.path.join(TRUNK_DIR, "esp", "servo_controller")
ESP_SOURCE_FILE = os.path.join(ESP_PROJECT_DIR, "main", "main.c")
PI_SCRIPT = os.path.join(TRUNK_DIR, "pi", "system", "control_interface.py")
PORT = "/dev/ttyUSB0"

def run_command(cmd, cwd):
    process = subprocess.Popen(
        cmd,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    for line in process.stdout:
        print(line, end="")

    process.wait()

    if process.returncode != 0:
        raise SystemExit(process.returncode)

def main():
    if not os.path.isdir(ESP_PROJECT_DIR):
        print(f"ESP-IDF project directory not found:\n{ESP_PROJECT_DIR}")
        sys.exit(1)

    if not os.path.isfile(ESP_SOURCE_FILE):
        print(f"ESP source file not found:\n{ESP_SOURCE_FILE}")
        sys.exit(1)

    if not os.path.isfile(PI_SCRIPT):
        print(f"Pi control script not found:\n{PI_SCRIPT}")
        sys.exit(1)

    print(f"Using ESP source:\n{ESP_SOURCE_FILE}\n")
    print(f"Using Pi script:\n{PI_SCRIPT}\n")
    print(f"Flashing on port: {PORT}\n")

    run_command(["idf.py", "-p", PORT, "build"], ESP_PROJECT_DIR)
    run_command(["idf.py", "-p", PORT, "flash"], ESP_PROJECT_DIR)

    print("\nESP flashed successfully.\n")
    print("Starting control_interface.py...\n")

    os.execvp("python3", ["python3", PI_SCRIPT])

if __name__ == "__main__":
    main()
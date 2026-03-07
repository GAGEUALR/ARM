import hashlib
import os
import subprocess
import sys

TRUNK_DIR = os.path.expanduser("~/Projects/ARM/trunk")
ESP_PROJECT_DIR = os.path.join(TRUNK_DIR, "esp", "servo_controller")
ESP_SOURCE_FILE = os.path.join(ESP_PROJECT_DIR, "main", "main.c")
EXPORT_SCRIPT = os.path.expanduser("~/esp/esp-idf/export.sh")
PORT = "/dev/ttyUSB0"

STATE_DIR = os.path.join(TRUNK_DIR, ".state")
HASH_FILE = os.path.join(STATE_DIR, "esp_main_c.sha256")


def file_sha256(path):
    h = hashlib.sha256()

    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)

    return h.hexdigest()


def read_saved_hash():
    if not os.path.exists(HASH_FILE):
        return None

    with open(HASH_FILE, "r", encoding="utf-8") as f:
        return f.read().strip()


def write_saved_hash(value):
    os.makedirs(STATE_DIR, exist_ok=True)

    with open(HASH_FILE, "w", encoding="utf-8") as f:
        f.write(value)


def verify_paths():
    if not os.path.isdir(ESP_PROJECT_DIR):
        print("ESP-IDF project directory not found:")
        print(ESP_PROJECT_DIR)
        sys.exit(1)

    if not os.path.isfile(ESP_SOURCE_FILE):
        print("ESP source file not found:")
        print(ESP_SOURCE_FILE)
        sys.exit(1)

    if not os.path.isfile(EXPORT_SCRIPT):
        print("ESP-IDF export script not found:")
        print(EXPORT_SCRIPT)
        sys.exit(1)


def run_bash(command, cwd):
    process = subprocess.Popen(
        command,
        cwd=cwd,
        shell=True,
        executable="/bin/bash"
    )
    process.wait()

    if process.returncode != 0:
        raise SystemExit(process.returncode)


def build_and_flash():
    cmd = f'. "{EXPORT_SCRIPT}" && idf.py -p {PORT} build flash'
    run_bash(cmd, ESP_PROJECT_DIR)


def print_usage():
    print("Usage:")
    print("  python3 esp_flash.py --check   -> build + flash only if main.c changed")
    print("  python3 esp_flash.py --build   -> always build + flash")


def main():
    verify_paths()

    if len(sys.argv) != 2 or sys.argv[1] not in ("--check", "--build"):
        print_usage()
        sys.exit(1)

    mode = sys.argv[1]
    current_hash = file_sha256(ESP_SOURCE_FILE)
    saved_hash = read_saved_hash()

    if mode == "--build":
        print("Forcing build + flash...")
        build_and_flash()
        write_saved_hash(current_hash)
        return

    if current_hash == saved_hash:
        print("main.c unchanged. Skipping ESP build/flash.")
        return

    print("main.c changed since last successful flash. Running build + flash...")
    build_and_flash()
    write_saved_hash(current_hash)


if __name__ == "__main__":
    main()
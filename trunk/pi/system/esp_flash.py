import hashlib
import json
import os
import subprocess
import sys

TRUNK_DIR = os.path.expanduser("~/Projects/ARM/trunk")
ESP_PROJECT_DIR = os.path.join(TRUNK_DIR, "esp", "servo_controller")
EXPORT_SCRIPT = os.path.expanduser("~/esp/esp-idf/export.sh")
PORT = "/dev/ttyUSB0"

STATE_DIR = os.path.join(TRUNK_DIR, ".state")
HASH_FILE = os.path.join(STATE_DIR, "esp_project_sources.sha256")

SOURCE_FILES = [
    os.path.join(ESP_PROJECT_DIR, "main", "main.c"),
    os.path.join(ESP_PROJECT_DIR, "main", "main.h"),
    os.path.join(ESP_PROJECT_DIR, "main", "control.c"),
    os.path.join(ESP_PROJECT_DIR, "main", "control.h"),
    os.path.join(ESP_PROJECT_DIR, "main", "uart.c"),
]


def file_sha256(path):
    h = hashlib.sha256()

    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)

    return h.hexdigest()



def compute_project_hash(paths):
    h = hashlib.sha256()

    for path in paths:
        normalized_path = os.path.relpath(path, ESP_PROJECT_DIR).replace("\\", "/")
        h.update(normalized_path.encode("utf-8"))
        h.update(b"\0")
        h.update(file_sha256(path).encode("utf-8"))
        h.update(b"\0")

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

    if not os.path.isfile(EXPORT_SCRIPT):
        print("ESP-IDF export script not found:")
        print(EXPORT_SCRIPT)
        sys.exit(1)

    missing_files = [path for path in SOURCE_FILES if not os.path.isfile(path)]
    if missing_files:
        print("ESP source file(s) not found:")
        for path in missing_files:
            print(path)
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
    print("  python3 esp_flash.py --check   -> build + flash only if tracked ESP source files changed")
    print("  python3 esp_flash.py --build   -> always build + flash")
    print("  python3 esp_flash.py --list    -> print tracked source files")



def print_tracked_files():
    tracked_files = [os.path.relpath(path, ESP_PROJECT_DIR) for path in SOURCE_FILES]
    print(json.dumps(tracked_files, indent=2))



def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("--check", "--build", "--list"):
        print_usage()
        sys.exit(1)

    mode = sys.argv[1]

    verify_paths()

    if mode == "--list":
        print_tracked_files()
        return

    current_hash = compute_project_hash(SOURCE_FILES)
    saved_hash = read_saved_hash()

    if mode == "--build":
        print("Forcing build + flash...")
        build_and_flash()
        write_saved_hash(current_hash)
        return

    if current_hash == saved_hash:
        print("Tracked ESP source files unchanged. Skipping ESP build/flash.")
        return

    print("Tracked ESP source files changed since last successful flash. Running build + flash...")
    build_and_flash()
    write_saved_hash(current_hash)


if __name__ == "__main__":
    main()
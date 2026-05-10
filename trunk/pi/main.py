import subprocess
import sys
import time
from system.header import FLASH_SCRIPT_PATH
from system.system import System

def startup():
    result = subprocess.run(
        ["python3", FLASH_SCRIPT_PATH, "--check"]
    )

    if result.returncode != 0:
        print(f"esp_flash.py failed with exit code {result.returncode}")
        sys.exit(result.returncode)

def main():

    system = System()
    try:
        while(1):

            spent_time = time.monotonic()

            #assign the controller state to self.servos
            system.update_state()
            #limit to two self.servos at a time to prevent large current draw
            system.limit_active_servos()
            #compile uart packet
            system.send_commands()
            #receive feedback from uart and print error if possible

            elapsed = time.monotonic() - spent_time

            time.sleep(max(0, 0.010 - elapsed))
            #log events

    except KeyboardInterrupt:
        print("\nStopping controller mode")

    finally:
        system.shutdown()

if __name__ == "__main__":
    startup()
    main()

from evdev import InputDevice, ecodes

class Controller:

    def __init__(self, event_path, required_name):

        self.lt_code = ecodes.ABS_Z
        self.rt_code = ecodes.ABS_RZ
        self.lsx_code = ecodes.ABS_X
        self.rsx_code = ecodes.ABS_RX
        self.dpad_x_code = ecodes.ABS_HAT0X
        self.lb_code = ecodes.BTN_TL
        self.rb_code = ecodes.BTN_TR

        self.required_name = required_name  #only supports xbox wireless controller
        self.event_path = event_path
        self.input_device = None
        self.connected = False
        self.state = {
            "lt": 0,
            "rt": 0,
            "lsx": 0,
            "rsx": 0,
            "dpad_x": 0,
            "lb": 0,
            "rb": 0,
        }

    def startup(self):
        self.input_device = InputDevice(self.event_path)

        if self.input_device.name != self.required_name:
            raise RuntimeError(
                f"{self.event_path} name mismatch. "
                f"Expected '{self.required_name}', got '{self.input_device.name}'"
            )

        self.input_device.grab()
        self.connected = True

        print(f"Using controller: {self.event_path} name='{self.input_device.name}'")

    def read_controller(self):
        if not self.connected or self.input_device is None:
            return self.state

        try:
            for event in self.input_device.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == self.lt_code:
                        self.state["lt"] = event.value

                    elif event.code == self.rt_code:
                        self.state["rt"] = event.value

                    elif event.code == self.lsx_code:
                        self.state["lsx"] = event.value

                    elif event.code == self.rsx_code:
                        self.state["rsx"] = event.value

                    elif event.code == self.dpad_x_code:
                        self.state["dpad_x"] = event.value

                elif event.type == ecodes.EV_KEY:
                    if event.code == self.lb_code:
                        self.state["lb"] = 1 if event.value else 0

                    elif event.code == self.rb_code:
                        self.state["rb"] = 1 if event.value else 0

        except BlockingIOError as e:
            pass

        return self.state

    def shutdown(self):
        if self.input_device is None:
            return

        try:
            self.input_device.ungrab()
        except Exception as e:
            print("Error ungrabbing Controller!: \n", e)

        self.connected = False
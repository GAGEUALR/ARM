from .servo import Servo
from .controller import Controller
from .uart_to_esp import UartCom
from .header import EVENT_PATH, REQUIRED_NAME, STICK_DEADBAND,\
                    TRIGGER_DEADBAND, ESP_PORT, ESP_BAUD

class System:

    def __init__(self):

        Base = Servo("B")
        Shoulder = Servo("S")
        Forearm = Servo("F")
        Wrist = Servo("W")
        Gripper = Servo("G")

        self.servos = [Base, Shoulder, Forearm, Wrist, Gripper]

        self.controller = Controller(EVENT_PATH, REQUIRED_NAME)
        self.controller.startup()

        self.uart_link = UartCom(ESP_PORT, ESP_BAUD)
        if not self.uart_link.start_comms():
            raise RuntimeError("UART startup failed")

        self.message_id = 0
        self.press_order = []

        
    def update_state(self):

        state = self.controller.read_controller()
        try: 

            if abs(state["lsx"]) < STICK_DEADBAND:
                self.servos[0].set_command(False, None)
            elif state["lsx"] < -STICK_DEADBAND:
                self.servos[0].set_command(True, False)
            elif state["lsx"] > STICK_DEADBAND:
                self.servos[0].set_command(True, True)

            if (state["lb"] == 0) and (state["rb"] == 0):
                self.servos[1].set_command(False, None)
            elif (state["lb"] == 1) and (state["rb"] == 0):
                self.servos[1].set_command(True, False)
            elif (state["lb"] == 0) and (state["rb"] == 1):
                self.servos[1].set_command(True, True)
            else:
                self.servos[1].set_command(False, None)

            if state["dpad_x"] == 0:
                self.servos[2].set_command(False, None)
            elif state["dpad_x"] < 0:
                self.servos[2].set_command(True, False)
            elif state["dpad_x"] > 0:
                self.servos[2].set_command(True, True)

            if abs(state["rsx"]) < STICK_DEADBAND:
                self.servos[3].set_command(False, None)
            elif state["rsx"] < -STICK_DEADBAND:
                self.servos[3].set_command(True, False)
            elif state["rsx"] > STICK_DEADBAND:
                self.servos[3].set_command(True, True)

            if (state["lt"] < TRIGGER_DEADBAND) and (state["rt"] < TRIGGER_DEADBAND):
                self.servos[4].set_command(False, None)
            elif (state["lt"] > TRIGGER_DEADBAND) and (state["lt"] > state["rt"]):
                self.servos[4].set_command(True, False)
            elif (state["rt"] > TRIGGER_DEADBAND) and (state["rt"] > state["lt"]):
                self.servos[4].set_command(True, True)
            else:
                self.servos[4].set_command(False, None)

        except Exception as e:
            print("Urecognizable controller reading\nError: ", e)
            pass

    def limit_active_servos(self):

        for servo in self.servos:
            if not servo.active and servo.id in self.press_order:
                self.press_order.remove(servo.id)

        for servo in self.servos:
            if servo.active and servo.id not in self.press_order:
                self.press_order.append(servo.id)

        allowed_ids = self.press_order[:2]

        for servo in self.servos:
            if servo.active and servo.id not in allowed_ids:
                servo.set_command(False, None)

    def send_commands(self):

        packet = self.uart_link.build_update_status_packet(self.servos, self.message_id)

        if packet is None:
            return

        if self.uart_link.send_packet(packet):
            self.message_id = (self.message_id + 1) & 0xFF

    def read_from_esp(self):

        feedback = self.uart_link.read_feedback_packet()

        if feedback is not None:
            print(f'\n{feedback}\n')

    def shutdown(self):
        self.controller.shutdown()
        self.uart_link.stop_comms()

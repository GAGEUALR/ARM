import serial
from .header import START_BYTE, CRC8_POLYNOMIAL, UPDATE_STATUS_PACKET_TYPE, \
                    FEEDBACK_PACKET_TYPE, FEEDBACK_PACKET_SIZE, FEEDBACK_CRC_INDEX, \
                    FEEDBACK_SERVO_STATE_START_INDEX, FEEDBACK_ADC_VALID_FLAGS_INDEX, \
                    FEEDBACK_ADC_LEVEL_START_INDEX
class UartCom:
      
      def __init__(self, port_name, baud):
            
            
            self.port_name = port_name
            self.baud = baud
            self.port = None
            self.connected = None
            self.receive_buffer = bytearray()

      def start_comms(self):

            try:
                  self.port = serial.Serial(self.port_name, self.baud, timeout=0.02)
            except Exception as e:
                  print("\nError initializing uart port! \n",e)
                  self.connected = False
                  return 0
      
            print(f"\nConnected to ESP32 on {self.port} @baud {self.baud}")
            self.connected = True
            return 1
      

      def build_update_status_packet(self, servos, message_id):
            packet_bytes = bytearray()

            try:
                  packet_bytes.append(START_BYTE)
                  packet_bytes.append(message_id & 0xFF)
                  packet_bytes.append(ord(UPDATE_STATUS_PACKET_TYPE))

                  for servo in servos:
                        if not servo.active:
                              packet_bytes.append(ord('I'))

                        elif servo.direction:
                              packet_bytes.append(ord('P'))

                        else:
                              packet_bytes.append(ord('N'))

                  crc = self.calculate_crc8(packet_bytes)
                  packet_bytes.append(crc)

            except Exception as e:
                  print("Error building status packet:", packet_bytes,
                        "\n(message id: ", message_id, ")\n", e)
                  return None

            return bytes(packet_bytes)

      def send_packet(self, packet):

            if self.port is None:
                  return 0
            try: 
                  self.port.write(packet)
            except Exception as e:
                  print("\nError: \n", e, "\nOccured while writing packet:\n", packet)
                  return 0
            
            return 1

      def read_feedback_packet(self):

            if self.port is None:
                  return None

            try:
                  waiting_bytes = self.port.in_waiting

                  if waiting_bytes > 0:
                        self.receive_buffer.extend(self.port.read(waiting_bytes))

                  latest_feedback = None

                  while len(self.receive_buffer) >= FEEDBACK_PACKET_SIZE:

                        if self.receive_buffer[0] != START_BYTE:
                              start_index = self.receive_buffer.find(bytes([START_BYTE]))

                              if start_index == -1:
                                    self.receive_buffer.clear()
                                    return latest_feedback

                              del self.receive_buffer[:start_index]

                              if len(self.receive_buffer) < FEEDBACK_PACKET_SIZE:
                                    return latest_feedback

                        packet = self.receive_buffer[:FEEDBACK_PACKET_SIZE]

                        if packet[2] != ord(FEEDBACK_PACKET_TYPE):
                              del self.receive_buffer[0]
                              continue

                        crc = self.calculate_crc8(packet[:FEEDBACK_CRC_INDEX])

                        if crc != packet[FEEDBACK_CRC_INDEX]:
                              del self.receive_buffer[0]
                              continue

                        servo_states = []

                        for state in packet[
                              FEEDBACK_SERVO_STATE_START_INDEX:
                              FEEDBACK_SERVO_STATE_START_INDEX + 5
                        ]:
                              servo_states.append(chr(state))

                        adc_levels = list(packet[
                              FEEDBACK_ADC_LEVEL_START_INDEX:
                              FEEDBACK_ADC_LEVEL_START_INDEX + 5
                        ])

                        latest_feedback = {
                              "message_id": packet[1],
                              "servo_states": servo_states,
                              "adc_valid_flags": packet[FEEDBACK_ADC_VALID_FLAGS_INDEX],
                              "adc_levels": adc_levels
                        }

                        del self.receive_buffer[:FEEDBACK_PACKET_SIZE]

                  return latest_feedback

            except Exception as e:
                  print("\nError reading feedback packet:\n", e)
                  return None

      def calculate_crc8(self, data):
            crc = 0x00

            for byte in data:
                  crc ^= byte

                  for _ in range(8):
                        if crc & 0x80:
                              crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF
                        else:
                              crc = (crc << 1) & 0xFF

            return crc

      def stop_comms(self):
            try:
                  if self.port is not None:
                        self.port.close()
            except Exception as e:
                  print("\nError closing port!\n", e)
                  return 0

            return 1
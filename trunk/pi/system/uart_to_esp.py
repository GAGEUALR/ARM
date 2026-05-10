import serial
from header import START_BYTE, CRC8_POLYNOMIAL, UPDATE_STATUS_PACKET_TYPE

class UartCom:
      
      def __init__(self, port_name, baud):
            
            
            self.port_name = port_name
            self.baud = baud
            self.port = None
            self.connected = None

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
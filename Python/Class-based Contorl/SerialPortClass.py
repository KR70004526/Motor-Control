import serial
import threading
import time

class SerialPort:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.lock = threading.Lock()
        try:
            self.serial = serial.Serial(port=port,
                                        baudrate=baudrate,
                                        timeout=timeout)
            # Arduino Reset & Stabilization
            time.sleep(2)
            # Empty the input buffer
            self.serial.reset_input_buffer()
        except serial.SerialException as e:
            print(f"<Serial Port Error: {e}>")
            self.serial = None

    def send_command(self, command: str):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")
        with self.lock:
            self.serial.write((command + '\n').encode('utf-8'))
            self.serial.flush()

    def read_response(self) -> str:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")
        raw = self.serial.readline().decode('utf-8', errors='ignore')
        return raw.strip()

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("<Serial Port Closed>")
        else:
            print("<Serial Port is Already Closed>")

# Example for Serial
if __name__ == "__main__":
    # Example usage
    serial_port = SerialPort(port='COM4', baudrate=115200, timeout=1)
    serial_port.send_command("ON")
    response = serial_port.read_response()
    print(f"Response: {response}")
    serial_port.close()

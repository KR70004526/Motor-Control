import serial
import serial.tools.list_ports

class SerialPortManager:
    def __init__(self):
        self.ser = None
    
    # List available serial ports
    def list_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]
    
    # Connect to the specified serial port
    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            if self.ser.is_open:
                return True, "Connected to port successfully."
        except serial.SerialException as e:
            print(f"Error connecting to port {port}: {e}")
        return False, "Failed to connect to port."
    
    # Send motor ID to the motor
    def send_motor_id(self, motor_id: str):
        if not self.ser or not self.ser.is_open:
            return False, "Please Connect Serial Port First."
        if not motor_id.isdigit():
            return False, "Motor ID must be a number."
        motor_id = int(motor_id)
        motor_id = max(0, min(motor_id, 127))
        try:
            self.ser.write(f"ID:{motor_id}\n".encode())
            return True, f"Successfully Send Motor ID {motor_id}!!"
        except Exception as e:
            return False, str(e)

    # Send command to the motor    
    def send_command(self, command: str):
        if not self.ser or not self.ser.is_open:
            return False, "Please Connect Serial Port First."
        try:
            self.ser.write(f"{command}\n".encode())
            return True, f"Command '{command}' sent successfully."
        except Exception as e:
            return False, f"Failed to send command: {e}"

    # Send ON command to the motor    
    def send_on(self):
        return self.send_command("ON")
    
    # Send OFF command to the motor
    def send_off(self):
        return self.send_command("OFF")

    # Send ORIGIN command to the motor
    def send_origin(self):
        return self.send_command("ORIGIN")

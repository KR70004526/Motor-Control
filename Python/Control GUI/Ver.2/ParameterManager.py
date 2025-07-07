class ParameterManager:
    def __init__(self, serial_manager):
        self.serial_manager = serial_manager

    # Send parameters to the motor
    def send_parameters(self, position, velocity, kp, kd, torque):
        if not self.serial_manager.ser or not self.serial_manager.ser.is_open:
            return False, "Please Connect Serial Port First."
        
        try:
            pos = float(position)
            vel = float(velocity)
            kp = float(kp)
            kd = float(kd)
            tor = float(torque)
        except ValueError:
            return False, "Invalid input. Please enter numeric values."
        
        command = f"SEND POS:{pos},VEL:{vel},KP:{kp},KD:{kd},TOR:{tor}"
        try:
            self.serial_manager.ser.write(f"{command}\n".encode())
            return True, "Parameters sent successfully."
        except Exception as e:
            return False, f"Failed to send parameters: {e}"
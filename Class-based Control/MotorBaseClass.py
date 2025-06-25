from SerialPortClass import SerialPort
from typing import Optional

class MotorBase:
    _sp: Optional[SerialPort] = None

    @classmethod
    def init_serial(cls, port, baudrate=115200, timeout=1):
        if cls._sp is None:
            cls._sp = SerialPort(port, baudrate, timeout)

    @classmethod
    def close_serial(cls):
        if cls._sp:
            cls._sp.close()
            cls._sp = None

    def __init__(self, motor_id):
        if MotorBase._sp is None:
            raise RuntimeError("Serial port not initialized. Call init_serial() first.")
        self.motor_id = motor_id
        self.pos_des = 0.0
        self.vel_des = 0.0
        self.tor_des = 0.0
        self.kp      = 0.0
        self.kd      = 0.0
        self.pos_out = 0.0
        self.vel_out = 0.0
        self.tor_out = 0.0

    def send_id(self):
        MotorBase._sp.send_command(f"ID:{self.motor_id}")

    def on(self):
        MotorBase._sp.send_command("ON")

    def off(self):
        MotorBase._sp.send_command("OFF")

    def read(self, prefix: str = None) -> str:
        while True:
            line = MotorBase._sp.read_response()
            if not line:
                continue
            if prefix is None or line.startswith(prefix):
                return line
            
    def update_parameters(self, pos=None, vel=None, kp=None, kd=None, tor=None):
        if pos is not None:
            self.pos_des = pos
        if vel is not None:
            self.vel_des = vel
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if tor is not None:
            self.tor_des = tor
        MotorBase._sp.send_command(f"SEND POS:{self.pos_des},VEL:{self.vel_des},KP:{self.kp},KD:{self.kd},TOR:{self.tor_des}")

    def get_parameters(self) -> tuple[float, float, float]:
        line = self.read(prefix="Position:")

        parts = line.split(',')
        if len(parts) != 3:
            raise ValueError(f"Unexpected format: {line!r}")

        self.pos_out = float(parts[0].split(':', 1)[1].strip())
        self.vel_out = float(parts[1].split(':', 1)[1].strip())
        self.tor_out = float(parts[2].split(':', 1)[1].strip())

        return self.pos_out, self.vel_out, self.tor_out

    
if __name__ == "__main__":
    import time
    # Example usage
    MotorBase.init_serial(port='COM4', baudrate=115200, timeout=1)
    motor = MotorBase(motor_id=2)
    motor.send_id()
    # print(motor.read(prefix="MOTOR ID"))
    motor.on()
    print(motor.read(prefix="Motor Mode"))

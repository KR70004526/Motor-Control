# MotorBaseClass.py

from SerialPortClass import SerialPort
from typing import Optional
import time

class MotorBase:
    # ─── 클래스 레벨에 단 하나만 열려 있는 SerialPort/IPCSerial ───
    _sp: Optional[SerialPort] = None

    @classmethod
    def init_serial(cls, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """최초 1회만 COM 포트를 열어 SerialPort 인스턴스를 생성합니다."""
        if cls._sp is None:
            cls._sp = SerialPort(port, baudrate, timeout)

    @classmethod
    def close_serial(cls):
        """프로그램 종료 시 포트를 닫습니다."""
        if cls._sp:
            cls._sp.close()
            cls._sp = None

    def __init__(self, motor_id: int):
        if MotorBase._sp is None:
            raise RuntimeError("Serial port not initialized. Call init_serial() first.")
        # 클래스 레벨 _sp 를 인스턴스 속성으로 복사해 두면 self._sp 로 편하게 사용 가능
        self._sp = MotorBase._sp

        self.motor_id = motor_id
        self.pos_des = self.vel_des = self.tor_des = 0.0
        self.kp = self.kd = 0.0
        self.pos_out = self.vel_out = self.tor_out = 0.0

    def send_id(self):
        self._sp.send_command(f"ID:{self.motor_id}")

    def on(self):
        self._sp.send_command("ON")

    def off(self):
        self._sp.send_command("OFF")

    def read(self, prefix: str = None) -> str:
        while True:
            line = MotorBase._sp.read_response()
            if not line:
                continue
            if prefix is None or line.startswith(prefix):
                return line

    def update_parameters(self, pos=None, vel=None, kp=None, kd=None, tor=None):
        if pos  is not None: self.pos_des = pos
        if vel  is not None: self.vel_des = vel
        if kp   is not None: self.kp      = kp
        if kd   is not None: self.kd      = kd
        if tor  is not None: self.tor_des = tor
        cmd = f"SEND POS:{self.pos_des},VEL:{self.vel_des},KP:{self.kp},KD:{self.kd},TOR:{self.tor_des}"
        self._sp.send_command(cmd)

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
    # 예제 실행 흐름
    MotorBase.init_serial('COM4', 115200, timeout=1.0)
    motor = MotorBase(motor_id=1)
    motor.send_id()
    motor.on()
    print(motor.read(prefix="Motor Mode"))
    # 파라미터 업데이트 및 읽기 테스트
    motor.update_parameters(pos=10.0, vel=5.0, kp=1.0, kd=0.1, tor=0.5)
    print(motor.get_parameters())
    motor.off()
    MotorBase.close_serial()

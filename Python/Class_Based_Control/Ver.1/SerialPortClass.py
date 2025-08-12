# SerialPortClass.py

import serial
from typing import Optional
import time

DEFAULT_TIMEOUT = 1.0  # 기존 기본 타임아웃

class SerialPort:
    def __init__(self, port: str, baudrate: int, timeout: float = DEFAULT_TIMEOUT):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def send_command(self, cmd: str):
        self.serial.write((cmd + '\n').encode('utf-8'))

    def read_response(self) -> Optional[str]:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")
        raw = self.serial.readline().decode('utf-8', errors='ignore')
        return raw.strip()

    def clear_buffer(self):
        """
        pyserial 내부 버퍼에 남은 모든 데이터를 비웁니다.
        """
        try:
            # 가장 빠른 일괄 삭제
            self.serial.reset_input_buffer()
        except AttributeError:
            # fallback: 한 줄씩
            while self.serial.in_waiting:
                self.serial.readline()

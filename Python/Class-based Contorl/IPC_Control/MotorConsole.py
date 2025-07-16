# motor_console.py
import sys
import socket
import json
from MotorBaseClass import MotorBase
from LogControlManager import LogControlManager

class IPCSerial:
    """SerialPortClass 인터페이스(send_command/read_response)를 IPC로 대체"""
    def __init__(self, host='localhost', port=9999, timeout=1):
        self.sock = socket.create_connection((host, port), timeout)
        self.file = self.sock.makefile(mode='rwb')

    def send_command(self, cmd: str):
        msg = {'type': 'command', 'payload': cmd}
        self.file.write((json.dumps(msg) + '\n').encode('utf-8'))
        self.file.flush()
        # 서버 응답 수신(성공 여부만 확인)
        line = self.file.readline()
        resp = json.loads(line.decode('utf-8'))
        if resp.get('status') != 'ok':
            raise RuntimeError(resp.get('error', 'IPC 에러'))

    def read_response(self) -> str:
        msg = {'type': 'read'}
        self.file.write((json.dumps(msg) + '\n').encode('utf-8'))
        self.file.flush()
        line = self.file.readline()
        resp = json.loads(line.decode('utf-8'))
        if 'response' in resp:
            return resp['response']
        raise RuntimeError(resp.get('error', 'IPC 에러'))

    def close(self):
        self.sock.close()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python motor_console.py <motor_id>')
        sys.exit(1)

    motor_id = int(sys.argv[1])

    # 기존 MotorBase._sp를 IPCSerial 인스턴스로 교체
    MotorBase._sp = IPCSerial()

    # 나머지 로직은 LogControlManager 그대로 사용
    manager = LogControlManager(port=None, baudrate=None, motor_id=motor_id)
    manager.send_id()
    manager.command_loop()

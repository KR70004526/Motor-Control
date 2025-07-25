# # MotorConsole.py

# import socket
# import json
# import time
# from typing import Optional

# class IPCSerial:
#     def __init__(self, host: str, port: int):
#         self.sock = socket.create_connection((host, port))
#         self.file = self.sock.makefile(mode='rwb')

#     def send_command(self, cmd: str):
#         msg = json.dumps({'type': 'command', 'payload': cmd}) + '\n'
#         self.file.write(msg.encode('utf-8'))
#         self.file.flush()
#         # 서버 응답은 무시하거나 확인 로직을 넣어도 됩니다

#     def read_response(self) -> Optional[str]:
#         msg = {'type': 'read'}
#         self.file.write((json.dumps(msg) + '\n').encode('utf-8'))
#         self.file.flush()
#         line = self.file.readline()
#         resp = json.loads(line.decode('utf-8'))
#         if 'response' in resp:
#             return resp['response']
#         raise RuntimeError(resp.get('error', 'IPC 에러'))

#     def clear_buffer(self):
#         """
#         IPC 서버로부터 남아 있는 모든 응답을 소진합니다.
#         """
#         while True:
#             try:
#                 # 즉시 반환 모드로 반복 읽기
#                 self.read_response(timeout=0.0)
#             except TimeoutError:
#                 break

# if __name__ == '__main__':
#     import sys
#     from MotorBaseClass import MotorBase
#     from LogControlManager import LogControlManager

#     if len(sys.argv) != 2:
#         print("Usage: python MotorConsole.py [motor_id]")
#         sys.exit(1)

#     motor_id = int(sys.argv[1])

#     # IPCSerial 인스턴스를 MotorBase에 주입
#     ipc = IPCSerial('localhost', 9999)
#     MotorBase._sp = ipc

#     # 모터 제어 시작
#     manager = LogControlManager(port=None, baudrate=None, motor_id=motor_id)
#     manager.send_id()
#     manager.command_loop()

# MotorConsole.py

import socket
import json
import sys
import threading
import time

from MotorBaseClass import MotorBase
from LogControlManager import LogControlManager

class IPCSerial:
    """
    IPC 서버 ↔ 클라이언트 통신 래퍼
    - command: 서버에 명령 전송 (type='command')
    - read:    모터 상태 요청  (type='read')
    """

    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port))
        self.file = self.sock.makefile(mode='rwb')

    def send_command(self, cmd: str):
        """ON/OFF, SEND POS:..., etc."""
        msg = json.dumps({'type': 'command', 'payload': cmd}) + '\n'
        self.file.write(msg.encode('utf-8'))
        self.file.flush()

    def read_response(self, timeout: float = None) -> str:
        """
        1) 모터 상태 요청 메시지 전송(type='read')
        2) 서버로부터 JSON 응답을 한 줄(readline) 받아 파싱
        3) {'status':'ok'} ACK는 무시하고, {'response':...}만 리턴
        """
        # ① read 요청
        req = json.dumps({'type': 'read'}) + '\n'
        self.file.write(req.encode('utf-8'))
        self.file.flush()

        # ② timeout 설정 (필요시)
        if timeout is not None:
            self.sock.settimeout(timeout)

        # ③ 응답 수신
        line = self.file.readline()
        if not line:
            raise RuntimeError("IPC 응답 없음")

        resp = json.loads(line.decode('utf-8'))
        # ④ ACK 무시
        if resp.get('status') == 'ok':
            return self.read_response(timeout)
        # ⑤ 실제 모터 데이터 리턴
        if 'response' in resp:
            return resp['response']
        raise RuntimeError(resp.get('error', 'IPC 에러'))

    def clear_buffer(self):
        """
        start() 직후나 send_id() 뒤에 남은 ACK/잔여 메시지 전부 비우기
        """
        self.sock.settimeout(0.0)
        try:
            while True:
                line = self.file.readline()
                if not line:
                    break
        except Exception:
            pass
        finally:
            self.sock.settimeout(None)


# if __name__ == "__main__":
#     # ─── 인자 체크 ──────────────────────────
#     if len(sys.argv) != 2:
#         print("Usage: python MotorConsole.py [motor_id]")
#         sys.exit(1)

#     motor_id = int(sys.argv[1])

#     # ─── IPCSerial 생성 & 버퍼 클리어 ─────────
#     ipc = IPCSerial('localhost', 9999)
#     ipc.clear_buffer()

#     # ─── MotorBase에 IPC 주입 ─────────────────
#     MotorBase._sp = ipc

#     # ─── LogControlManager 실행 ───────────────
#     manager = LogControlManager(port=None,
#                                 baudrate=None,
#                                 motor_id=motor_id)
#     manager.send_id()
#     # 수동으로 start 명령을 내리면 폴링 시작
#     manager.command_loop()

def run_console(motor_id: int):
    # 1) IPCSerial 생성 & 버퍼 클리어
    ipc = IPCSerial('localhost', 9999)
    ipc.clear_buffer()

    # 2) MotorBase에 IPC 주입
    MotorBase._sp = ipc

    # 3) LogControlManager 실행 → 명령 루프 진입
    manager = LogControlManager(port=None,
                                baudrate=None,
                                motor_id=motor_id)
    manager.send_id()
    manager.command_loop()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: <exe> console <motor_id>")
        sys.exit(1)
    run_console(int(sys.argv[1]))
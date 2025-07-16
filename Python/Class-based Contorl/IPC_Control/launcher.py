# launcher.py
import os
import sys
import threading
import subprocess
from IPCServer import start_server

def launch_motor_console(motor_id: str):
    """OS에 따라 새 콘솔 창에서 motor_console.py 실행"""
    if os.name == 'nt':
        subprocess.Popen(
            [sys.executable, 'MotorConsole.py', motor_id],
            creationflags=subprocess.CREATE_NEW_CONSOLE
        )
    else:
        # Linux GUI 환경
        try:
            subprocess.Popen([
                'lxterminal', '-e',
                f'{sys.executable} MotorConsole.py {motor_id}'
            ])
        except FileNotFoundError:
            # 헤드리스(tmux) 환경
            os.system(
                f'tmux new-session -d -s motor{motor_id} '
                f'"{sys.executable} MotorConsole.py {motor_id}"'
            )

def main():
    print('==== Motor Launcher (IPC 버전) ====')
    port, baud = input(
        "직렬 포트와 baudrate 입력 (예: COM4 115200): "
    ).split()
    baud = int(baud)

    # 1) IPC 서버 백그라운드로 시작
    server_thread = threading.Thread(
        target=start_server, args=(port, baud), daemon=True
    )
    server_thread.start()

    print(f'IPC 서버 실행 중 → localhost:9999, 시리얼 {port}@{baud}')

    # 2) 사용자로부터 ID 리스트 입력받아 각 창 실행
    while True:
        cmd = input(
            "제어할 모터 ID 리스트 입력(예: 1,2,3; 종료는 exit): "
        ).strip().lower()
        if cmd == 'exit':
            break
        ids = [mid.strip() for mid in cmd.split(',') if mid.strip().isdigit()]
        for mid in ids:
            launch_motor_console(mid)

if __name__ == '__main__':
    main()

# launcher.py
import os
import sys
import threading
import subprocess
from IPCServer import start_server
from MotorConsole import run_console

def launch_motor_console(motor_id: str):
    """OS에 따라 새 콘솔 창에서 motor_console.py 실행"""
    # PyInstaller로 묶인 EXE인지 확인
    is_frozen = getattr(sys, 'frozen', False)

    if os.name == 'nt':
        if is_frozen:
            # EXE로 배포된 상태: 번들 내부 모드 분기
            cmd = [sys.executable, 'console', motor_id]
        else:
            # 개발 모드: 스크립트 파일 경로를 함께 넘겨야 호출 가능
            launcher_py = os.path.abspath(sys.argv[0])
            cmd = [sys.executable, launcher_py, 'console', motor_id]

        subprocess.Popen(
            cmd,
            creationflags=subprocess.CREATE_NEW_CONSOLE
        )
    else:
        # Linux GUI 환경
        try:
            # EXE 모드든 개발 모드든, 내부 모드 분기로 통일
            if is_frozen:
                linux_cmd = [sys.executable, 'console', motor_id]
            else:
                launcher_py = os.path.abspath(sys.argv[0])
                linux_cmd = [sys.executable, launcher_py, 'console', motor_id]

            subprocess.Popen(['lxterminal', '-e', *linux_cmd])
        except FileNotFoundError:
            # 헤드리스(tmux) 환경
            tmux_cmd = (
                f'tmux new-session -d -s motor{motor_id} '
                f'"{" ".join(linux_cmd)}"'
            )
            os.system(tmux_cmd)

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
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == 'server':
            port, baud = sys.argv[2], int(sys.argv[3])
            start_server(port, baud)
        elif mode == 'console':
            # ✔ 여기서 직접 콘솔 로직 실행
            run_console(int(sys.argv[2]))
        else:
            print(f"Unknown mode: {mode}")
            sys.exit(1)
    else:
        main()

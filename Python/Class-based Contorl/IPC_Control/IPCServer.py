# ipc_server.py
import socket
import threading
import json
from SerialPortClass import SerialPort

HOST = 'localhost'
PORT = 9999

def handle_client(conn, sp):
    """클라이언트 요청을 받아 실제 SerialPort로 읽기/쓰기를 수행"""
    f = conn.makefile(mode='rwb')
    while True:
        try:
            line = f.readline()
            if not line:
                # 클라이언트가 정상 종료했을 때
                break

            msg = json.loads(line.decode('utf-8'))
            if msg.get('type') == 'command':
                sp.send_command(msg['payload'])
                resp = {'status': 'ok'}
            elif msg.get('type') == 'read':
                data = sp.read_response()
                resp = {'response': data}
            else:
                resp = {'error': 'Unknown message type'}

        except ConnectionResetError:
            # 클라이언트가 강제 종료했을 때 조용히 빠져나감
            break
        except Exception as e:
            # 기타 에러는 클라이언트로 에러 메시지 전송
            resp = {'error': str(e)}
            f.write((json.dumps(resp) + '\n').encode('utf-8'))
            f.flush()
            continue

        # 정상 처리된 응답 전송
        f.write((json.dumps(resp) + '\n').encode('utf-8'))
        f.flush()

    # 소켓/리소스 정리
    try:
        f.close()
    except:
        pass
    conn.close()

def start_server(port: str, baudrate: int):
    """IPC 서버 시작 (한 번만 SerialPort 열기)"""
    sp = SerialPort(port=port, baudrate=baudrate, timeout=1)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    while True:
        conn, _ = server.accept()
        threading.Thread(target=handle_client, args=(conn, sp), daemon=True).start()

if __name__ == '__main__':
    import sys
    # 직접 실행 시 커맨드라인으로 포트·baud 입력 가능
    p, b = ('COM4', 115200)
    if len(sys.argv) == 3:
        p, b = sys.argv[1], int(sys.argv[2])
    start_server(p, b)

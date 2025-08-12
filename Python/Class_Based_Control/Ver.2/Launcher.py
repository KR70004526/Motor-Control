#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launcher — Integrated (Serial UX from TestLauncher, sessions/recorder/popups from v2)
- 포트 탐색→대화형 선택 (connect 묻지 않음: 실행 즉시 질문)
- start [제목] [csv|json] : 세션 시작 (logs/DATE/TITLE/ 하위에 Log_*.txt + data_*.csv|jsonl)
- end : 세션 저장 종료 (CSV/JSON, TXT 모두 중단)
- windows : 팝업 수동 재오픈
- close/stop : 모든 팝업 종료 후 시리얼/스레드 정리
- ids / id / on|off|origin / update / get / show / where / help : 제어/조회
"""

from __future__ import annotations
import os, sys, re, csv, json, time, queue, threading, subprocess, shutil, shlex
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List, Tuple, TYPE_CHECKING

try:
    import serial
    from serial.tools import list_ports
except Exception:
    print("pyserial이 필요합니다:  pip install pyserial")
    raise

if TYPE_CHECKING:
    import serial as _serial
    SerialT = _serial.Serial
else:
    SerialT = object

# =========================
# Settings
# =========================
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -50.0, 50.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -25.0, 25.0

DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.02
WRITE_TIMEOUT = 0.02

ARDUINO_SUPPORTS_GET = True
ALLOW_PARTIAL_SEND  = True

AUTO_POPUP_ON_CONNECT = False   # ← 연결 직후 팝업 금지 (요청사항)
AUTO_POPUP_ON_START   = True    # start 시 LOG/DATA 팝업

#%%
# =========================
# SerialPort
# =========================
class SerialPort:
    def __init__(self):
        self._ser: Optional[SerialT] = None
        self._lock = threading.Lock()

    def connect(self, port: str, baud: int = DEFAULT_BAUD):
        self._ser = serial.Serial(
            port=port, baudrate=baud, timeout=READ_TIMEOUT, write_timeout=WRITE_TIMEOUT
        )
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

    def send_command(self, line: str):
        if not self._ser:
            raise RuntimeError("[SerialPort] Not connected")
        data = (line.strip() + "\n").encode("utf-8")
        with self._lock:
            self._ser.write(data)

    def read_line(self) -> str:
        if not self._ser:
            return ""
        try:
            raw = self._ser.readline()
            if not raw:
                return ""
            return raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            return ""

    def close(self):
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

#%%
# =========================
# IPCSerial (TX queue)
# =========================
class IPCSerial:
    def __init__(self, sp: SerialPort):
        self._sp = sp
        self._q: "queue.Queue[str]" = queue.Queue(maxsize=2048)
        self._running = threading.Event()
        self._th = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self._running.set()
        self._th.start()

    def stop(self):
        self._running.clear()
        try:
            self._q.put_nowait("__QUIT__")
        except Exception:
            pass
        try:
            if self._th.is_alive():
                self._th.join(timeout=1.0)
        except Exception:
            pass

    def send(self, cmd: str):
        self._q.put(cmd.strip())

    def _worker(self):
        while self._running.is_set():
            try:
                cmd = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if cmd == "__QUIT__":
                break
            try:
                self._sp.send_command(cmd)
            except Exception as e:
                print(f"[IPCSerial] send error: {e}")

#%%
# =========================
# Session / Recorder
# =========================
class Session:
    def __init__(self, title: Optional[str]=None, kind: str='csv'):
        self.kind = (kind or 'csv').lower()
        now = datetime.now()
        date_dir = now.strftime('%Y-%m-%d')
        base = (title or now.strftime('%H%M%S')).strip()
        safe = ''.join(c for c in base if c.isalnum() or c in ('-','_'))
        self.dir = Path('logs')/date_dir/safe
        self.dir.mkdir(parents=True, exist_ok=True)
        ts = now.strftime('%Y%m%d_%H%M%S')
        self.log_path  = self.dir/f'Log_{ts}.txt'
        self.data_path = self.dir/(f'data_{ts}.csv' if self.kind=='csv' else f'data_{ts}.jsonl')
        self.cmd_path  = self.dir/'commands.log'

@dataclass
class MotorState:
    pos: float = 0.0
    vel: float = 0.0
    tor: float = 0.0

class AsyncRecorder:
    """비동기 라이터 (CSV/JSONL 모두 라인 단위 기록)"""
    def __init__(self, session: Session):
        self.session = session
        self._q: "queue.Queue" = queue.Queue()
        self._running = False
        self._t: Optional[threading.Thread] = None
        self._fh = None
        self._is_csv = (session.kind == 'csv')
        self._csvw: Optional[csv.writer] = None
        self._ids: List[int] = []
        self._header_written = False

    def start(self):
        if self._is_csv:
            self._fh = open(self.session.data_path, "a", newline="", buffering=1, encoding="utf-8")
            self._csvw = csv.writer(self._fh)
        else:
            self._fh = open(self.session.data_path, "a", buffering=1, encoding="utf-8")
        self._running = True
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def enqueue_frame(self, ts_ms: int, frame: Dict[int, Tuple[float,float,float]], *, host_ms: Optional[int] = None):
        if self._running:
            # ("F", mcu_ms_abs, host_epoch_ms, frame)
            self._q.put(("F", ts_ms, host_ms, frame))

    def _run(self):
        while self._running or not self._q.empty():
            try:
                item = self._q.get(timeout=0.5)
            except queue.Empty:
                continue

            if isinstance(item, tuple) and item and item[0] == "F":
                _, ts_mcu_abs, host_ms, frame = item

                # 새 ID 등장하면 컬럼 확정 전 리스트에 추가
                for mid in sorted(frame.keys()):
                    if mid not in self._ids:
                        self._ids.append(mid)

                if self._is_csv:
                    # 헤더(파일 처음 쓸 때 1회)
                    if not self._header_written and self._fh.tell() == 0:
                        header = ["host_iso","mcu_ms_abs"]
                        for mid in self._ids:
                            header += [f"id{mid}_pos", f"id{mid}_vel", f"id{mid}_tor"]
                        self._csvw.writerow(header)
                        self._header_written = True

                    # 행 작성(없는 ID는 빈 칸)
                    host_iso = ""
                    if host_ms is not None:
                        host_iso = datetime.fromtimestamp(host_ms/1000, tz=None).isoformat(timespec='milliseconds')
                    row = [host_iso, ts_mcu_abs]
                    for mid in self._ids:
                        if mid in frame:
                            p, v, tq = frame[mid]
                            row += [round(p, 3), round(v, 3), round(tq, 3)]
                        else:
                            row += ["", "", ""]
                    self._csvw.writerow(row)
                    self._fh.flush()
                else:
                    # JSONL일 때도 프레임 단위로 한 줄에 묶어 저장
                    obj = {"host_epoch_ms": host_ms, "mcu_ms_abs": ts_mcu_abs}
                    for mid, (p, v, tq) in frame.items():
                        obj[str(mid)] = {"pos": p, "vel": v, "tor": tq}
                    self._fh.write(json.dumps(obj, ensure_ascii=False) + "\n")
                    self._fh.flush()

    def stop(self):
        self._running = False
        if self._t:
            self._t.join(timeout=2.0)
        if self._fh:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass

#%%
# =========================
# LogControlManager (RX, parse, cache, log, popups)
# =========================
ID_TRIPLE = re.compile(r'(\d+):([\-0-9.]+),([\-0-9.]+),([\-0-9.]+)')

class LogControlManager:
    def __init__(self, sp: SerialPort, log_path: Optional[str]=None):
        self._sp = sp
        self._running = threading.Event()
        self._th = threading.Thread(target=self._rx_worker, daemon=True)
        self._states: Dict[int, MotorState] = {}
        self._log_path = None
        self._log_fh = None
        self._rec: Optional[AsyncRecorder] = None
        self._lock = threading.Lock()
        self._logging_enabled = False          # 세션 중에만 파일 로그
        self._popups: List[subprocess.Popen] = []   # 열린 팝업 핸들

    # lifecycle
    def start(self):
        self._running.set()
        self._th.start()

    def stop(self):
        self._running.clear()
        if self._th.is_alive():
            try: self._th.join(timeout=1.0)
            except Exception: pass
        if self._log_fh:
            try:
                self._log_fh.flush(); self._log_fh.close()
            except Exception: pass

    # session hooks
    def enable_logging(self, enabled: bool):
        self._logging_enabled = enabled

    def attach_recorder(self, rec: Optional[AsyncRecorder]):
        with self._lock:
            self._rec = rec
        if rec:
            rec.start()

    def rotate_log(self, new_log_path: Path):
        try:
            self._log_fh.flush()
            self._log_fh.close()
        except Exception:
            pass
        self._log_path = Path(new_log_path).absolute()
        self._log_fh = open(self._log_path, 'a', encoding='utf-8')

    # accessors
    def get_state(self, mid: int) -> MotorState:
        with self._lock:
            return self._states.get(mid, MotorState())

    def get_log_path(self) -> Optional[Path]:
        return self._log_path

    # parse
    def _handle_time_frame(self, line: str):
        # ex) Time:26500, 1:0.1,0.2,0.0, 3:-0.5,0.0,0.01
        m = re.search(r'(?:^|,) *T(?:ime)?:\s*([0-9]+)', line)
        if not m:
            return
        t_ms = int(m.group(1))
        frame: Dict[int, Tuple[float,float,float]] = {}

        for g in ID_TRIPLE.finditer(line):
            mid = int(g.group(1))
            pos = float(g.group(2)); vel = float(g.group(3)); tor = float(g.group(4))
            with self._lock:
                self._states[mid] = MotorState(pos=pos, vel=vel, tor=tor)
            frame[mid] = (pos, vel, tor)
        
        host_ms = int(time.time() * 1000)
        with self._lock:
            rec = self._rec
        if rec and frame:
            rec.enqueue_frame(t_ms, frame, host_ms=host_ms)

    def _rx_worker(self):
        while self._running.is_set():
            line = self._sp.read_line()
            if not line:
                continue
            if self._logging_enabled and self._log_fh:
                try:
                    self._log_fh.write(line + "\n")
                    self._log_fh.flush()
                except Exception:
                    pass
            if line.startswith('Time:') or line.startswith('T:') or 'Time:' in line:
                self._handle_time_frame(line)

    # popups
    def start_tail_popups(self):
        # 세션 전에는 팝업 금지
        if self._log_path is None and (not self._rec or not self._rec.session):
            print("[info] 세션이 아직 없습니다. 'start' 후 사용하세요.")
            return
        def spawn_tail(path: str, title: str):
            if not path:
                return None
            if os.name == 'nt':
                return subprocess.Popen(
                    ["powershell", "-NoExit", "-Command",
                     f"Write-Host '{title} → {path}'; Get-Content -Path '{path}' -Wait"],
                    creationflags=subprocess.CREATE_NEW_CONSOLE
                )
            if sys.platform == 'darwin':
                esc_path = shlex.quote(path)
                osa = (
                    'tell application "Terminal"\n'
                    f'  do script "tail -f {esc_path}"\n'
                    '  activate\n'
                    'end tell'
                )
                return subprocess.Popen(["osascript", "-e", osa])
            for term in ("gnome-terminal","konsole","xfce4-terminal","xterm","x-terminal-emulator"):
                if shutil.which(term):
                    if term == "xterm":
                        return subprocess.Popen([term, "-T", title, "-hold", "-e", "tail", "-f", path])
                    return subprocess.Popen([term, "--", "bash", "-lc", f"echo '{title}'; tail -f '{path}'"])
            print(f"[warn] no GUI terminal found; run manually: tail -f '{path}'")
            return None

        # LOG
        if self._log_path:
            p = spawn_tail(str(self._log_path), "LOG")
            if p: self._popups.append(p)
        # DATA
        data_path = None
        with self._lock:
            if self._rec and self._rec.session:
                data_path = self._rec.session.data_path
        if data_path:
            p = spawn_tail(str(data_path), "DATA")
            if p: self._popups.append(p)

    def close_popups(self):
        for p in list(self._popups):
            try:
                p.terminate()
                p.wait(timeout=1.5)
            except Exception:
                pass
            if p.poll() is None and os.name == 'nt':
                subprocess.run(
                    ["taskkill", "/PID", str(p.pid), "/F", "/T"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
        self._popups.clear()

# =========================
# MotorBase
# =========================
class MotorBase:
    _sp: SerialPort = None
    _ipc: IPCSerial = None
    _rx:  LogControlManager = None
    _connected = False
    _last_cmds: Dict[int, Dict[str,float]] = {}

    @classmethod
    def connect(cls, port: str, baud: int=DEFAULT_BAUD):
        if cls._connected:
            return
        cls._sp = SerialPort(); cls._sp.connect(port, baud)
        cls._ipc = IPCSerial(cls._sp); cls._ipc.start()
        cls._rx  = LogControlManager(cls._sp); cls._rx.start()
        cls._connected = True

    @classmethod
    def close_all(cls):
        if cls._rx: cls._rx.stop()
        if cls._ipc: cls._ipc.stop()
        if cls._sp: cls._sp.close()
        cls._sp=cls._ipc=cls._rx=None; cls._connected=False

    def __init__(self, motor_id:int):
        if not MotorBase._connected: raise RuntimeError('MotorBase.connect() 먼저 호출')
        self.id = motor_id
        if motor_id not in MotorBase._last_cmds:
            MotorBase._last_cmds[motor_id] = {"pos":0.0,"vel":0.0,"kp":0.0,"kd":0.0,"tor":0.0}

    def _ensure_id(self): MotorBase._ipc.send(f"ID:{self.id}")
    def on(self):     self._ensure_id(); MotorBase._ipc.send("ON")
    def off(self):    self._ensure_id(); MotorBase._ipc.send("OFF")
    def origin(self): self._ensure_id(); MotorBase._ipc.send("ORIGIN")

    def set_mode(self, mode:str):
        if mode.upper()=="MIT": self.on()
        else: raise ValueError('MIT만 지원')

    def _guard(self, pos,vel,kp,kd,tor):
        def chk(x,lo,hi,name):
            if x is None: return None
            if x<lo or x>hi: raise ValueError(f"{name} 범위초과: {x} ∉ [{lo},{hi}]")
            return x
        return (chk(pos,P_MIN,P_MAX,'pos'), chk(vel,V_MIN,V_MAX,'vel'),
                chk(kp,KP_MIN,KP_MAX,'kp'), chk(kd,KD_MIN,KD_MAX,'kd'), chk(tor,T_MIN,T_MAX,'tor'))

    def update_parameters(self, pos=None, vel=None, kp=None, kd=None, tor=None):
        pos,vel,kp,kd,tor = self._guard(pos,vel,kp,kd,tor)
        cache = MotorBase._last_cmds[self.id]
        if ALLOW_PARTIAL_SEND:
            if pos is not None: cache['pos']=pos
            if vel is not None: cache['vel']=vel
            if kp  is not None: cache['kp']=kp
            if kd  is not None: cache['kd']=kd
            if tor is not None: cache['tor']=tor
            p,v,Kp,Kd,T = cache['pos'],cache['vel'],cache['kp'],cache['kd'],cache['tor']
        else:
            if None in (pos,vel,kp,kd,tor): raise ValueError('모두 필요(pos/vel/kp/kd/tor)')
            p,v,Kp,Kd,T = pos,vel,kp,kd,tor; cache.update({'pos':p,'vel':v,'kp':Kp,'kd':Kd,'tor':T})
        self._ensure_id(); MotorBase._ipc.send(f"SEND POS:{p},VEL:{v},KP:{Kp},KD:{Kd},TOR:{T}")

    def get_parameters(self) -> MotorState:
        st = MotorBase._rx.get_state(self.id)
        if ARDUINO_SUPPORTS_GET:
            self._ensure_id(); MotorBase._ipc.send("GET"); time.sleep(0.02); st = MotorBase._rx.get_state(self.id)
        return st

    @classmethod
    def rx(cls) -> LogControlManager: return cls._rx
    @classmethod
    def ipc(cls) -> IPCSerial: return cls._ipc

# =========================
# Console helpers
# =========================
HELP_TEXT = """
명령:
  ids 1,2,4 | id 4 | on [all]|off [all]|origin [all]
  update [id=<n>] pos=.. vel=.. kp=.. kd=.. tor=..
  get <id> | show | start [제목] [csv|json] | end | stop [all|<id>] | help | close
"""

def list_available_ports():
    try:
        return [(p.device, p.description) for p in list_ports.comports()]
    except Exception:
        return []

def prompt_pick_port(default="COM4"):
    ports = list_available_ports()
    if ports:
        print("사용 가능 포트:")
        for dev, desc in ports:
            print(f"  - {dev} ({desc})")
    p = input(f"COM 포트를 입력 (예: COM4) [기본 {default}] : ").strip() or default
    return p

def prompt_pick_baud(default=DEFAULT_BAUD):
    b = input(f"Baudrate 입력 [기본 {default}] : ").strip()
    return int(b) if b else default

def header_ui(port: str, baud: int, logf: Optional[Path]):
    print("\n==============================================")
    print(" AK70-10 MIT Console (Integrated)")
    print("==============================================")
    print(f" Serial   : {port} @ {baud}")
    print(f" LogFile  : {logf or '(start 후 생성)'}")
    print(" Input    : 'help' 입력")
    print("==============================================\n")

# =========================
# Commands
# =========================
_current_session: Optional[Session] = None
_current_rec: Optional[AsyncRecorder] = None

def parse_kv(tokens: List[str]):
    mid=None; vals={}
    for t in tokens:
        if '=' not in t: continue
        k,v = t.split('=',1); k=k.strip().lower(); v=v.strip()
        if k=='id':
            try: mid=int(v)
            except Exception: pass
        else:
            try: vals[k]=float(v)
            except Exception: pass
    return mid, vals

def cmd_start(args: List[str]):
    global _current_session, _current_rec
    title=None; kind='csv'
    for a in args:
        la=a.lower()
        if la in ('csv','json'): kind=la
        else: title = a if title is None else (title+" "+a)
    ses = Session(title=title, kind=kind)
    _current_session = ses
    rx = MotorBase.rx()
    rx.rotate_log(ses.log_path)
    rec = AsyncRecorder(ses)
    rx.attach_recorder(rec)
    rx.enable_logging(True)                     # 세션 로깅 on
    ts = datetime.now()
    with open(ses.cmd_path,'a',encoding='utf-8') as f:
        f.write(f"START {ts.isoformat(timespec='milliseconds')} title={title or ses.dir.name} kind={kind}\n")
    print(f"[session] started: {ses.dir} (kind={kind})")
    _current_rec = rec
    if AUTO_POPUP_ON_START:
        rx.start_tail_popups()                  # 여기서만 팝업

def cmd_end():
    global _current_session, _current_rec
    if not _current_session:
        print('[info] 세션 없음. start 먼저')
        return
    rx = MotorBase.rx()

    # 1) TXT 로그부터 끊고
    rx.enable_logging(False)

    # 2) 레코더를 RX에서 떼어내 새 프레임이 더는 enqueue되지 않게 함(레이스 차단)
    rx.attach_recorder(None)

    # 3) 기존 큐에 남아있는 프레임까지 flush/close
    rec = _current_rec
    if rec:
        rec.stop()

    # 4) END 마커 기록 (이제 파일들이 모두 닫힌 뒤이므로 경계가 잘 맞음)
    te = datetime.now()
    with open(_current_session.cmd_path, 'a', encoding='utf-8') as f:
        f.write(f"END   {te.isoformat(timespec='milliseconds')}\n")

    print(f"[session] saved: {_current_session.data_path}\n[session] log: {rx.get_log_path()}")
    _current_session = None
    _current_rec = None

def _soft_zero(m: "MotorBase", mid: int):
    # pos/vel/kp/kd/tor = 0 → 충돌 방지용 소프트 스톱
    m.update_parameters(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tor=0.0)
    print(f"[ok] stop -> id={mid} pos=0 vel=0 kp=0 kd=0 tor=0")

def cmd_stop_one(mid: int, motors: Dict[int, "MotorBase"]):
    m = motors.get(mid) or MotorBase(mid)
    motors[mid] = m
    try:
        _soft_zero(m, mid)
        m.origin()  # 개별 원점
        print(f"[ok] origin {mid}")
    except Exception as e:
        print(f"[warn] stop {mid}: {e}")

def cmd_stop_all(motors: Dict[int, "MotorBase"]):
    if not motors:
        print("[info] 모터 리스트가 비었습니다. 먼저 'ids ...'를 설정하세요.")
        return
    for mid, m in motors.items():
        try:
            _soft_zero(m, mid)
        except Exception as e:
            print(f"[warn] stop {mid}: {e}")
    try:
        MotorBase.ipc().send("ORIGIN ALL")  # 전체 원점
        print("[ok] ORIGIN ALL")
    except Exception as e:
        print(f"[warn] origin all: {e}")

# =========================
# Main loop
# =========================
def run_console():
    # 실행 즉시 포트/baud 입력
    port = prompt_pick_port()
    baud = prompt_pick_baud()
    MotorBase.connect(port, baud)
    header_ui(port, baud, MotorBase.rx().get_log_path())

    motors: Dict[int, MotorBase] = {}
    current_id: Optional[int] = None

    print("연결 완료. 'help' 확인 후 진행.\n")
    while True:
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[exit]"); break
        if not line: continue
        lo = line.lower(); tokens = lo.split(); cmd = tokens[0]
        try:
            if cmd=='help':
                print(HELP_TEXT)

            elif cmd=='ids':
                arg = line.partition(' ')[2].strip().replace(' ','')
                if not arg: print('usage: ids 1,2,4'); continue
                MotorBase.ipc().send(f'IDS:{arg}')
                for tok in arg.split(','):
                    try: motors[int(tok)] = MotorBase(int(tok))
                    except Exception: pass
                if motors:
                    current_id = sorted(motors.keys())[0]
                print(f"[ok] ids: {sorted(motors.keys())}")

            elif cmd=='id':
                if len(tokens)<2: print('usage: id <n>'); continue
                mid=int(tokens[1])
                if mid not in motors: motors[mid]=MotorBase(mid)
                motors[mid]._ensure_id(); current_id=mid; print(f"[ok] current id: {mid}")

            elif cmd in ('on','off','origin'):
                if len(tokens)>=2 and tokens[1]=='all':
                    MotorBase.ipc().send(cmd.upper()+" ALL"); print(f"[ok] {cmd} all")
                else:
                    tid=current_id
                    if len(tokens)>=2 and tokens[1].isdigit(): tid=int(tokens[1])
                    if tid is None: print("먼저 'id <n>' 또는 '<cmd> <id>'"); continue
                    m = motors.get(tid) or MotorBase(tid)
                    getattr(m, cmd)(); print(f"[ok] {cmd} {tid}")

            elif cmd=='update':
                rest=line.split()[1:]
                if rest and rest[0].isdigit(): rest[0]=f'id={rest[0]}'
                mid, kv = parse_kv(rest)
                if mid is None: mid=current_id
                if mid is None: print('usage: update <id> pos=.. vel=.. kp=.. kd=.. tor=..'); continue
                m = motors.get(mid) or MotorBase(mid)
                m.update_parameters(pos=kv.get('pos'), vel=kv.get('vel'), kp=kv.get('kp'), kd=kv.get('kd'), tor=kv.get('tor'))
                motors[mid]=m; print(f"[ok] update -> id={mid} {kv}")

            elif cmd=='get':
                if len(tokens)>=2 and tokens[1].isdigit():
                    mid=int(tokens[1])
                else:
                    mid=current_id
                if mid is None: print('usage: get <id>'); continue
                m = motors.get(mid) or MotorBase(mid)
                st = m.get_parameters(); print(f"id={mid} pos={st.pos:.3f} vel={st.vel:.3f} tor={st.tor:.3f}")

            elif cmd=='show':
                rx = MotorBase.rx(); ids = sorted(list(rx._states.keys()))
                if not ids: print('[info] 수신 상태 없음')
                for mid in ids:
                    st = rx.get_state(mid)
                    print(f"id={mid} pos={st.pos:.3f} vel={st.vel:.3f} tor={st.tor:.3f}")

            elif cmd=='start':
                args=line.split()[1:]; cmd_start(args)

            elif cmd=='end':
                cmd_end()
            
            elif cmd=='stop':
                # stop all / stop <id> / stop (현재 id)
                if len(tokens) >= 2 and tokens[1] == 'all':
                    cmd_stop_all(motors)
                else:
                    tid = None
                    if len(tokens) >= 2 and tokens[1].isdigit():
                        tid = int(tokens[1])
                    else:
                        tid = current_id
                    if tid is None:
                        print("usage: stop [all|<id>]  (예: stop 2)")
                        continue
                    cmd_stop_one(tid, motors)

            elif cmd=='close':
                cmd_end()  # 세션 저장 종료(로그/CSV flush & close) ← 먼저
                try:
                    MotorBase.rx().close_popups()  # 모든 팝업 종료
                except Exception:
                    pass
                break

            else:
                print('[err] unknown. help 참조')

        except ValueError as ve:
            print(f'[ValueError] {ve}')
        except Exception as e:
            print(f'[Error] {e}')

    try:
        MotorBase.rx().stop()
    except Exception:
        pass
    MotorBase.close_all()
    print('[ok] closed')

if __name__ == '__main__':
    run_console()

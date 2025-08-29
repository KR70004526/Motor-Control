# Launcher.py  (mode-corrected + runtime patch for mit_can)
# =============================================================================
# RPi4 + Ubuntu + Seengreat 2CH CAN HAT (SocketCAN: can0)
# TMotorCANControl (MIT mode) + console thread + CSV logging
# - Main thread: SoftRealtimeLoop 제어/수신/로깅
# - Console thread: 명령을 큐로 전달 → 메인 루프에서 적용
# - 모드-세터 정합성 보장, step()은 dev.update()만
# - Runtime 패치: on_message_received ID 매칭, _update_state_async dt 부호
# =============================================================================

import os
import csv
import time
import shlex
import signal
import queue
import threading
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

# --- Realtime loop (use main thread) ---
try:
    from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
except Exception as e:
    print(f"[WARN] NeuroLocoMiddleware not available ({e}). Using simple loop (lower precision).")
    class SoftRealtimeLoop:
        def __init__(self, dt: float, report: bool = False, fade: float = 0.0):
            self.dt = dt
        def __iter__(self):
            t0 = time.perf_counter(); n = 0
            while True:
                yield time.perf_counter() - t0
                n += 1
                target = t0 + n * self.dt
                while True:
                    now = time.perf_counter()
                    if now >= target: break
                    time.sleep(min(0.001, max(0.0, target - now)))

# --- TMotor MIT CAN ---
from TMotorCANControl.mit_can import (
    TMotorManager_mit_can, MIT_Params, CAN_Manager, motorListener
)

# =============================================================================
# Runtime patches for TMotorCANControl.mit_can
# =============================================================================

def _patch_mit_can():
    """Fix two common issues at runtime:
       1) RX ID matching uses data[0] only; also accept arbitration_id.
       2) _update_state_async uses negative dt; fix sign and div-by-zero guard.
    """
    import types

    # Patch 1: motorListener.on_message_received
    def _patched_on_message_received(self, msg):
        data = bytes(msg.data)
        id_data = data[0] if len(data) > 0 else -1
        id_arb  = msg.arbitration_id & 0xFF
        if id_data == self.motor.ID or id_arb == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_MIT_message(data, self.motor.type))
    motorListener.on_message_received = _patched_on_message_received

    # Patch 2: TMotorManager_mit_can._update_state_async
    def _patched_update_state_async(self, MIT_state):
        if MIT_state.error != 0:
            raise RuntimeError('Driver board error for device: ' + self.device_info_string()
                               + ": " + MIT_Params['ERROR_CODES'][MIT_state.error])

        now = time.time()
        # Fix: positive dt + zero-division guard
        if not hasattr(self, "_last_update_time") or self._last_update_time is None:
            dt = 0.0
        else:
            dt = now - self._last_update_time
        self._last_update_time = now

        if dt > 1e-6:
            acceleration = (MIT_state.velocity - self._motor_state_async.velocity) / dt
        else:
            acceleration = 0.0

        # Convert TMotor 'current'(= torque est.) → q-axis current
        self._motor_state_async.set_state(
            MIT_state.position,
            MIT_state.velocity,
            self.TMotor_current_to_qaxis_current(MIT_state.current),
            MIT_state.temperature,
            MIT_state.error,
            acceleration
        )
        self._updated = True

    TMotorManager_mit_can._update_state_async = _patched_update_state_async

# Apply patches once
_patch_mit_can()
# Optional: enable CAN debug frames
# CAN_Manager.debug = True

# =============================================================================
# Utils / Data
# =============================================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def now_ms() -> int:
    return int(time.time() * 1000)

@dataclass
class MotorState:
    pos: float = 0.0   # rad
    vel: float = 0.0   # rad/s
    tau: float = 0.0   # Nm
    iq:  float = 0.0   # A
    temp: float = 0.0  # °C
    err: int = 0

# =============================================================================
# CSV Recorder
# =============================================================================

class CSVRecorder:
    def __init__(self, base_dir: str = "./logs"):
        self._base = base_dir
        os.makedirs(self._base, exist_ok=True)
        self._fh = None
        self._csv = None
        self._path = None
        self._lock = threading.Lock()
        self._running = False

    def start(self, label: str = "run") -> str:
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._base, f"{ts}_{label}.csv")
        with self._lock:
            self._fh = open(path, "w", newline="")
            self._csv = csv.writer(self._fh)
            self._csv.writerow(
                ["host_ms", "loop_ms", "motor_id", "pos_rad", "vel_rad_s", "tau_Nm", "iq_A", "temp_C", "err"]
            )
            self._path = path
            self._running = True
        return path

    def stop(self):
        with self._lock:
            self._running = False
            if self._fh:
                try: self._fh.flush()
                except Exception: pass
                self._fh.close()
            self._fh = None
            self._csv = None
            self._path = None

    def write_states(self, loop_ms: int, states: Dict[int, MotorState]):
        with self._lock:
            if not self._running or not self._csv:
                return
            hms = now_ms()
            for mid, st in states.items():
                self._csv.writerow([hms, loop_ms, mid, st.pos, st.vel, st.tau, st.iq, st.temp, st.err])

    @property
    def path(self) -> Optional[str]:
        return self._path

# =============================================================================
# LogControlManager (state cache + recorder)
# =============================================================================

class LogControlManager:
    def __init__(self):
        self._states: Dict[int, MotorState] = {}
        self._lock = threading.Lock()
        self._rec = CSVRecorder()
        self._recording = False

    def start_record(self, label: str = "run") -> str:
        path = self._rec.start(label)
        self._recording = True
        return path

    def stop_record(self):
        self._recording = False
        self._rec.stop()

    def ingest_states(self, loop_ms: int, states: Dict[int, MotorState]):
        with self._lock:
            for mid, st in states.items():
                self._states[mid] = st
        if self._recording:
            self._rec.write_states(loop_ms, states)

    def get_state(self, motor_id: int) -> MotorState:
        with self._lock:
            return self._states.get(motor_id, MotorState())

    def get_all_states(self) -> Dict[int, MotorState]:
        with self._lock:
            return dict(self._states)

    @property
    def log_path(self) -> Optional[str]:
        return self._rec.path

# =============================================================================
# TMotor device (mode-aware; MIT mode)
# =============================================================================

class TMotorDevice:
    """모드 규칙에 맞는 setter만 호출하고, step()은 dev.update()만 수행."""
    def __init__(self, motor_type: str, motor_id: int,
                 pos_lim: Tuple[float, float], vel_lim: Tuple[float, float], tau_lim: Tuple[float, float]):
        self.motor_type = motor_type
        self.motor_id = motor_id
        self.pos_lim = pos_lim
        self.vel_lim = vel_lim
        self.tau_lim = tau_lim

        self.dev = TMotorManager_mit_can(motor_type=motor_type, motor_ID=motor_id)

        self.enabled = False
        self.ctx_opened = False

        # command cache
        self.kp = 0.0; self.kd = 0.0
        self.pos = 0.0; self.vel = 0.0; self.tau = 0.0

        self.mode = "IDLE"  # IMPEDANCE | FULL_STATE | CURRENT | SPEED | IDLE

    # ---------- Mode helpers ----------
    def set_mode_impedance(self, kp: float, kd: float):
        self.kp, self.kd = kp, kd
        self.dev.set_impedance_gains_real_unit(K=kp, B=kd)
        self.mode = "IMPEDANCE"

    def set_mode_full_state(self, kp: float, kd: float):
        self.kp, self.kd = kp, kd
        self.dev.set_impedance_gains_real_unit_full_state_feedback(K=kp, B=kd)
        self.mode = "FULL_STATE"

    def set_mode_current(self):
        self.dev.set_current_gains()
        self.mode = "CURRENT"

    def set_mode_speed(self, kd: float = 1.0):
        self.dev.set_speed_gains(kd=kd)
        self.mode = "SPEED"

    def set_gains(self, kp: float, kd: float, full_state: bool = False):
        if full_state: self.set_mode_full_state(kp, kd)
        else:          self.set_mode_impedance(kp, kd)

    # ---------- Lifecycle ----------
    def on(self):
        if not self.ctx_opened:
            self.dev.__enter__(); self.ctx_opened = True
        self.dev.power_on()
        # 기본 임피던스로 진입(미설정 시)
        if self.mode == "IDLE":
            self.set_mode_impedance(kp=10.0, kd=0.5)
        # 현재 위치로 홀드 (점프 방지)
        try: self.pos = self.dev.get_output_angle_radians()
        except Exception: pass
        try:
            self.dev.position = self.pos
            self.dev.update()
        except Exception: pass
        self.enabled = True

    def off(self):
        if self.enabled:
            try:
                if self.mode in ("FULL_STATE", "SPEED"): self.dev.velocity = 0.0
                if self.mode in ("FULL_STATE", "CURRENT"): self.dev.torque = 0.0
                self.dev.update()
            except Exception: pass
            self.dev.power_off()
        self.enabled = False

    def close(self):
        try:
            if self.enabled: self.off()
            if self.ctx_opened: self.dev.__exit__(None, None, None)
        except Exception: pass
        self.ctx_opened = False; self.mode = "IDLE"

    def origin(self):
        try:
            if self.mode in ("FULL_STATE", "SPEED"): self.dev.velocity = 0.0
            if self.mode in ("FULL_STATE", "CURRENT"): self.dev.torque = 0.0
            self.dev.update()
        except Exception: pass
        self.dev.set_zero_position()
        time.sleep(1.0)

    # ---------- Command ----------
    def set_command(self, pos=None, vel=None, tor=None):
        if pos is not None: self.pos = clamp(pos, *self.pos_lim)
        if vel is not None: self.vel = clamp(vel, *self.vel_lim)
        if tor is not None: self.tau = clamp(tor, *self.tau_lim)

        try:
            if self.mode == "FULL_STATE":
                if pos is not None: self.dev.position = self.pos
                if vel is not None: self.dev.velocity = self.vel
                if tor is not None: self.dev.torque   = self.tau
            elif self.mode == "IMPEDANCE":
                if pos is not None: self.dev.position = self.pos
            elif self.mode == "SPEED":
                if vel is not None: self.dev.velocity = self.vel
            elif self.mode == "CURRENT":
                if tor is not None: self.dev.torque   = self.tau
            # IDLE: do nothing
        except Exception as e:
            print(f"[WARN] set_command mid={self.motor_id} mode={self.mode} "
                  f"(pos={pos}, vel={vel}, tor={tor}) failed: {e}")

    def step(self):
        self.dev.update()

    def read_state(self) -> MotorState:
        try:
            p    = self.dev.get_output_angle_radians()
            v    = self.dev.get_output_velocity_radians_per_second()
            tau  = self.dev.get_output_torque_newton_meters()
            iq   = self.dev.get_current_qaxis_amps()
            temp = self.dev.get_temperature_celsius()
            err  = self.dev.get_motor_error_code()
            return MotorState(p, v, tau, iq, temp, err)
        except Exception:
            return MotorState(self.pos, self.vel, self.tau, 0.0, 0.0, 0)

# =============================================================================
# TMotorPool (stepped by main loop)
# =============================================================================

class TMotorPool:
    def __init__(self, rx: LogControlManager, motor_type: str = "AK70-10"):
        self.rx = rx
        self.motor_type = motor_type
        self.devs: Dict[int, TMotorDevice] = {}

        mp = MIT_Params.get(motor_type, MIT_Params.get("AK70-10"))
        self.pos_lim = (float(mp["P_min"]), float(mp["P_max"]))
        self.vel_lim = (float(mp["V_min"]), float(mp["V_max"]))
        self.tau_lim = (float(mp["T_min"]), float(mp["T_max"]))
        self.kp_lim  = (float(mp["Kp_min"]), float(mp["Kp_max"]))
        self.kd_lim  = (float(mp["Kd_min"]), float(mp["Kd_max"]))

    def add(self, motor_id: int):
        if motor_id not in self.devs:
            self.devs[motor_id] = TMotorDevice(
                self.motor_type, motor_id, self.pos_lim, self.vel_lim, self.tau_lim
            )

    def remove_all(self):
        for d in list(self.devs.values()):
            d.close()
        self.devs.clear()

    # proxies
    def on(self, motor_id: int):     self.devs[motor_id].on()
    def off(self, motor_id: int):    self.devs[motor_id].off()
    def origin(self, motor_id: int): self.devs[motor_id].origin()

    def set_gains(self, motor_id: int, kp: float, kd: float, full_state: bool = False):
        kp = clamp(kp, *self.kp_lim); kd = clamp(kd, *self.kd_lim)
        self.devs[motor_id].set_gains(kp, kd, full_state=full_state)

    def set_mode_current(self, motor_id: int):
        self.devs[motor_id].set_mode_current()

    def set_mode_speed(self, motor_id: int, kd: float = 1.0):
        kd = clamp(kd, *self.kd_lim)
        self.devs[motor_id].set_mode_speed(kd=kd)

    def set_command(self, motor_id: int, pos=None, vel=None, tor=None):
        self.devs[motor_id].set_command(pos, vel, tor)

    def get_state(self, motor_id: int) -> MotorState:
        return self.devs[motor_id].read_state()

    def step_once(self) -> Dict[int, MotorState]:
        states: Dict[int, MotorState] = {}
        for mid, dev in self.devs.items():
            if not dev.enabled:
                continue
            try:
                dev.step()
                st = dev.read_state()
                states[mid] = st
            except Exception as e:
                print(f"[WARN] step_once mid={mid}: {e}")
        if not states:
            # Helpful hint when no RX frames are seen
            pass  # uncomment for verbose: print("[WARN] no states this tick (no RX?)")
        return states

# =============================================================================
# High-level wrapper
# =============================================================================

class MotorBase:
    _pool: Optional[TMotorPool] = None
    _rx: Optional[LogControlManager] = None
    _connected = False
    _last_cmds: Dict[int, Dict[str, float]] = {}

    @classmethod
    def connect(cls, motor_type: str = "AK70-10"):
        if cls._connected:
            return
        cls._rx = LogControlManager()
        cls._pool = TMotorPool(rx=cls._rx, motor_type=motor_type)
        cls._connected = True

    @classmethod
    def close_all(cls):
        if cls._pool:
            cls._pool.remove_all()
        cls._pool = None
        cls._rx = None
        cls._connected = False

    def __init__(self, motor_id: int):
        if not MotorBase._connected:
            raise RuntimeError("MotorBase.connect() 먼저 호출하세요.")
        self.id = motor_id
        MotorBase._pool.add(motor_id)
        if motor_id not in MotorBase._last_cmds:
            MotorBase._last_cmds[motor_id] = {"pos": 0.0, "vel": 0.0, "kp": 0.0, "kd": 0.0, "tor": 0.0}

    def on(self):     MotorBase._pool.on(self.id)
    def off(self):    MotorBase._pool.off(self.id)
    def origin(self): MotorBase._pool.origin(self.id)

    def update_parameters(self, pos=None, vel=None, kp=None, kd=None, tor=None):
        cache = MotorBase._last_cmds[self.id]
        if pos is not None: cache["pos"] = float(pos)
        if vel is not None: cache["vel"] = float(vel)
        if kp  is not None: cache["kp"]  = float(kp)
        if kd  is not None: cache["kd"]  = float(kd)
        if tor is not None: cache["tor"] = float(tor)

        want_k = (kp is not None) or (kd is not None)
        want_t = (tor is not None)
        want_v = (vel is not None)
        chosen = None
        dev = MotorBase._pool.devs[self.id]

        if want_k and want_t:
            MotorBase._pool.set_gains(self.id, cache["kp"], cache["kd"], full_state=True)
            chosen = "FULL_STATE"
        elif want_k:
            MotorBase._pool.set_gains(self.id, cache["kp"], cache["kd"], full_state=False)
            chosen = "IMPEDANCE"
        elif want_t:
            MotorBase._pool.set_mode_current(self.id)
            chosen = "CURRENT"
        elif want_v:
            kd_eff = cache["kd"] if cache["kd"] != 0.0 else 1.0
            MotorBase._pool.set_mode_speed(self.id, kd=kd_eff)
            chosen = "SPEED"
        else:
            if dev.mode == "IDLE":
                dev.set_mode_impedance(kp=10.0, kd=0.5)
                chosen = "IMPEDANCE"

        MotorBase._pool.set_command(self.id, pos=cache["pos"], vel=cache["vel"], tor=cache["tor"])
        if chosen:
            print(f"[{self.id}] mode → {chosen}")

    def get_parameters(self) -> MotorState:
        return MotorBase._pool.get_state(self.id)

    @classmethod
    def rx(cls) -> LogControlManager:
        return cls._rx

# =============================================================================
# Console helpers (background thread → command queue)
# =============================================================================

HELP_TEXT = """
Commands:
  help
  ids <a,b,c>        : register motor ids
  id <n>             : select current id
  on / off / origin  : power on/off and zero (safe)
  update k=v ...     : set pos / vel / kp / kd / tor (Nm)
                       ex) update kp=30 kd=0.8 pos=0.2
                           update tor=1.2
                           update kp=30 kd=0.8 tor=1.0 pos=0.2 vel=0
                           update vel=2.0
  get [id]           : print state of id (default: current id)
  show               : print all states
  start <label>      : start CSV logging
  stop | end         : stop logging
  close              : power off motors and exit
"""

def header_ui(interface="can0", bitrate=1_000_000, log_path=None):
    print("=" * 76)
    print(" TMotor MIT CAN Console (mode-corrected, main-thread control)")
    print(f" Interface : {interface} @ {bitrate}")
    print(f" Logging   : {log_path or '-'}")
    print("=" * 76)

def parse_update_args(arg_str: str) -> Dict[str, float]:
    out = {}
    for tok in shlex.split(arg_str):
        if "=" in tok:
            k, v = tok.split("=", 1)
            k = k.strip().lower()
            try: out[k] = float(v)
            except Exception: pass
    return out

# Global command queue
cmd_q: "queue.Queue[Tuple[str,str]]" = queue.Queue()

def console_thread():
    print("연결 완료. 'help'로 명령을 확인하세요.\n")
    while True:
        try:
            line = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            line = "close"
        if not line:
            continue
        cmd, *rest = line.split(maxsplit=1)
        arg = rest[0] if rest else ""
        cmd_q.put((cmd.lower(), arg))
        if cmd.lower() == "close":
            break

# =============================================================================
# Main
# =============================================================================

def run_main_precision():
    MOTOR_TYPE = "AK70-10"  # 사용 모델에 맞게
    DT = 0.005              # 200 Hz

    MotorBase.connect(motor_type=MOTOR_TYPE)
    header_ui("can0", 1_000_000, MotorBase.rx().log_path)

    motors: Dict[int, MotorBase] = {}
    current_id: Optional[int] = None

    th = threading.Thread(target=console_thread, daemon=True, name="ConsoleThread")
    th.start()

    loop = SoftRealtimeLoop(dt=DT, report=False, fade=0.0)
    t0 = time.monotonic()
    running = True

    try:
        for _ in loop:
            # 1) drain console commands
            while True:
                try:
                    cmd, arg = cmd_q.get_nowait()
                except queue.Empty:
                    break

                # print(f"[CMD] {cmd} {arg}")  # 필요 시 디버그

                if cmd in ("help", "?"):
                    print(HELP_TEXT)

                elif cmd == "ids":
                    try:
                        raw = arg.replace(",", " ").split()
                        for tok in raw:
                            mid = int(tok)
                            if mid not in motors:
                                motors[mid] = MotorBase(mid)
                        if raw:
                            current_id = int(raw[0])
                        print(f"등록된 모터: {sorted(motors.keys())}, current id = {current_id}")
                    except Exception:
                        print("형식: ids 1,2,3")

                elif cmd == "id":
                    try:
                        n = int(arg.strip())
                        if n not in motors:
                            motors[n] = MotorBase(n)
                        current_id = n
                        print(f"current id = {current_id}")
                    except Exception:
                        print("형식: id 1")

                elif cmd == "on":
                    if current_id is None:
                        print("먼저 id를 선택하세요."); continue
                    motors[current_id].on(); print(f"[{current_id}] ON")

                elif cmd == "off":
                    if current_id is None:
                        print("먼저 id를 선택하세요."); continue
                    motors[current_id].off(); print(f"[{current_id}] OFF")

                elif cmd == "origin":
                    if current_id is None:
                        print("먼저 id를 선택하세요."); continue
                    motors[current_id].origin(); print(f"[{current_id}] ORIGIN done")

                elif cmd == "update":
                    if current_id is None:
                        print("먼저 id를 선택하세요."); continue
                    kv = parse_update_args(arg)
                    motors[current_id].update_parameters(
                        pos=kv.get("pos"), vel=kv.get("vel"),
                        kp=kv.get("kp"), kd=kv.get("kd"), tor=kv.get("tor"))
                    print(f"[{current_id}] updated {kv}")

                elif cmd == "get":
                    target = current_id
                    if arg.strip():
                        try: target = int(arg.strip())
                        except Exception: pass
                    if target is None:
                        print("먼저 id를 선택하세요."); continue
                    st = motors[target].get_parameters()
                    print(f"[{target}] pos={st.pos:+.4f} rad  vel={st.vel:+.3f} rad/s  "
                          f"tau={st.tau:+.3f} Nm  iq={st.iq:+.3f} A  T={st.temp:.1f}°C  err={st.err}")

                elif cmd == "show":
                    states = MotorBase.rx().get_all_states()
                    if not states:
                        print("상태 없음")
                    for mid in sorted(states.keys()):
                        st = states[mid]
                        print(f"[{mid}] pos={st.pos:+.4f} rad  vel={st.vel:+.3f} rad/s  "
                              f"tau={st.tau:+.3f} Nm  iq={st.iq:+.3f} A  T={st.temp:.1f}°C  err={st.err}")

                elif cmd == "start":
                    label = arg.strip() or "run"
                    path = MotorBase.rx().start_record(label)
                    print(f"CSV logging → {path}")

                elif cmd in ("stop", "end"):
                    MotorBase.rx().stop_record()
                    print("logging stopped")

                elif cmd == "close":
                    print("모터 종료 및 종료 중..."); running = False

                else:
                    print("알 수 없는 명령입니다. 'help'를 입력하세요.")

            if not running:
                break

            # 2) one control tick (main thread)
            states = MotorBase._pool.step_once()
            if states:
                loop_ms = int((time.monotonic() - t0) * 1000)
                MotorBase.rx().ingest_states(loop_ms, states)

    except Exception as e:
        print(f"[ERR] main loop exception: {e}")
    finally:
        # 안전 종료
        try:
            pool = MotorBase._pool
            if pool:
                for mid, dev in list(pool.devs.items()):
                    try: dev.off()
                    except Exception: pass
        finally:
            MotorBase.close_all()
        print("bye.")

# =============================================================================
# Entry
# =============================================================================

if __name__ == "__main__":
    def _sigint(_sig, _frm):
        print("\n(SIGINT) -> close")
        cmd_q.put(("close", ""))  # 안전 종료
    signal.signal(signal.SIGINT, _sigint)

    run_main_precision()

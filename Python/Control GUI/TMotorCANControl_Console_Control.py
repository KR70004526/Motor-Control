# Launcher.py
# =============================================================================
# RPi4 + Ubuntu + Seengreat 2CH CAN HAT (SocketCAN: can0)
# TMotorCANControl (MIT mode) multi-motor console + logger
# -----------------------------------------------------------------------------
# Features
# - Multi-motor control over CAN (MIT mode)
# - Console: ids / id / on / off / origin / update / get / show / start / stop / end / close
# - CSV logging: host_ms, loop_ms, motor_id, pos, vel, tau(Nm), iq(A), temp(C), err
# - Safe sequence: power_on/off if available, zero with 0-torque, torque clamp by MIT_Params
# - Gains once; update pos/vel/torque each loop
# -----------------------------------------------------------------------------
# Prereqs:
#   sudo ip link set can0 up type can bitrate 1000000
#   pip install TMotorCANControl NeuroLocoMiddleware
# Run:
#   python Launcher.py
# =============================================================================

import os
import csv
import time
import shlex
import signal
import threading
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from TMotorCANControl.mit_can import TMotorManager_mit_can

# Pull model params (limits, torque constants) from library
try:
    from TMotorCANControl.mit_can import MIT_Params
except Exception:
    MIT_Params = {
        "AK70-10": {
            "P_min": -12.5, "P_max": 12.5,
            "V_min": -50.0, "V_max": 50.0,
            "T_min": -25.0, "T_max": 25.0,
            "Kp_min": 0.0, "Kp_max": 500.0,
            "Kd_min": 0.0, "Kd_max": 5.0,
        }
    }

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
    tau: float = 0.0   # Nm  (output torque)
    iq:  float = 0.0   # A   (q-axis current)
    temp: float = 0.0  # °C
    err: int = 0       # driver error code

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
                try:
                    self._fh.flush()
                except Exception:
                    pass
                self._fh.close()
            self._fh = None
            self._csv = None
            self._path = None

    def write_frame(self, loop_ms: int, states: Dict[int, MotorState]):
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
            self._rec.write_frame(loop_ms, states)

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
# TMotor device & pool (MIT mode)
# =============================================================================

class TMotorDevice:
    """One physical actuator (MIT mode)."""
    def __init__(self, motor_type: str, motor_id: int,
                 pos_lim: Tuple[float, float], vel_lim: Tuple[float, float], tau_lim: Tuple[float, float]):
        self.motor_type = motor_type
        self.motor_id = motor_id
        self.pos_lim = pos_lim
        self.vel_lim = vel_lim
        self.tau_lim = tau_lim

        self.dev = TMotorManager_mit_can(motor_type=motor_type, motor_ID=motor_id)

        # Runtime flags
        self.ctx_opened = False
        self.enabled = False

        # Command cache
        self.kp = 0.0
        self.kd = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.tau = 0.0   # Nm (we command torque; lib converts to current)

    # ----- Lifecycle -----
    def on(self):
        if not self.ctx_opened:
            self.dev.__enter__()   # open CAN + init internal state
            self.ctx_opened = True
        # (optional but preferred) power_on if available
        if hasattr(self.dev, "power_on"):
            try:
                self.dev.power_on()
            except Exception:
                pass
        # quick connection sanity check
        try:
            ok = self.dev.check_can_connection()
            if not ok:
                print(f"[WARN] motor {self.motor_id}: CAN ping failed")
        except Exception:
            pass
        self.enabled = True

    def off(self):
        if self.enabled:
            # send a safe zero command first
            try:
                self.dev.set_output_velocity_radians_per_second(0.0)
                self.dev.set_output_torque_newton_meters(0.0)
                self.dev.update()
            except Exception:
                pass
            # power_off or exit motor mode
            if hasattr(self.dev, "power_off"):
                try:
                    self.dev.power_off()
                except Exception:
                    pass
            elif hasattr(self.dev, "exit_motor_mode"):
                try:
                    self.dev.exit_motor_mode()
                except Exception:
                    pass
        self.enabled = False

    def close(self):
        try:
            if self.enabled:
                self.off()
            if self.ctx_opened:
                self.dev.__exit__(None, None, None)
        except Exception:
            pass
        self.ctx_opened = False

    # ----- Control API -----
    def origin(self):
        # zero with safe stop
        try:
            self.dev.set_output_velocity_radians_per_second(0.0)
            self.dev.set_output_torque_newton_meters(0.0)
            self.dev.update()
        except Exception:
            pass
        self.dev.set_zero_position()
        time.sleep(1.0)

    def set_gains(self, kp: float, kd: float):
        self.kp, self.kd = kp, kd
        self.dev.set_impedance_gains_real_unit(K=kp, B=kd)

    def set_command(self, pos=None, vel=None, tor=None):
        if pos is not None:
            self.pos = clamp(pos, *self.pos_lim)
        if vel is not None:
            self.vel = clamp(vel, *self.vel_lim)
        if tor is not None:  # torque in Newton-meters
            self.tau = clamp(tor, *self.tau_lim)

    def step(self):
        # send cached command; gains are applied when updated
        self.dev.set_output_angle_radians(self.pos)
        self.dev.set_output_velocity_radians_per_second(self.vel)
        self.dev.set_output_torque_newton_meters(self.tau)
        self.dev.update()

    def read_state(self) -> MotorState:
        try:
            p   = self.dev.get_output_angle_radians()
            v   = self.dev.get_output_velocity_radians_per_second()
            tau = self.dev.get_output_torque_newton_meters()
            iq  = self.dev.get_current_qaxis_amps()
            tC  = self.dev.get_temperature_celsius()
            err = self.dev.get_motor_error_code()
            return MotorState(p, v, tau, iq, tC, err)
        except Exception:
            # fall back to last command if read failed
            return MotorState(self.pos, self.vel, self.tau, 0.0, 0.0, 0)

class TMotorPool:
    def __init__(self, rx: LogControlManager, motor_type: str = "AK70-10", dt: float = 0.005):
        self.rx = rx
        self.motor_type = motor_type
        self.dt = dt
        self.devs: Dict[int, TMotorDevice] = {}
        self.running = threading.Event()
        self.th = threading.Thread(target=self._loop, daemon=True)

        # limits from MIT_Params (per model)
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

    def start(self):
        self.running.set()
        if not self.th.is_alive():
            self.th.start()

    def stop(self):
        self.running.clear()
        if self.th.is_alive():
            self.th.join(timeout=1.0)
        self.remove_all()

    # proxies for high-level
    def on(self, motor_id: int):     self.devs[motor_id].on()
    def off(self, motor_id: int):    self.devs[motor_id].off()
    def origin(self, motor_id: int): self.devs[motor_id].origin()

    def set_gains(self, motor_id: int, kp: float, kd: float):
        kp = clamp(kp, *self.kp_lim)
        kd = clamp(kd, *self.kd_lim)
        self.devs[motor_id].set_gains(kp, kd)

    def set_command(self, motor_id: int, pos=None, vel=None, tor=None):
        self.devs[motor_id].set_command(pos, vel, tor)

    def get_state(self, motor_id: int) -> MotorState:
        return self.devs[motor_id].read_state()

    # realtime loop
    def _loop(self):
        loop = SoftRealtimeLoop(dt=self.dt, report=False, fade=0.0)
        t0 = time.monotonic()
        for _ in loop:
            if not self.running.is_set():
                break
            loop_ms = int((time.monotonic() - t0) * 1000)
            states: Dict[int, MotorState] = {}
            for mid, dev in self.devs.items():
                if not dev.enabled:
                    continue
                try:
                    dev.step()                  # tx/rx
                    states[mid] = dev.read_state()
                except Exception:
                    # keep going on bus hiccup
                    pass
            if states:
                self.rx.ingest_states(loop_ms, states)

# =============================================================================
# High-level Motor wrapper & console
# =============================================================================

class MotorBase:
    _pool: Optional[TMotorPool] = None
    _rx: Optional[LogControlManager] = None
    _connected = False
    _last_cmds: Dict[int, Dict[str, float]] = {}

    @classmethod
    def connect(cls, motor_type: str = "AK70-10", dt: float = 0.005):
        if cls._connected:
            return
        cls._rx = LogControlManager()
        cls._pool = TMotorPool(rx=cls._rx, motor_type=motor_type, dt=dt)
        cls._pool.start()
        cls._connected = True

    @classmethod
    def close_all(cls):
        if cls._pool:
            cls._pool.stop()
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
        if tor is not None: cache["tor"] = float(tor)   # Nm

        if (kp is not None) or (kd is not None):
            MotorBase._pool.set_gains(self.id, cache["kp"], cache["kd"])
        MotorBase._pool.set_command(self.id, pos=cache["pos"], vel=cache["vel"], tor=cache["tor"])

    def get_parameters(self) -> MotorState:
        return MotorBase._pool.get_state(self.id)

    @classmethod
    def rx(cls) -> LogControlManager:
        return cls._rx

# =============================================================================
# Console helpers
# =============================================================================

HELP_TEXT = """
Commands:
  help
  ids <a,b,c>        : register motor ids
  id <n>             : select current id
  on / off / origin  : motor power on/off (mode) and set zero (safe)
  update k=v ...     : set any of pos, vel, kp, kd, tor (Nm)
                       e.g., update pos=0.3 vel=0 kp=30 kd=0.8 tor=1.5
  get [id]           : print state of id (default: current id)
  show               : print all states
  start <label>      : start CSV logging (./logs/<timestamp>_<label>.csv)
  stop | end         : stop logging
  close              : power off motors and exit
"""

def header_ui(interface="can0", bitrate=1_000_000, log_path=None):
    print("=" * 72)
    print(" TMotor MIT CAN Console")
    print(f" Interface : {interface} @ {bitrate}")
    print(f" Logging   : {log_path or '-'}")
    print("=" * 72)

def parse_update_args(arg_str: str) -> Dict[str, float]:
    out = {}
    for tok in shlex.split(arg_str):
        if "=" in tok:
            k, v = tok.split("=", 1)
            k = k.strip().lower()
            try:
                out[k] = float(v)
            except Exception:
                pass
    return out

def run_console():
    MOTOR_TYPE = "AK70-10"
    DT = 0.005  # 200 Hz

    MotorBase.connect(motor_type=MOTOR_TYPE, dt=DT)
    header_ui("can0", 1_000_000, MotorBase.rx().log_path)

    motors: Dict[int, MotorBase] = {}
    current_id: Optional[int] = None

    print("연결 완료. 'help'로 명령을 확인하세요.\n")
    while True:
        try:
            line = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            line = "close"

        if not line:
            continue

        cmd, *rest = line.split(maxsplit=1)
        cmd = cmd.lower()
        arg = rest[0] if rest else ""

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
                print("먼저 id를 선택하세요.")
                continue
            motors[current_id].on()
            print(f"[{current_id}] ON")

        elif cmd == "off":
            if current_id is None:
                print("먼저 id를 선택하세요.")
                continue
            motors[current_id].off()
            print(f"[{current_id}] OFF")

        elif cmd == "origin":
            if current_id is None:
                print("먼저 id를 선택하세요.")
                continue
            motors[current_id].origin()
            print(f"[{current_id}] ORIGIN done")

        elif cmd == "update":
            if current_id is None:
                print("먼저 id를 선택하세요.")
                continue
            kv = parse_update_args(arg)
            motors[current_id].update_parameters(
                pos=kv.get("pos"), vel=kv.get("vel"),
                kp=kv.get("kp"), kd=kv.get("kd"), tor=kv.get("tor"))
            print(f"[{current_id}] updated {kv}")

        elif cmd == "get":
            target = current_id
            if arg.strip():
                try:
                    target = int(arg.strip())
                except Exception:
                    pass
            if target is None:
                print("먼저 id를 선택하세요.")
                continue
            st = motors[target].get_parameters()
            print(
                f"[{target}] pos={st.pos:+.4f} rad  vel={st.vel:+.3f} rad/s  "
                f"tau={st.tau:+.3f} Nm  iq={st.iq:+.3f} A  T={st.temp:.1f}°C  err={st.err}"
            )

        elif cmd == "show":
            states = MotorBase.rx().get_all_states()
            if not states:
                print("상태 없음")
            for mid in sorted(states.keys()):
                st = states[mid]
                print(
                    f"[{mid}] pos={st.pos:+.4f} rad  vel={st.vel:+.3f} rad/s  "
                    f"tau={st.tau:+.3f} Nm  iq={st.iq:+.3f} A  T={st.temp:.1f}°C  err={st.err}"
                )

        elif cmd == "start":
            label = arg.strip() or "run"
            path = MotorBase.rx().start_record(label)
            print(f"CSV logging → {path}")

        elif cmd in ("stop", "end"):
            MotorBase.rx().stop_record()
            print("logging stopped")

        elif cmd == "close":
            print("모터 종료 및 종료 중...")
            for _, m in motors.items():
                try:
                    m.off()
                except Exception:
                    pass
            MotorBase.close_all()
            print("bye.")
            break

        else:
            print("알 수 없는 명령입니다. 'help'를 입력하세요.")

# =============================================================================
# Entry Point
# =============================================================================

if __name__ == "__main__":
    def _sigint(_sig, _frm):
        print("\n(SIGINT) -> close")
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, _sigint)

    try:
        run_console()
    except KeyboardInterrupt:
        try:
            MotorBase.close_all()
        except Exception:
            pass
        print("종료.")

# -*- coding: utf-8 -*-
"""
Created on Fri Jul 11 13:06:43 2025

@author: USER
"""

#%% Import Libraries
import os
import threading
import time
import logging
from subprocess import Popen, CREATE_NEW_CONSOLE
from MotorBaseClass import MotorBase

#%% Log Motor Control Manager Class
class LogControlManager:
    def __init__(self, port, baudrate, motor_id, log_file=None):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.log_file = log_file or f"Log_Motor{motor_id}.txt"
        self.polling = False
        self.stop_event = threading.Event()
        self._setup_logger()
        
        # Initialize Serial Communication
        MotorBase.init_serial(port=port, baudrate=baudrate, timeout=1)
        self.motor = MotorBase(motor_id=motor_id)
        self.logger.info("Serial Initialized & MotorBase Created")

    def _setup_logger(self):
        """ Configure per-motor loggers with both file and console handlers """
        self.logger = logging.getLogger(f"Motor{self.motor_id}")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False  # Prevent duplicate propagation to the parent logger

        # File Handler
        fh = logging.FileHandler(self.log_file, mode='w', encoding='utf-8')
        fh.setLevel(logging.INFO)
        fmt = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s',
                                datefmt='%Y-%m-%d %H:%M:%S')
        fh.setFormatter(fmt)
        self.logger.addHandler(fh)

        # Console (Input CMD) Handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(fmt)
        self.logger.addHandler(ch)

        self.logger.info(f"==== Motor {self.motor_id} LOG START ====")

    def start_log_window(self):
        """ Open a separate CMD window for monitoring the log file only once """
        log_path = os.path.abspath(self.log_file)
        if os.name == 'nt':
            # Windows: PowerShell을 새 콘솔로 실행
            ps_cmd = [
                "powershell",
                "-NoExit",
                "-Command",
                f"Get-Content -Path \"{log_path}\" -Wait"
            ]
            # CREATE_NEW_CONSOLE 플래그로 새 창에서 실행
            self.log_proc = Popen(ps_cmd, creationflags=CREATE_NEW_CONSOLE)
        else:
            # Ubuntu 등 Linux: 기존 로직 유지
            try:
                cmd = ['gnome-terminal', '--', 'bash', '-c', f'tail -f {log_path}; exec bash']
                self.log_proc = Popen(cmd)
            except FileNotFoundError:
                cmd = ['xterm', '-e', f'tail -f {log_path}']
                self.log_proc = Popen(cmd)

    def send_id(self):
        """ Sending Motor ID & Open Log Window """
        self.motor.send_id()
        self.logger.info("ID Sent to Motor")
        self.start_log_window()

    def start(self):
        """ Start Status Polling (log Recording) """
        if self.polling:
            self.logger.warning("Already Polling")
            return
        self.stop_event.clear()
        t = threading.Thread(target=self._poll_motor, args=(0.2,), daemon=True)
        t.start()
        self.polling = True
        self.logger.info("Polling Started")

    def end(self):
        """ Stop Status Polling """
        if not self.polling:
            self.logger.warning("Polling is not Active")
            return
        self.stop_event.set()
        self.polling = False
        self.logger.info("Polling Stopped")

    def save(self, filename: str):
        """ Backup Log File into Another File Name """
        if not filename.endswith(".txt"):
            filename += ".txt"
        os.rename(self.log_file, filename)
        self.logger.info(f"Log File Saved as {filename}")

    def stop(self):
        """ Emergency Stop ==> Set All Parameters into 0 """
        self.motor.update_parameters(pos=0, vel=0, kp=0, kd=0, tor=0)
        self.logger.info("Emergency Stop!!")

    def close(self):
        """ Stop Polling & End Process """
        if self.polling:
            self.stop_event.set()
        if hasattr(self, "log_proc"):
            try:
                self.log_proc.terminate()   # 우선 gentle terminate
                self.log_proc.wait(timeout=2)
            except Exception:
                self.log_proc.kill()
        self.logger.info("Closing Motor Instance")
        os._exit(0)

    def _poll_motor(self, interval: float):
        """ Periodically Invoke get_parameters & Log the Results """
        while not self.stop_event.is_set():
            try:
                pos, vel, tor = self.motor.get_parameters()
                self.logger.info(f"PARAM pos:{pos:.2f} vel:{vel:.2f} tor:{tor:.2f}")
            except Exception as e:
                self.logger.error(f"Error During Polling: {e}")
            time.sleep(interval)

    def command_loop(self):
        """ Process User Command in Input CMD """
        self.logger.info("Command loop started. Enter commands (start, end, save, stop, on, off, update_parameters, get_parameters, close)")
        while True:
            cmd = input(f"[Motor {self.motor_id}] >>> ").strip().lower()
            if cmd == "start":
                self.start()
            elif cmd == "end":
                self.end()
            elif cmd.startswith("save"):
                parts = cmd.split(maxsplit=1)
                if len(parts) == 2:
                    self.save(parts[1])
                else:
                    self.logger.warning("Usage: save [filename]")
            elif cmd == "stop":
                self.stop()
            elif cmd == "close":
                self.close()
            elif cmd == "on":
                self.motor.on()
                self.logger.info("Motor ON")
            elif cmd == "off":
                self.motor.off()
                self.logger.info("Motor OFF")
            elif cmd.startswith("update_parameters"):
                parts = cmd[len("update_parameters"):].strip().split()
                kwargs = {}
                for p in parts:
                    if "=" not in p:
                        continue
                    k, v = p.split("=", 1)
                    if v.lower() == "none":
                        # None으로 명시하면 해당 파라미터는 건너뜀
                        continue
                    try:
                        kwargs[k] = float(v)
                    except ValueError:
                        self.logger.error(f"Invalid value: {p}")
                        break
                if kwargs:
                    # MotorBase.update_parameters의 default None 처리 덕에,
                    # 전달되지 않은 키들은 내부에 저장된 마지막 값이 유지됩니다.
                    self.motor.update_parameters(**kwargs)
                    self.logger.info(f"update_parameters → applied: {kwargs}")
                else:
                    self.logger.warning("No valid parameters to update (all skipped or invalid).")
            elif cmd == "get_parameters":
                pos, vel, tor = self.motor.get_parameters()
                self.logger.info(f"MANUAL pos:{pos:.2f} vel:{vel:.2f} tor:{tor:.2f}")
            else:
                self.logger.warning("Unknown command")

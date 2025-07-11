# -*- coding: utf-8 -*-
"""
Created on Fri Jul 11 13:50:25 2025

@author: USER
"""


import os
import sys

def launch_motor(motor_id: int, port: str, baudrate: int):
    os.system(
        f'start "" cmd /k "python MotorInstance.py {motor_id} {port} {baudrate}"'
    )

def main():
    print("==== Motor Launcher ====")
    # 포트·baudrate 입력
    if len(sys.argv) == 3:
        port, baud = sys.argv[1], int(sys.argv[2])
    else:
        port, baud = input("Enter port and baudrate (e.g. COM4 115200): ").split()
        baud = int(baud)

    while True:
        cmd = input("Enter motor ID (or 'exit'): ").strip().lower()
        if cmd == "exit":
            break
        try:
            mid = int(cmd)
            launch_motor(mid, port, baud)
        except ValueError:
            print("Invalid ID. Please enter an integer.")

if __name__ == "__main__":
    main()

